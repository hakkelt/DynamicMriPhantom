// Host emulator of an ESP32-S3 for the segment_player firmware.
//
// The firmware's ESP32 code path (gptimer alarm callback, GPIO register
// writes, TTL interrupt) is compiled natively against the stub headers in
// stubs/ and runs on a virtual 240 MHz clock:
//   * the gptimer counts at its resolution from the virtual clock and calls the
//     alarm callback like an interrupt, ISR_LATENCY cycles after the alarm;
//   * interrupts preempt the main loop between "instructions" (every stub
//     call and every LOOP_COST cycles), never inside a critical section;
//   * reading the cycle counter costs cycles, so busy-waits take virtual time.
// It checks the logic and the integer timing of the ESP32 back end; it is not a
// cycle-accurate model of the Xtensa core (the AVR emulator is).
//
// Command line and stdin protocol are the same as emulator/avr/run_avr.mjs:
//   sim [--port 0] [--trace trace.txt] [--realtime] [--max-seconds 60]
//   prints "LISTENING <port>"; stdin: ttl [us] | button [ms] | time | flush | quit
// Trace lines are "<cycle> <pin> <level>" with pin dir, step, button or mri.
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <deque>
#include <string>
#include <vector>

#include "Arduino.h"
#include "driver/gptimer.h"
#include "esp_cpu.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

void setup();
void loop();

namespace {

const uint64_t kCpuHz = F_CPU;
const uint64_t kIsrLatency = 360;   // alarm/edge -> first instruction of the handler (1.5 us)
const uint64_t kIsrExit = 120;
const uint64_t kLoopCost = 480;     // one pass of loop() with nothing to do (2 us)
const uint64_t kCallCost = 24;      // any other stub call
const uint64_t kPollEvery = 24000;  // poll the socket and stdin every 100 us of virtual time

uint64_t g_cyc = 0;  // virtual clock (CPU cycles)
int g_crit = 0;
bool g_in_isr = false;

// Pins: STEP/DIR are GPIO outputs; BUTTON (pull-up) and MRI are driven from stdin.
const int kPinDir = 5, kPinStep = 6, kPinButton = 8, kPinMri = 9;
uint32_t g_out = 0;
int g_button_level = 1, g_mri_level = 0;
void (*g_mri_isr)() = nullptr;
bool g_mri_pending = false;
uint64_t g_mri_edge = 0;

// Trace
FILE* g_trace = nullptr;
std::vector<std::string> g_trace_pending;
long g_traced = 0;

void record(const char* pin, int level) {
    if (!g_trace) return;
    char buf[64];
    snprintf(buf, sizeof buf, "%llu %s %d", (unsigned long long)g_cyc, pin, level);
    g_trace_pending.push_back(buf);
}

void flush_trace() {
    if (!g_trace) return;
    for (auto& s : g_trace_pending) fprintf(g_trace, "%s\n", s.c_str());
    fflush(g_trace);
    g_traced += (long)g_trace_pending.size();
    g_trace_pending.clear();
}

// gptimer (one timer at a time, like the firmware uses)
struct Timer {
    bool exists = false, running = false, armed = false;
    uint64_t cyc_per_tick = 1, start = 0, alarm = 0;
    gptimer_alarm_cb_t cb = nullptr;
    void* ctx = nullptr;
} g_tm;

uint64_t timer_count(uint64_t cyc) {
    return g_tm.running ? (cyc - g_tm.start) / g_tm.cyc_per_tick : 0;
}

// Cycle at which the pending alarm fires (UINT64_MAX if none).
uint64_t alarm_cycle() {
    if (!g_tm.exists || !g_tm.running || !g_tm.armed || !g_tm.cb) return UINT64_MAX;
    return g_tm.start + g_tm.alarm * g_tm.cyc_per_tick;
}

uint64_t next_irq() {
    uint64_t t = alarm_cycle();
    if (g_mri_pending) t = std::min(t, g_mri_edge);
    return t == UINT64_MAX ? t : t + kIsrLatency;
}

void run_isrs();

// Let `d` cycles of main-context time pass, servicing interrupts on the way.
void advance(uint64_t d) {
    uint64_t end = g_cyc + d;
    if (g_in_isr || g_crit) {
        g_cyc = end;
        return;
    }
    for (;;) {
        uint64_t t = next_irq();
        if (t > end) break;
        g_cyc = std::max(g_cyc, t);
        run_isrs();
        end = std::max(end, g_cyc);
    }
    g_cyc = end;
}

void run_isrs() {
    if (g_in_isr || g_crit) return;
    for (;;) {
        uint64_t t = next_irq();
        if (t > g_cyc) return;
        g_in_isr = true;
        if (g_mri_pending && g_mri_edge + kIsrLatency <= g_cyc &&
            (alarm_cycle() == UINT64_MAX || g_mri_edge <= alarm_cycle())) {
            g_mri_pending = false;
            if (g_mri_isr) g_mri_isr();
        } else {
            g_tm.armed = false;  // one-shot until the callback sets a new alarm
            gptimer_alarm_event_data_t ev = {timer_count(g_cyc), g_tm.alarm};
            g_tm.cb((gptimer_handle_t)&g_tm, &ev, g_tm.ctx);
        }
        g_cyc += kIsrExit;
        g_in_isr = false;
    }
}

// Serial over TCP
int g_listen = -1, g_client = -1;
std::deque<uint8_t> g_rx;
std::vector<uint8_t> g_tx_pending;

void tx(const char* s, size_t n) {
    if (g_client >= 0) {
        while (n) {
            ssize_t w = send(g_client, s, n, MSG_NOSIGNAL);
            if (w <= 0) {
                if (errno == EINTR) continue;
                close(g_client);
                g_client = -1;
                return;
            }
            s += w;
            n -= (size_t)w;
        }
    } else {
        g_tx_pending.insert(g_tx_pending.end(), s, s + n);
    }
}

// stdin control
std::string g_stdin_buf;
bool g_quit = false;
struct Scheduled { uint64_t cyc; int what; };  // 0: MRI low, 1: button released
std::vector<Scheduled> g_sched;

void drive_mri(int level) {
    if (level && !g_mri_level) {
        g_mri_pending = true;
        g_mri_edge = g_cyc;
    }
    g_mri_level = level;
    record("mri", level);
}

void handle_command(const std::string& line) {
    char cmd[16] = {0};
    double arg = -1;
    if (sscanf(line.c_str(), "%15s %lf", cmd, &arg) < 1) return;
    std::string c = cmd;
    if (c == "ttl") {
        double us = arg > 0 ? arg : 100;
        drive_mri(1);
        g_sched.push_back({g_cyc + (uint64_t)(us * (kCpuHz / 1000000)), 0});
        printf("TTL %llu\n", (unsigned long long)g_cyc);
    } else if (c == "button") {
        double ms = arg > 0 ? arg : 20;
        g_button_level = 0;
        record("button", 0);
        g_sched.push_back({g_cyc + (uint64_t)(ms * (kCpuHz / 1000)), 1});
        printf("BUTTON %llu\n", (unsigned long long)g_cyc);
    } else if (c == "time") {
        printf("TIME %llu\n", (unsigned long long)g_cyc);
    } else if (c == "flush") {
        flush_trace();
        printf("FLUSHED %ld\n", g_traced);
    } else if (c == "quit") {
        g_quit = true;
    }
    fflush(stdout);
}

void poll_io() {
    fd_set rd;
    FD_ZERO(&rd);
    int maxfd = 0;
    FD_SET(0, &rd);
    FD_SET(g_listen, &rd);
    maxfd = std::max(maxfd, g_listen);
    if (g_client >= 0) {
        FD_SET(g_client, &rd);
        maxfd = std::max(maxfd, g_client);
    }
    timeval tv = {0, 0};
    if (select(maxfd + 1, &rd, nullptr, nullptr, &tv) <= 0) return;
    if (FD_ISSET(g_listen, &rd)) {
        int c = accept(g_listen, nullptr, nullptr);
        if (c >= 0) {
            if (g_client >= 0) close(g_client);
            g_client = c;
            int one = 1;
            setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            if (!g_tx_pending.empty()) {
                std::vector<uint8_t> p;
                p.swap(g_tx_pending);
                tx((const char*)p.data(), p.size());
            }
        }
    }
    if (g_client >= 0 && FD_ISSET(g_client, &rd)) {
        char buf[512];
        ssize_t n = recv(g_client, buf, sizeof buf, 0);
        if (n <= 0) {
            close(g_client);
            g_client = -1;
        } else {
            g_rx.insert(g_rx.end(), buf, buf + n);
        }
    }
    if (FD_ISSET(0, &rd)) {
        char buf[256];
        ssize_t n = read(0, buf, sizeof buf);
        if (n <= 0) {
            g_quit = true;
        } else {
            g_stdin_buf.append(buf, (size_t)n);
            size_t p;
            while ((p = g_stdin_buf.find('\n')) != std::string::npos) {
                handle_command(g_stdin_buf.substr(0, p));
                g_stdin_buf.erase(0, p + 1);
            }
        }
    }
}

void run_scheduled() {
    for (size_t i = 0; i < g_sched.size();) {
        if (g_sched[i].cyc <= g_cyc) {
            if (g_sched[i].what == 0) {
                g_mri_level = 0;
                record("mri", 0);
            } else {
                g_button_level = 1;
                record("button", 1);
            }
            g_sched.erase(g_sched.begin() + (long)i);
        } else {
            ++i;
        }
    }
}

double wall_seconds() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}

}  // namespace

// ---------------------------------------------------------------------------
// Stub implementations
// ---------------------------------------------------------------------------
HostSerial Serial;

void HostSerial::begin(unsigned long) {}
int HostSerial::available() {
    advance(kCallCost);
    return (int)g_rx.size();
}
int HostSerial::read() {
    advance(kCallCost);
    if (g_rx.empty()) return -1;
    int c = g_rx.front();
    g_rx.pop_front();
    return c;
}
size_t HostSerial::print(const char* s) {
    size_t n = strlen(s);
    tx(s, n);
    advance(kCallCost);
    return n;
}
size_t HostSerial::print(char c) {
    tx(&c, 1);
    return 1;
}
size_t HostSerial::print(long v) {
    char b[24];
    snprintf(b, sizeof b, "%ld", v);
    return print(b);
}
size_t HostSerial::print(unsigned long v) {
    char b[24];
    snprintf(b, sizeof b, "%lu", v);
    return print(b);
}

void pinMode(uint8_t, uint8_t) { advance(kCallCost); }

static void set_out(int pin, int level) {
    uint32_t bit = 1u << pin;
    uint32_t old = g_out;
    g_out = level ? (g_out | bit) : (g_out & ~bit);
    if (old != g_out) record(pin == kPinStep ? "step" : (pin == kPinDir ? "dir" : "gpio"), level);
}

void digitalWrite(uint8_t pin, uint8_t level) {
    set_out(pin, level);
    advance(kCallCost);
}

int digitalRead(uint8_t pin) {
    advance(kCallCost);
    if (pin == kPinButton) return g_button_level;
    if (pin == kPinMri) return g_mri_level;
    return (g_out >> pin) & 1;
}

void attachInterrupt(uint8_t pin, void (*isr)(void), int) {
    if (pin == kPinMri) g_mri_isr = isr;
}

void sim_enter_critical() { ++g_crit; }
void sim_exit_critical() {
    if (--g_crit == 0) run_isrs();
}

void sim_reg_write(uint32_t reg, uint32_t value) {
    for (int pin = 0; pin < 32; ++pin)
        if (value & (1u << pin)) set_out(pin, reg == GPIO_OUT_W1TS_REG);
    g_cyc += 4;
}

uint32_t esp_cpu_get_cycle_count(void) {
    g_cyc += 8;
    return (uint32_t)g_cyc;
}

esp_err_t gptimer_new_timer(const gptimer_config_t* cfg, gptimer_handle_t* ret) {
    g_tm = Timer();
    g_tm.exists = true;
    g_tm.cyc_per_tick = kCpuHz / cfg->resolution_hz;
    *ret = (gptimer_handle_t)&g_tm;
    return ESP_OK;
}
esp_err_t gptimer_del_timer(gptimer_handle_t) {
    g_tm = Timer();
    return ESP_OK;
}
esp_err_t gptimer_get_raw_count(gptimer_handle_t, uint64_t* value) {
    g_cyc += 40;
    *value = timer_count(g_cyc);
    return ESP_OK;
}
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t, const gptimer_event_callbacks_t* cbs, void* ctx) {
    g_tm.cb = cbs->on_alarm;
    g_tm.ctx = ctx;
    return ESP_OK;
}
esp_err_t gptimer_set_alarm_action(gptimer_handle_t, const gptimer_alarm_config_t* cfg) {
    g_cyc += 40;
    if (!cfg) {
        g_tm.armed = false;
        return ESP_OK;
    }
    // An alarm value already reached fires at once, like the hardware does.
    g_tm.alarm = std::max(cfg->alarm_count, timer_count(g_cyc));
    g_tm.armed = true;
    return ESP_OK;
}
esp_err_t gptimer_enable(gptimer_handle_t) { return ESP_OK; }
esp_err_t gptimer_disable(gptimer_handle_t) { return ESP_OK; }
esp_err_t gptimer_start(gptimer_handle_t) {
    g_tm.running = true;
    g_tm.start = g_cyc;
    return ESP_OK;
}
esp_err_t gptimer_stop(gptimer_handle_t) {
    g_tm.running = false;
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    int port = 0;
    bool realtime = false;
    double max_seconds = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) port = atoi(argv[++i]);
        else if (a == "--trace" && i + 1 < argc) g_trace = fopen(argv[++i], "w");
        else if (a == "--realtime") realtime = true;
        else if (a == "--max-seconds" && i + 1 < argc) max_seconds = atof(argv[++i]);
        else {
            fprintf(stderr, "unknown argument %s\n", a.c_str());
            return 2;
        }
    }
    g_listen = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(g_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);
    if (bind(g_listen, (sockaddr*)&addr, sizeof addr) || listen(g_listen, 1)) {
        perror("listen");
        return 1;
    }
    socklen_t len = sizeof addr;
    getsockname(g_listen, (sockaddr*)&addr, &len);
    printf("LISTENING %d\n", ntohs(addr.sin_port));
    fflush(stdout);

    setup();
    double t0 = wall_seconds();
    uint64_t next_poll = 0;
    while (!g_quit) {
        if (g_cyc >= next_poll) {
            poll_io();
            run_scheduled();
            next_poll = g_cyc + kPollEvery;
            if (max_seconds > 0 && g_cyc >= (uint64_t)(max_seconds * kCpuHz)) break;
            if (realtime) {
                double ahead = (double)g_cyc / kCpuHz - (wall_seconds() - t0);
                if (ahead > 0.002) usleep((useconds_t)(ahead * 1e6));
            }
        }
        loop();
        advance(kLoopCost);
    }
    flush_trace();
    printf("EXIT %llu\n", (unsigned long long)g_cyc);
    fflush(stdout);
    if (g_client >= 0) close(g_client);
    close(g_listen);
    return 0;
}
