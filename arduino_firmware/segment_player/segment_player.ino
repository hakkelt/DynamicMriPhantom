// segment_player: plays a precomputed step schedule with a hardware timer.
//
// The PC (python_serial/phantom_core.py) computes the exact time of every
// step and sends it as constant-jerk integer segments (see seg_engine.h).
// This firmware only plays them back: one timer interrupt per event, each
// compare value scheduled relative to the previous one, so interrupt latency
// never accumulates. No floats, no AccelStepper.
//
// Boards:
//   ATmega328P/328PB @ 16 MHz (Timer1, 16-bit): DIR=D2 STEP=D3 BUTTON=D5 MRI=D6
//   ESP32-S3 (gptimer, 54-bit; Arduino-ESP32 3.x): DIR=5 STEP=6 BUTTON=8 MRI=9
//   The ESP32 path is NOT yet tested on hardware.
//
// Serial protocol (115200 baud, ASCII lines, see README.md):
//   HELLO                                  -> CAPS {json}
//   LOAD <nseg> <tick_hz> <cycles> <trig>  -> OK LOAD          trig: ttl|button|now
//     ttl:    start on the first MRI TTL rising edge = start of the sequence (time zero)
//     button: debug stand-in for the MRI;  now: start at ARM (debug)
//   SEG <count> <interval> <add> <add2> <flags> -> OK <idx>
//   END <crc16>                            -> OK END <nseg> <crc16>
//   ARM [trig] [cycles]                    -> OK ARM <trig>     (then EV START / EV DONE)
//   STOP | FINISH | STATUS | ZERO | JOG <steps> <rate_hz>
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#if defined(ARDUINO_ARCH_ESP32)
#define SEG_ISR_ATTR IRAM_ATTR
#endif
#include "seg_engine.h"

#define FW_NAME "segment_player"
#define FW_VERSION "1.0.0"
#define PROTOCOL_VERSION 1
#define STEP_PULSE_US 3
#define DIR_SETUP_US 1

// ---------------------------------------------------------------------------
// Board configuration
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_AVR)
#include <avr/interrupt.h>
#include <avr/io.h>

#if defined(__AVR_ATmega328PB__)
#define BOARD_NAME "atmega328pb"
#else
#define BOARD_NAME "atmega328p"
#endif
// Arduino pin numbers (for CAPS) and the matching port bits (PORTD on Uno/Nano/Xplained Mini).
#define PIN_DIR 2
#define PIN_STEP 3
#define PIN_BUTTON 5
#define PIN_MRI 6
#define DIR_BIT _BV(PD2)
#define STEP_BIT _BV(PD3)
#define BUTTON_BIT _BV(PD5)
#define MRI_BIT _BV(PD6)
#define MAX_SEGMENTS 72
#define MAX_STEP_RATE_HZ 20000UL
#define MIN_EVENT_US 40
#define TRIGGER_LATENCY_US 7
#define TIMER_BITS 16
// Extra cycles after the step computation so STEP stays high >= STEP_PULSE_US.
// (the step computation itself already takes ~20 us)
#define PULSE_STAMP() 0
#define STEP_PULSE_PAD(t0) ((void)(t0), __builtin_avr_delay_cycles(16))
#define RX_BUFFER_BYTES SERIAL_RX_BUFFER_SIZE

#define STEP_HIGH() (PORTD |= STEP_BIT)
#define STEP_LOW() (PORTD &= (uint8_t)~STEP_BIT)
#define DIR_WRITE(v) do { if (v) PORTD |= DIR_BIT; else PORTD &= (uint8_t)~DIR_BIT; } while (0)
#define BUTTON_PRESSED() ((PIND & BUTTON_BIT) == 0)
#define CRIT_BEGIN() uint8_t sreg_ = SREG; cli()
#define CRIT_END() SREG = sreg_

static const uint16_t kPrescalers[] = {1, 8, 64, 256, 1024};
static const uint8_t kPrescalerBits[] = {_BV(CS10), _BV(CS11), _BV(CS11) | _BV(CS10), _BV(CS12),
                                         _BV(CS12) | _BV(CS10)};
#define N_TICK_RATES 5
#define TICK_RATE(i) ((uint32_t)(F_CPU / kPrescalers[i]))
#define DEFAULT_TICK_INDEX 1  // F_CPU / 8

#elif defined(ARDUINO_ARCH_ESP32)
#include "driver/gptimer.h"
#include "esp_cpu.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

#define BOARD_NAME "esp32s3"
#define PIN_DIR 5
#define PIN_STEP 6
#define PIN_BUTTON 8
#define PIN_MRI 9
#define MAX_SEGMENTS 4096
#define MAX_STEP_RATE_HZ 50000UL
#define MIN_EVENT_US 12
#define TRIGGER_LATENCY_US 2
#define TIMER_BITS 54
// Wait only what is left of the pulse after the step computation.
#define PULSE_STAMP() esp_cpu_get_cycle_count()
#define STEP_PULSE_PAD(t0) \
    while ((uint32_t)(esp_cpu_get_cycle_count() - (t0)) < STEP_PULSE_US * (F_CPU / 1000000UL)) {}
#define RX_BUFFER_BYTES 256  // HWCDC default

#define STEP_HIGH() REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << PIN_STEP)
#define STEP_LOW() REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << PIN_STEP)
#define DIR_WRITE(v) REG_WRITE((v) ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1UL << PIN_DIR)
#define BUTTON_PRESSED() (digitalRead(PIN_BUTTON) == LOW)
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
#define CRIT_BEGIN() portENTER_CRITICAL(&g_mux)
#define CRIT_END() portEXIT_CRITICAL(&g_mux)

static const uint32_t kTickRates[] = {40000000UL, 10000000UL, 4000000UL, 2000000UL, 1000000UL};
#define N_TICK_RATES 5
#define TICK_RATE(i) kTickRates[i]
#define DEFAULT_TICK_INDEX 3  // 2 MHz

#else
#error "Unsupported board: segment_player supports AVR (ATmega328P/PB) and ESP32"
#endif

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum : uint8_t { ST_IDLE, ST_LOADING, ST_READY, ST_ARMED, ST_RUNNING };
enum : uint8_t { TRIG_NOW, TRIG_TTL, TRIG_BUTTON };
enum : uint8_t { NOTE_START = 1, NOTE_DONE = 2 };

static seg::Segment g_segs[MAX_SEGMENTS];
static seg::Segment g_jog;
static seg::Engine g_eng;

static uint16_t g_nseg = 0;       // segments announced by LOAD
static uint16_t g_received = 0;   // segments received so far
static bool g_valid = false;      // a complete, checksummed program is loaded
static bool g_jogging = false;
static uint32_t g_tick_hz = 0;
static uint32_t g_cycles = 0;
static uint8_t g_trigger = TRIG_NOW;

static volatile uint8_t g_state = ST_IDLE;
static volatile int32_t g_pos = 0;
static volatile uint16_t g_overruns = 0;
static volatile uint8_t g_notes = 0;

// Pending event (owned by the timer interrupt while running).
static volatile uint8_t g_kind = seg::EV_DELAY;
static volatile uint8_t g_dir_level = 0;
static uint32_t g_first_ticks = 0;
static uint32_t g_margin = 2;  // ticks: minimum lead time for a compare value

static const char* trigger_name(uint8_t t) {
    return t == TRIG_TTL ? "ttl" : (t == TRIG_BUTTON ? "button" : "now");
}

static int parse_trigger(const char* s) {
    if (!s) return -1;
    if (!strcmp(s, "now")) return TRIG_NOW;
    if (!strcmp(s, "ttl")) return TRIG_TTL;
    if (!strcmp(s, "button")) return TRIG_BUTTON;
    return -1;
}

// Prepare the engine and the first event; called with the timer interrupt idle.
static void prepare_playback(const seg::Segment* segs, uint16_t n, uint32_t cycles) {
    g_eng.begin(segs, n, cycles);
    seg::Event ev;
    g_eng.next(ev);
    g_kind = ev.kind;
    g_first_ticks = ev.ticks;
    if (ev.kind == seg::EV_STEP) {
        g_dir_level = ev.dir;
        DIR_WRITE(ev.dir);
    }
}

static int tick_index(uint32_t hz) {
    for (uint8_t i = 0; i < N_TICK_RATES; ++i)
        if (TICK_RATE(i) == hz) return i;
    return -1;
}

static void print_tick_rates() {
    for (uint8_t i = 0; i < N_TICK_RATES; ++i) {
        if (i) Serial.print(',');
        Serial.print(TICK_RATE(i));
    }
}

// Lead time every compare value needs: >= 8 us and >= 2 ticks.
static void set_tick_hz(uint32_t hz) {
    g_tick_hz = hz;
    g_margin = (uint32_t)((8ULL * hz + 999999UL) / 1000000UL);
    if (g_margin < 2) g_margin = 2;
}

static void SEG_ISR_ATTR hw_stop();

// Leave the running/armed state; `done` reports EV DONE to the host.
static void SEG_ISR_ATTR end_playback(bool done) {
    hw_stop();
    g_state = g_valid ? ST_READY : ST_IDLE;
    if (done) g_notes |= NOTE_DONE;
}

// Fire the pending event and compute the next one (shared by both timer back
// ends). Returns false when playback is over, else the delay in `ticks`.
static inline SEG_ISR_ATTR bool fire_event(uint32_t& ticks) {
    uint8_t kind = g_kind;
    if (kind == seg::EV_END) return false;
    uint32_t pulse_t0 = 0;
    if (kind == seg::EV_STEP) {
        STEP_HIGH();
        pulse_t0 = PULSE_STAMP();
        if (g_dir_level) ++g_pos; else --g_pos;
    }
    seg::Event ev;
    g_eng.next(ev);
    if (kind == seg::EV_STEP) {
        STEP_PULSE_PAD(pulse_t0);
        STEP_LOW();
    }
    g_kind = ev.kind;
    if (ev.kind == seg::EV_STEP && ev.dir != g_dir_level) {
        g_dir_level = ev.dir;  // DIR changes after STEP fell, >= one event before the next step
        DIR_WRITE(ev.dir);
    }
    ticks = ev.ticks;
    return !(ev.kind == seg::EV_END && ticks == 0);
}

// ---------------------------------------------------------------------------
// AVR timer back end
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_AVR)
#define CHUNK_MAX 0xC000UL
static uint32_t g_rem = 0;  // ticks still to wait before the pending event fires
static uint8_t g_saved_timsk0 = 0;
static bool g_timer0_off = false;

static void hw_set_tick(uint8_t i) {
    CRIT_BEGIN();
    TIMSK1 = 0;
    TCCR1A = 0;
    TCCR1C = 0;
    TCCR1B = kPrescalerBits[i];  // normal mode, free running
    set_tick_hz(TICK_RATE(i));
    CRIT_END();
}

// Next timer-sized piece of a delay of t ticks. A delay between one and two
// chunks is split in half, so the last piece is never only a few ticks long
// (it would already be in the past when OCR1A is written).
static inline uint16_t next_chunk(uint32_t t) {
    if (t <= CHUNK_MAX) return (uint16_t)t;
    return t >= 2 * CHUNK_MAX ? (uint16_t)CHUNK_MAX : (uint16_t)(t / 2);
}

static inline void avr_schedule(uint32_t t) {
    uint16_t c = next_chunk(t);
    g_rem = t - c;
    uint16_t prev = OCR1A;
    OCR1A = prev + c;
    uint16_t elapsed = TCNT1 - prev;
    if ((uint32_t)elapsed + g_margin >= c) {  // too late: the compare would be missed
        ++g_overruns;
        g_rem = 0;
        OCR1A = TCNT1 + (uint16_t)g_margin;
    }
}

static void hw_stop() {
    TIMSK1 &= (uint8_t)~_BV(OCIE1A);
    PCMSK2 &= (uint8_t)~_BV(PCINT22);
    STEP_LOW();
    if (g_timer0_off) {
        TIMSK0 = g_saved_timsk0;
        g_timer0_off = false;
    }
}

// Start the prepared playback with t0 as time zero. Interrupts must be off.
static void hw_start_at(uint64_t t0) {
    g_saved_timsk0 = TIMSK0;
    TIMSK0 = 0;  // no millis() interrupt during playback: it would add step jitter
    g_timer0_off = true;
    OCR1A = (uint16_t)t0;
    avr_schedule(g_first_ticks);
    TIFR1 = _BV(OCF1A);
    TIMSK1 |= _BV(OCIE1A);
    g_state = ST_RUNNING;
    g_notes |= NOTE_START;
}

static uint64_t hw_now() { return TCNT1; }

ISR(TIMER1_COMPA_vect) {
    if (g_rem) {
        uint16_t c = next_chunk(g_rem);
        g_rem -= c;
        OCR1A += c;
        return;
    }
    uint32_t ticks;
    if (fire_event(ticks)) avr_schedule(ticks);
    else end_playback(true);
}

static uint8_t g_mri_level = 0;  // MRI input level seen by the last PCINT2

ISR(PCINT2_vect) {
    uint16_t t0 = TCNT1;
    uint8_t level = PIND & MRI_BIT;
    // A rising edge happened unless the pin only fell. A pulse that is already
    // over when this (possibly delayed) interrupt runs still counts.
    bool rose = !g_mri_level || level;
    g_mri_level = level;
    if (rose && g_state == ST_ARMED && g_trigger == TRIG_TTL) {
        PCMSK2 &= (uint8_t)~_BV(PCINT22);  // only the first edge matters
        hw_start_at(t0);
    }
}

static void hw_arm_ttl() {
    g_mri_level = PIND & MRI_BIT;
    PCIFR = _BV(PCIF2);
    PCICR |= _BV(PCIE2);
    PCMSK2 |= _BV(PCINT22);
}

static void hw_init() {
    DDRD |= DIR_BIT | STEP_BIT;
    PORTD &= (uint8_t)~(DIR_BIT | STEP_BIT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_MRI, INPUT);
    hw_set_tick(DEFAULT_TICK_INDEX);
}

// ---------------------------------------------------------------------------
// ESP32 timer back end (gptimer, absolute alarm updated in the callback)
// ---------------------------------------------------------------------------
#elif defined(ARDUINO_ARCH_ESP32)
static gptimer_handle_t g_timer = nullptr;
static uint64_t g_alarm = 0;

static void IRAM_ATTR esp_schedule(uint32_t ticks) {
    g_alarm += ticks;
    uint64_t now = 0;
    gptimer_get_raw_count(g_timer, &now);
    if (now + g_margin >= g_alarm) {
        ++g_overruns;
        g_alarm = now + g_margin;
    }
    gptimer_alarm_config_t cfg = {};
    cfg.alarm_count = g_alarm;
    gptimer_set_alarm_action(g_timer, &cfg);
}

static bool IRAM_ATTR on_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*) {
    portENTER_CRITICAL_ISR(&g_mux);
    uint32_t ticks;
    if (fire_event(ticks)) esp_schedule(ticks);
    else end_playback(true);
    portEXIT_CRITICAL_ISR(&g_mux);
    return false;
}

static void hw_set_tick(uint8_t i) {
    if (g_timer) {
        gptimer_handle_t old = g_timer;
        g_timer = nullptr;
        gptimer_set_alarm_action(old, nullptr);
        gptimer_stop(old);
        gptimer_disable(old);
        gptimer_del_timer(old);
    }
    gptimer_config_t cfg = {};
    cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction = GPTIMER_COUNT_UP;
    cfg.resolution_hz = TICK_RATE(i);
    gptimer_new_timer(&cfg, &g_timer);
    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = on_alarm;
    gptimer_register_event_callbacks(g_timer, &cbs, nullptr);
    gptimer_enable(g_timer);
    gptimer_start(g_timer);
    set_tick_hz(TICK_RATE(i));
}

static void IRAM_ATTR hw_start_at(uint64_t t0) {
    g_alarm = t0;
    g_state = ST_RUNNING;
    g_notes |= NOTE_START;
    esp_schedule(g_first_ticks);
}

static uint64_t IRAM_ATTR hw_now() {
    uint64_t now = 0;
    gptimer_get_raw_count(g_timer, &now);
    return now;
}

static void IRAM_ATTR hw_stop() {
    gptimer_set_alarm_action(g_timer, nullptr);
    STEP_LOW();
}

static void IRAM_ATTR ttl_isr() {
    // Only armed for TTL can the timer be in use (LOAD, which re-creates it, is refused then).
    if (g_state != ST_ARMED || g_trigger != TRIG_TTL) return;
    uint64_t t0 = hw_now();
    portENTER_CRITICAL_ISR(&g_mux);
    if (g_state == ST_ARMED && g_trigger == TRIG_TTL) hw_start_at(t0);
    portEXIT_CRITICAL_ISR(&g_mux);
}

// The TTL interrupt stays attached (attaching/detaching is not allowed inside
// a critical section); ttl_isr only acts while armed for a TTL trigger.
static void hw_arm_ttl() {}

static void hw_init() {
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_STEP, OUTPUT);
    digitalWrite(PIN_DIR, LOW);
    digitalWrite(PIN_STEP, LOW);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_MRI, INPUT);
    hw_set_tick(DEFAULT_TICK_INDEX);
    attachInterrupt(digitalPinToInterrupt(PIN_MRI), ttl_isr, RISING);
}
#endif

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
static void start_now() {
    CRIT_BEGIN();
    hw_start_at(hw_now());
    CRIT_END();
}

static void stop_playback() {
    CRIT_BEGIN();
    end_playback(false);
    CRIT_END();
    g_jogging = false;
}

static bool busy() { return g_state == ST_RUNNING || g_state == ST_ARMED; }
// JOG and ZERO must not interrupt an upload either (the jog would end it).
static bool busy_or_loading() { return busy() || g_state == ST_LOADING; }

static void reply_err(const __FlashStringHelper* msg) {
    Serial.print(F("ERR "));
    Serial.println(msg);
}

static void cmd_hello() {
    Serial.print(F("CAPS {\"fw\":\"" FW_NAME "\",\"version\":\"" FW_VERSION "\",\"protocol\":"));
    Serial.print(PROTOCOL_VERSION);
    Serial.print(F(",\"board\":\"" BOARD_NAME "\",\"f_cpu\":"));
    Serial.print((uint32_t)F_CPU);
    Serial.print(F(",\"timer_bits\":"));
    Serial.print(TIMER_BITS);
    Serial.print(F(",\"tick_hz\":["));
    print_tick_rates();
    Serial.print(F("],\"max_segments\":"));
    Serial.print(MAX_SEGMENTS);
    Serial.print(F(",\"seg_bytes\":"));
    Serial.print((unsigned)sizeof(seg::Segment));
    Serial.print(F(",\"frac_bits\":16,\"max_step_rate_hz\":"));
    Serial.print(MAX_STEP_RATE_HZ);
    Serial.print(F(",\"min_event_us\":"));
    Serial.print(MIN_EVENT_US);
    Serial.print(F(",\"step_pulse_us\":"));
    Serial.print(STEP_PULSE_US);
    Serial.print(F(",\"dir_setup_us\":"));
    Serial.print(DIR_SETUP_US);
    Serial.print(F(",\"rx_buffer\":"));
    Serial.print(RX_BUFFER_BYTES);
    Serial.print(F(",\"trigger_latency_us\":"));
    Serial.print(TRIGGER_LATENCY_US);
    Serial.print(F(",\"pins\":{\"dir\":"));
    Serial.print(PIN_DIR);
    Serial.print(F(",\"step\":"));
    Serial.print(PIN_STEP);
    Serial.print(F(",\"button\":"));
    Serial.print(PIN_BUTTON);
    Serial.print(F(",\"mri\":"));
    Serial.print(PIN_MRI);
    Serial.println(F("},\"ttl\":true,\"triggers\":[\"now\",\"ttl\",\"button\"]}"));
}

static void cmd_status() {
    CRIT_BEGIN();
    uint8_t st = g_state;
    uint32_t cycle = g_eng.cycle;
    uint16_t idx = g_eng.idx;
    int32_t pos = g_pos;
    uint16_t ovr = g_overruns;
    CRIT_END();
    static const char* const names[] = {"idle", "loading", "ready", "armed", "running"};
    Serial.print(F("STATUS state="));
    Serial.print(names[st]);
    Serial.print(F(" cycle="));
    Serial.print(cycle);
    Serial.print(F(" seg="));
    Serial.print(st == ST_RUNNING ? (long)idx : -1L);
    Serial.print(F(" pos="));
    Serial.print(pos);
    Serial.print(F(" overruns="));
    Serial.print(ovr);
    Serial.print(F(" trigger="));
    Serial.print(trigger_name(g_trigger));
    Serial.print(F(" cycles="));
    Serial.print(g_cycles);
    Serial.print(F(" nseg="));
    Serial.print(g_valid ? g_nseg : 0);
    Serial.print(F(" tick_hz="));
    Serial.print(g_tick_hz);
    Serial.print(F(" jog="));
    Serial.println(g_jogging ? 1 : 0);
}

static void cmd_load(char* args) {
    if (busy()) return reply_err(F("busy"));
    char* tok[4];
    for (uint8_t i = 0; i < 4; ++i) tok[i] = strtok(i ? nullptr : args, " ");
    if (!tok[3]) return reply_err(F("usage: LOAD <nseg> <tick_hz> <cycles> <now|ttl|button>"));
    unsigned long n = strtoul(tok[0], nullptr, 10);
    unsigned long hz = strtoul(tok[1], nullptr, 10);
    unsigned long cycles = strtoul(tok[2], nullptr, 10);
    int trig = parse_trigger(tok[3]);
    if (n == 0 || n > MAX_SEGMENTS) return reply_err(F("bad segment count"));
    if (trig < 0) return reply_err(F("bad trigger"));
    int tick = tick_index(hz);
    if (tick < 0) return reply_err(F("unsupported tick_hz"));
    g_valid = false;
    hw_set_tick((uint8_t)tick);
    g_nseg = (uint16_t)n;
    g_received = 0;
    g_cycles = cycles;
    g_trigger = (uint8_t)trig;
    g_state = ST_LOADING;
    Serial.println(F("OK LOAD"));
}

static void cmd_seg(char* args) {
    if (g_state != ST_LOADING) return reply_err(F("not loading"));
    if (g_received >= g_nseg) return reply_err(F("too many segments"));
    char* tok[5];
    for (uint8_t i = 0; i < 5; ++i) tok[i] = strtok(i ? nullptr : args, " ");
    if (!tok[4]) return reply_err(F("usage: SEG <count> <interval> <add> <add2> <flags>"));
    seg::Segment& s = g_segs[g_received];
    s.count = (uint16_t)strtoul(tok[0], nullptr, 10);
    s.interval = (uint32_t)strtoul(tok[1], nullptr, 10);
    s.add = (int32_t)strtol(tok[2], nullptr, 10);
    s.add2 = (int32_t)strtol(tok[3], nullptr, 10);
    s.flags = (uint8_t)strtoul(tok[4], nullptr, 10);
    Serial.print(F("OK "));
    Serial.println(g_received);
    ++g_received;
}

static void cmd_end(char* args) {
    if (g_state != ST_LOADING) return reply_err(F("not loading"));
    if (g_received != g_nseg) return reply_err(F("missing segments"));
    unsigned long want = strtoul(args, nullptr, 10);
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < g_nseg; ++i) crc = seg::crc16_segment(crc, g_segs[i]);
    if (crc != (uint16_t)want) {
        g_state = ST_IDLE;
        Serial.print(F("ERR checksum "));
        Serial.println(crc);
        return;
    }
    g_valid = true;
    g_state = ST_READY;
    Serial.print(F("OK END "));
    Serial.print(g_nseg);
    Serial.print(' ');
    Serial.println(crc);
}

static void cmd_arm(char* args) {
    if (busy()) return reply_err(F("busy"));
    if (!g_valid) return reply_err(F("no program"));
    char* t = strtok(args, " ");
    char* c = strtok(nullptr, " ");
    if (t) {
        int trig = parse_trigger(t);
        if (trig < 0) return reply_err(F("bad trigger"));
        g_trigger = (uint8_t)trig;
    }
    if (c) g_cycles = strtoul(c, nullptr, 10);
    g_jogging = false;
    g_overruns = 0;
    prepare_playback(g_segs, g_nseg, g_cycles);
    g_state = ST_ARMED;
    Serial.print(F("OK ARM "));
    Serial.println(trigger_name(g_trigger));
    if (g_trigger == TRIG_NOW) {
        start_now();
    } else if (g_trigger == TRIG_TTL) {
        CRIT_BEGIN();
        hw_arm_ttl();
        CRIT_END();
    }
}

static void cmd_jog(char* args) {
    if (busy_or_loading()) return reply_err(F("busy"));
    char* a = strtok(args, " ");
    char* b = strtok(nullptr, " ");
    if (!b) return reply_err(F("usage: JOG <steps> <rate_hz>"));
    long steps = strtol(a, nullptr, 10);
    unsigned long rate = strtoul(b, nullptr, 10);
    if (steps == 0 || labs(steps) > 65535L) return reply_err(F("bad step count"));
    if (rate == 0 || rate > MAX_STEP_RATE_HZ) return reply_err(F("bad rate"));
    uint32_t ticks = g_tick_hz / rate;
    if (ticks > 65534UL || ticks < g_margin) return reply_err(F("rate out of range for tick_hz"));
    g_jog.count = (uint16_t)labs(steps);
    g_jog.interval = ticks << 16;
    g_jog.add = 0;
    g_jog.add2 = 0;
    g_jog.flags = steps > 0 ? seg::F_DIR : 0;
    g_jogging = true;
    prepare_playback(&g_jog, 1, 1);
    Serial.println(F("OK JOG"));
    start_now();
}

static void handle_line(char* line) {
    char* sp = strchr(line, ' ');
    char* args = sp ? sp + 1 : line + strlen(line);
    if (sp) *sp = '\0';
    if (!strcmp(line, "SEG")) cmd_seg(args);
    else if (!strcmp(line, "HELLO")) cmd_hello();
    else if (!strcmp(line, "STATUS")) cmd_status();
    else if (!strcmp(line, "LOAD")) cmd_load(args);
    else if (!strcmp(line, "END")) cmd_end(args);
    else if (!strcmp(line, "ARM")) cmd_arm(args);
    else if (!strcmp(line, "STOP")) {
        stop_playback();
        Serial.print(F("OK STOP pos="));
        Serial.println(g_pos);
    } else if (!strcmp(line, "FINISH")) {
        if (g_state == ST_ARMED) stop_playback();
        else g_eng.finish_req = 1;
        Serial.println(F("OK FINISH"));
    } else if (!strcmp(line, "ZERO")) {
        if (busy_or_loading()) return reply_err(F("busy"));
        g_pos = 0;
        Serial.println(F("OK ZERO"));
    } else if (!strcmp(line, "JOG")) cmd_jog(args);
    else if (line[0] == '\0') return;
    else {
        Serial.print(F("ERR unknown command "));
        Serial.println(line);
    }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
static char g_line[72];
static uint8_t g_len = 0;
static bool g_overflow = false;
static uint8_t g_btn_count = 0;

void setup() {
    Serial.begin(115200);
    hw_init();
    Serial.println(F("READY " FW_NAME " " FW_VERSION " " BOARD_NAME));
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            g_line[g_len] = '\0';
            if (g_overflow) reply_err(F("line too long"));
            else handle_line(g_line);
            g_len = 0;
            g_overflow = false;
        } else if (g_len < sizeof(g_line) - 1) {
            g_line[g_len++] = c;
        } else {
            g_overflow = true;
        }
    }

    // Button trigger: needs the pin low on several consecutive polls.
    if (g_state == ST_ARMED && g_trigger == TRIG_BUTTON) {
        if (BUTTON_PRESSED()) {
            if (++g_btn_count >= 50) {
                g_btn_count = 0;
                start_now();
            }
        } else {
            g_btn_count = 0;
        }
    }

    uint8_t notes;
    {
        CRIT_BEGIN();
        notes = g_notes;
        g_notes = 0;
        CRIT_END();
    }
    if (notes & NOTE_START) Serial.println(F("EV START"));
    if (notes & NOTE_DONE) {
        g_jogging = false;
        Serial.print(F("EV DONE cycles="));
        Serial.print(g_eng.cycle);
        Serial.print(F(" pos="));
        Serial.print(g_pos);
        Serial.print(F(" overruns="));
        Serial.println(g_overruns);
    }
}
