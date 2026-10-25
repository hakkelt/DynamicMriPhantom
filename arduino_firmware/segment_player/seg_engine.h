// Integer step-segment engine shared by the firmware and the host test.
//
// No Arduino includes, no floats. The Python side (python_serial/phantom_core.py,
// simulate_engine) mirrors this code bit for bit; tests/host/engine_host.cpp
// checks that.
//
// A program is a table of segments that is played once per cycle:
//   STEP segment: `count` steps. Before each step the engine waits
//                 (frac + interval) >> 16 ticks and keeps the low 16 bits in
//                 `frac`; after each step interval += add and add += add2.
//   WAIT segment: a pure delay of `interval` whole ticks (frac unchanged).
// Delays of consecutive WAITs are merged into the delay of the next step.
// `frac` is reset at the start of every cycle so every cycle is identical.
#pragma once

#include <stdint.h>

#ifndef SEG_ISR_ATTR
#define SEG_ISR_ATTR
#endif

namespace seg {

enum : uint8_t { F_DIR = 0x01, F_WAIT = 0x02 };
enum : uint8_t { EV_DELAY = 0, EV_STEP = 1, EV_END = 2 };

struct __attribute__((packed)) Segment {
    uint16_t count;
    uint32_t interval;  // 16.16 ticks (STEP) or whole ticks (WAIT)
    int32_t add;        // 16.16 ticks
    int32_t add2;       // 16.16 ticks
    uint8_t flags;
};
static_assert(sizeof(Segment) == 15, "Segment must be packed to 15 bytes");

struct Event {
    uint32_t ticks;  // delay from the previous event
    uint8_t kind;    // EV_STEP, EV_DELAY or EV_END
    uint8_t dir;     // DIR level for a step (1 = positive)
};

struct Engine {
    const Segment* segs;
    uint16_t nseg;
    uint32_t cycles;  // 0 = forever
    uint32_t cycle;   // completed cycles
    uint16_t idx;
    uint16_t left;
    uint32_t interval;
    uint32_t add;
    uint32_t add2;
    uint16_t frac;
    uint8_t flags;
    volatile uint8_t finish_req;  // stop at the end of the current cycle

    void begin(const Segment* s, uint16_t n, uint32_t ncycles) {
        segs = s;
        nseg = n;
        cycles = ncycles;
        cycle = 0;
        idx = 0xFFFF;
        left = 0;
        interval = add = add2 = 0;
        frac = 0;
        flags = 0;
        finish_req = 0;
    }

    SEG_ISR_ATTR bool enter() {
        ++idx;
        if (idx >= nseg) {
            ++cycle;
            if (finish_req || (cycles && cycle >= cycles)) {
                idx = nseg;
                return false;
            }
            idx = 0;
            frac = 0;
        }
        const Segment& s = segs[idx];
        flags = s.flags;
        left = (flags & F_WAIT) ? 1 : s.count;
        interval = s.interval;
        add = (uint32_t)s.add;
        add2 = (uint32_t)s.add2;
        return true;
    }

    SEG_ISR_ATTR void next(Event& ev) {
        uint32_t delay = 0;
        uint16_t guard = 0;
        uint8_t kind = EV_STEP;
        for (;;) {
            if (left == 0) {
                if (!enter()) {
                    kind = EV_END;
                    break;
                }
                if (++guard > nseg + 1) {  // a cycle without any step
                    kind = EV_DELAY;
                    break;
                }
                if (left == 0) continue;
            }
            if (flags & F_WAIT) {
                delay += interval;
                left = 0;
                continue;
            }
            uint32_t acc = (uint32_t)frac + interval;
            delay += acc >> 16;
            frac = (uint16_t)acc;
            interval += add;
            add += add2;
            --left;
            break;
        }
        ev.ticks = delay;
        ev.kind = kind;
        ev.dir = flags & F_DIR;
    }
};

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over the packed little-endian
// segment bytes; the same as phantom_core.segments_checksum.
inline uint16_t crc16_update(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)b << 8;
    for (uint8_t i = 0; i < 8; ++i)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
}

inline uint16_t crc16_segment(uint16_t crc, const Segment& s) {
    const uint32_t words[3] = {s.interval, (uint32_t)s.add, (uint32_t)s.add2};
    crc = crc16_update(crc, (uint8_t)s.count);
    crc = crc16_update(crc, (uint8_t)(s.count >> 8));
    for (uint8_t w = 0; w < 3; ++w)
        for (uint8_t i = 0; i < 4; ++i) crc = crc16_update(crc, (uint8_t)(words[w] >> (8 * i)));
    return crc16_update(crc, s.flags);
}

}  // namespace seg
