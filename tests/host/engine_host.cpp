// Host driver for seg_engine.h: reads a program on stdin, prints every event.
//
// Input:  "<nseg> <cycles>" then one "<count> <interval> <add> <add2> <flags>" per line.
// Output: one "<absolute tick> <kind> <dir>" line per event, until EV_END.
// Also prints "CRC <checksum>" first.
//
// "engine_host stream <ring_slots> <push_limit>": play the program `cycles`
// times through a seg::Ring of ring_slots, refilled after every event like the
// firmware's host link (F_CYCLE on the first segment of each cycle). With
// push_limit > 0 only that many segments are pushed and the stream is never
// closed, so it ends with EV_UNDERRUN.
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "../../arduino_firmware/segment_player/seg_engine.h"

int main(int argc, char** argv) {
    const bool stream = argc > 1 && !std::strcmp(argv[1], "stream");
    const unsigned slots = stream && argc > 2 ? (unsigned)std::atoi(argv[2]) : 0;
    const unsigned long push_limit = stream && argc > 3 ? std::strtoul(argv[3], nullptr, 10) : 0;
    unsigned nseg = 0;
    unsigned long cycles = 0;
    if (std::scanf("%u %lu", &nseg, &cycles) != 2 || cycles == 0) return 2;
    std::vector<seg::Segment> segs(nseg);
    uint16_t crc = 0xFFFF;
    for (unsigned i = 0; i < nseg; ++i) {
        unsigned count, flags;
        unsigned long interval;
        long add, add2;
        if (std::scanf("%u %lu %ld %ld %u", &count, &interval, &add, &add2, &flags) != 5) return 3;
        segs[i].count = (uint16_t)count;
        segs[i].interval = (uint32_t)interval;
        segs[i].add = (int32_t)add;
        segs[i].add2 = (int32_t)add2;
        segs[i].flags = (uint8_t)flags;
        crc = seg::crc16_segment(crc, segs[i]);
    }
    std::printf("CRC %u\n", crc);
    seg::Engine e;
    std::vector<seg::Segment> ring_buf(slots ? slots : 1);
    seg::Ring ring;
    unsigned long pushed = 0;
    const unsigned long total = push_limit ? push_limit : (unsigned long)nseg * cycles;
    auto refill = [&]() {
        while (pushed < total) {
            seg::Segment s = segs[pushed % nseg];
            if (pushed % nseg == 0) s.flags |= seg::F_CYCLE;
            if (!ring.push_write(s)) break;
            ring.push_commit();
            ++pushed;
        }
        if (pushed == total && !push_limit) ring.closed = 1;
    };
    if (stream) {
        ring.reset(ring_buf.data(), (uint16_t)slots);
        refill();
        e.begin_stream(&ring);
    } else {
        e.begin(segs.data(), (uint16_t)nseg, (uint32_t)cycles);
    }
    uint64_t t = 0;
    for (;;) {
        seg::Event ev;
        e.next(ev);
        if (stream) refill();
        t += ev.ticks;
        std::printf("%" PRIu64 " %u %u\n", t, ev.kind, ev.dir);
        if (ev.kind == seg::EV_END || ev.kind == seg::EV_UNDERRUN) break;
    }
    return 0;
}
