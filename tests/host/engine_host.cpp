// Host driver for seg_engine.h: reads a program on stdin, prints every event.
//
// Input:  "<nseg> <cycles>" then one "<count> <interval> <add> <add2> <flags>" per line.
// Output: one "<absolute tick> <kind> <dir>" line per event, until EV_END.
// Also prints "CRC <checksum>" first.
#include <cinttypes>
#include <cstdio>
#include <vector>

#include "../../arduino_firmware/segment_player/seg_engine.h"

int main() {
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
    e.begin(segs.data(), (uint16_t)nseg, (uint32_t)cycles);
    uint64_t t = 0;
    for (;;) {
        seg::Event ev;
        e.next(ev);
        t += ev.ticks;
        std::printf("%" PRIu64 " %u %u\n", t, ev.kind, ev.dir);
        if (ev.kind == seg::EV_END) break;
    }
    return 0;
}
