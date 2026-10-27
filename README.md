# Dynamic MRI Phantom Controller

Stepper-motor controller that moves an MRI phantom with known, repeatable
motion, to validate dynamic MRI reconstruction (golden-angle radial,
compressed sensing, ...).

The motion is synchronized with the MRI sequence: the controller is armed
beforehand and starts on the **first rising edge of the scanner's TTL**, which
marks the start of the sequence execution. That edge is time zero of the ground
truth. The PC computes the exact time of **every step** and the microcontroller
only plays the schedule back from a hardware timer, so the motion is known to
the microsecond relative to the TTL. The push button is a **debug feature** that
stands in for the MRI during development.

**→ [How it works](docs/README.md)** ([website](https://hakkelt.github.io/DynamicMriPhantom/)): diagrams of the system, the scan sequence,
the firmware state machine and the timing, with measured figures.

```
designer (Python)                      firmware (Arduino)
shape + parameters ──► step times ──► integer segments ──► timer ISR ──► STEP/DIR ──► driver
                          │                  │
                          └── simulated bit-exactly ──► "real" trajectory (ground truth)
```

## Hardware

- 200 steps/rev (1.8°) stepper, DRV8825 or A4988 STEP/DIR driver.
- Microstepping is set with jumpers (M0/M1/M2 or MS1/MS2/MS3). The GUI tells
  you the setting for each compiled trajectory and asks for a confirmation.

| Board | DIR | STEP | Button | MRI TTL | Port |
|---|---|---|---|---|---|
| ATmega328PB Xplained Mini (16 MHz) | D2 | D3 | D5 | D6 | /dev/ttyUSB0 |
| ESP32-S3 Dev Module (USB CDC on boot) | 5 | 6 | 8 | 9 | /dev/ttyACM0 |

The button connects to GND (internal pull-up). The MRI TTL input on the ESP32
is 3.3 V: level-shift a 5 V scanner trigger. See
[arduino_firmware/Notes.txt](arduino_firmware/Notes.txt) for board settings.

> **ESP32 status:** the ESP32 code path compiles in CI and its logic and
> integer timing pass the same end-to-end tests as the AVR in a host
> simulation (see [Emulator](#emulator-local-and-ci)), but it has **not** been
> tested on hardware. The AVR path is verified cycle-accurately. Real-hardware
> validation (logic analyser on STEP/DIR, driver current limit) is still to
> be done.

## Layout

| Path | What |
|---|---|
| `arduino_firmware/segment_player/` | Firmware: `segment_player.ino` + `seg_engine.h` (plain C++, also used by the host test) |
| `python_serial/phantom_core.py` | Shapes, step times, segment fitting, firmware simulator, planner, file format (numpy only) |
| `python_serial/phantom_device.py` | Serial protocol (real port or `socket://` URL) |
| `python_serial/phantom_gui.py` | Main window: connect, open/save, jumpers, upload, arm/start/stop, jog |
| `python_serial/phantom_designer.py` | Designer window: parameters, automatic microstep/tick choice, plots |
| `python_serial/phantom_style.py` | Shared look of both windows (ttk theme, report text, plot style) |
| `emulator/` | Board emulators: ATmega328P (avr8js) and ESP32-S3 (host simulation), build scripts, Python wrapper |
| `tests/` | Unit, host cross-check, emulator end-to-end and GUI tests |

## Quick start

```bash
pip install -r python_serial/requirements.txt     # numpy, matplotlib, pyserial, pytest
python python_serial/phantom_gui.py
```

1. **Designer…**: choose the shape and parameters and press **Compile**.
   Check the plots, then **Send to main window** (or **Save…** a `.phtraj`).
2. Select the port and **Connect**. The device capabilities are read and the
   trajectory is checked against them.
3. Set the microstep jumpers as shown and tick **Jumpers are set as shown**.
4. Keep the trigger on **MRI TTL (sequence start)**, set the number of cycles
   (0 = until Finish/STOP), then **Upload** and **Arm / Start**. The motor waits
   for the first TTL edge of the sequence; later edges are ignored.
   *Button* (debug: replaces the MRI) and *Immediately* (debug) are for bench
   work only; the GUI warns that the motion is then not synchronized.
5. **Finish** stops at the end of the current cycle (back at zero). **STOP**
   stops immediately. **Jog**/**Zero** reposition the phantom.
6. **Export ground truth…** writes the position after every step vs the time
   from the TTL edge (CSV, one cycle; the header gives the cycle length).

Flash the firmware with the Arduino IDE (open
`arduino_firmware/segment_player/segment_player.ino`) or with arduino-cli.

## Motion design

Each period is one of:

- **Skewed sinusoid**: half-cosine rise over `[0, a]` and fall over `[a, 1]`,
  with `a = (1 + skew)/2`. Zero velocity at the ends and the peak.
- **Skewed Gaussian**: split normal with the peak at `a`. Each side's width
  follows from the period: its end is 3σ from the peak, and each side is
  normalised so the ends are exactly 0. The end speed is small but not zero.
- **Skewed triangle**: constant speed on each ramp. The corners are not
  rounded; the designer warns about the speed jump at the start, peak and end.

Parameters are amplitude (deg), period length (ms), skewness (-0.95..0.95) and
the delay (dwell at zero) between periods. **Irregularity**: K = 1–5 distinct
periods repeat cyclically, with a % spread on length/amplitude/delay and an
absolute skew jitter drawn from a seed. The generated per-period table is
stored in the file, so it reproduces without the RNG.

Each amplitude is rounded to whole microsteps. A step is placed where the
curve crosses each half-integer microstep level, so every period has zero net
steps and there is no step chatter at the turning point.

**Planner**: tries microsteps from the largest down (only those the driver
supports and you allow) and timer ticks from the finest up. It skips ticks
coarser than the timing tolerance and keeps the first combination that
- stays below the device and driver step rates,
- fits the device's segment memory, and
- meets the timing tolerance (default 10 µs).

It reports peak speed, acceleration and step rate, the segment count and
memory use, and the max timing and position errors. Warnings for the speed,
acceleration and start/stop limits are shown but do not block compiling.

## Segment format and firmware

Every segment is 15 bytes: `count u16, interval u32 (16.16 ticks), add i32 (16.16), add2 i32 (16.16), flags u8`
(bit0 = DIR, bit1 = WAIT). Before each step the engine waits
`(frac + interval) >> 16` ticks and keeps the low 16 bits in `frac`. After
each step, `interval += add` and `add += add2`. Each segment is therefore a
cubic in the step index, which halves the segment count compared with
constant acceleration. A WAIT segment is a pure delay of `interval` whole
ticks. `frac` is reset at the start of every cycle, and the final WAIT makes
the cycle length exact in ticks.

Python takes the absolute time of each step, fits segments greedily (least
squares, then Lawson iterations towards the minimax fit), quantises them, and
verifies them by running the exact integer arithmetic of the firmware. It
checks the timing error, the minimum event spacing and the exact cycle length.

Firmware behaviour:

- One timer interrupt per event.
- Each compare value is scheduled relative to the previous one (`OCR1A += ticks`
  on AVR, an absolute gptimer alarm on ESP32), so latency never accumulates.
- Long delays are split into timer-sized chunks.
- DIR is changed after STEP falls, at least one event before the next step.
- On AVR the millis() interrupt is paused during playback.
- A late compare is counted as an overrun and reported in `STATUS` and `EV DONE`.

Only the **first** TTL rising edge matters: it marks the start of the sequence
execution and is time zero. The button (debug) starts the same way when the
trigger is `button`; `now` starts at the ARM command (debug).

### Serial protocol (115200 baud, ASCII lines)

| Command | Reply |
|---|---|
| `HELLO` | `CAPS {json}`: board, f_cpu, timer bits, tick rates, max segments, receive buffer size, max step rate, min event spacing, pulse/DIR timing, trigger latency, pins, triggers |
| `LOAD <nseg> <tick_hz> <cycles> <now\|ttl\|button>` | `OK LOAD` |
| `SEG <count> <interval> <add> <add2> <flags>` | `OK <index>` |
| `END <crc16>` | `OK END <nseg> <crc16>` or `ERR checksum <crc>` |
| `ARM [trigger] [cycles]` | `OK ARM <trigger>`, then `EV START` and `EV DONE cycles=… pos=… overruns=…` |
| `STOP` / `FINISH` | `OK STOP pos=…` / `OK FINISH` |
| `STATUS` | `STATUS state=… cycle=… seg=… pos=… overruns=… …` |
| `JOG <steps> <rate_hz>`, `ZERO` | `OK JOG` (then `EV DONE`), `OK ZERO` |
| `STREAM <tick_hz> <now\|ttl\|button>` | `OK STREAM <slots>`: the segment memory becomes a ring |
| `SS <count> <interval> <add> <add2> <flags> <crc16>` | `SS <seq>`, or `ERR SS crc\|full\|closed\|…` |
| `CLOSE` | `OK CLOSE`: no more segments; an empty ring then ends the run normally |
| (while streaming) | `EV FREE <consumed>`; `EV ABORT underrun consumed=… pos=… cycles=…` |

The checksum is CRC-16/CCITT-FALSE over the packed little-endian segments
(for `SS`: over that one segment).

### Streaming

A program larger than the device memory is streamed while it plays.

- **Firmware:** the segment table is used as a ring buffer. `SS` lines fill
  it, and the first segment of each cycle carries flag `0x04` (CYCLE), so
  `FINISH` still stops at a cycle boundary.
- **Host:** `phantom_device.StreamSession` fills the ring before `ARM`, then a
  thread sends a segment whenever the device reports a free slot
  (`EV FREE`). It never has more unacknowledged bytes in flight than the
  device's receive buffer.
- **Underrun safeguard:** if the engine needs the next segment and the ring is
  empty before `CLOSE`, the device stops at once and STEP stays low. It
  reports `EV ABORT underrun consumed=… pos=… cycles=…` and goes idle. The GUI
  turns the badge red and raises an alarm with the position.
- **Host-side failures:** a refused or unanswered line stops the device and
  is reported as `EV ABORT host <reason>`.
- **Planner:** streams only when nothing fits in memory at the chosen
  microstep. It models the link (115200 baud, 5 ms round trip, receive
  window) and the deadline of every segment, and requires a worst-case margin
  of `min_stream_slack_ms` (default 300 ms). A PC stall longer than that
  margin aborts the run. It picks the tick with the largest margin.

### Compiled trajectory file (`.phtraj`, JSON)

Contains: format name/version, microstep, driver, tick_hz, steps/rev and
degrees per full step, the designer parameters and limits, the per-period
table, the segments, totals (cycle ticks, steps, checksum), metrics, warnings
and the device capabilities it was compiled for.

## Limits worth knowing

- **ATmega328PB memory**: 72 segments (1080 bytes of the 2 KB RAM). One
  smooth period at 1/32 microstepping needs about 30–70 segments.
  - Irregular sequences with several periods (100–400 segments) are
    **streamed**; typical margins are 0.75–3.7 s.
  - The PC must stay connected for the whole scan. Use the ESP32 (4096
    segments) to keep them on the board.
  - Set `allow_streaming=False` to fall back to a lower microstep instead.
- **While streaming on the ATmega**, each received byte runs the USART
  interrupt (~3.5 µs), so about 1 % of steps come up to ~7 µs late (never
  accumulating).
- **Measured in the emulator (AVR)**:
  - trigger latency: constant 6.9 µs (reported in CAPS)
  - step jitter: < 0.5 µs; up to ~2 µs while the controller is sending a
    serial message (USART interrupt), never accumulating
  - STEP pulse: about 23 µs high
- Lost steps are mechanical as well: set the driver current limit and check
  with a logic analyser on real hardware.

## Emulator (local and CI)

Both board families can be emulated; they share one interface
(`emulator/emulator.py`: serial on a TCP socket, STEP/DIR/TTL edge trace,
`ttl`/`button` inputs), so the end-to-end tests run on both.

### ATmega328P (avr8js, cycle accurate)

The firmware for the ATmega328P runs unmodified in
[avr8js](https://github.com/wokwi/avr8js) (Node.js). Its UART is bridged to a
TCP socket, and every STEP/DIR/TTL edge is recorded with its CPU cycle.

```bash
# toolchain: avr-gcc + avr-libc (Ubuntu: apt install gcc-avr avr-libc), Node >= 18
npm ci --prefix emulator/avr
emulator/build_avr.sh                  # -> build/avr-atmega328p/segment_player.hex
python emulator/emulator.py            # virtual board on socket://127.0.0.1:5555 (real time)
python python_serial/phantom_gui.py --port socket://127.0.0.1:5555
```

Type `ttl` or `button` in the emulator terminal to pulse the MRI input or
press the button. `build_avr.sh` fetches ArduinoCore-avr into `.deps/`. Set
`AVR_GCC_BIN` if avr-gcc is not on `PATH`.

The harness works around an avr8js 0.21.1 bug: a write to UCSR0B drops a
pending RX interrupt, which loses received bytes while the firmware is
transmitting.

### ESP32-S3 (host simulation)

`emulator/build_esp32.sh` compiles the sketch's ESP32 code path with the host
g++ against small stand-ins for the Arduino-ESP32/ESP-IDF APIs it uses
(`emulator/esp32/stubs`) and links it with `emulator/esp32/sim.cpp`: a virtual
240 MHz clock, the gptimer (alarm callback run like an interrupt, 1.5 µs after
the alarm), GPIO register writes, the TTL edge interrupt and critical sections.
Interrupts preempt the main loop between API calls, never inside a critical
section.

```bash
emulator/build_esp32.sh                # -> build/esp32-host/segment_player
python emulator/emulator.py --board esp32s3
```

This checks the logic and the integer timing of the ESP32 back end (alarm
scheduling, overrun handling, TTL start, DIR/STEP order, pulse width). It is
not an Xtensa CPU model: the interrupt latencies are assumptions, so the
trigger latency must still be measured on hardware. The model gives ~3.2 µs,
while CAPS currently states 2 µs.

## Tests

```bash
npm ci --prefix emulator/avr && emulator/build_avr.sh
emulator/build_esp32.sh                # optional: built on demand by the tests
xvfb-run -a python -m pytest -v        # or plain pytest without the GUI tests
```

- `test_core.py`: shapes (zero ends, peak position, skew, derivatives),
  step-time inversion, zero net steps, segment fit error ≤ tolerance, WAIT
  segments, planner choices and warnings, file round-trip, CRC, jumper tables.
- `test_engine_cross.py`: `seg_engine.h` compiled with g++ vs the Python
  simulator. Event times must be bit-identical, including wrap-around edge
  cases.
- `test_emulator.py`: the real firmware on both emulated boards (each test
  runs for `atmega328p` and `esp32s3`):
  - TTL/now/button triggers, and that only the first TTL edge counts
  - streaming through a small ring (exact steps), FINISH, underrun abort with
    the reported position, a host stall, corrupted lines refused
  - every step time against the plan (constant latency, < 1 µs jitter)
  - pulse width and DIR setup
  - exact cycle length, FINISH/STOP/JOG, protocol errors
- `test_gui.py`: designer and main window under Xvfb, including a full
  upload and run against the emulator, with screenshot checks.

GitHub Actions (`.github/workflows/ci.yml`) runs all of this and also
compiles the sketch with arduino-cli for the Uno, the ATmega328PB (MiniCore)
and the ESP32-S3.
