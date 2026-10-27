"""Serial link to the segment_player firmware (arduino_firmware/segment_player).

Works with a real port (``/dev/ttyUSB0``, ``/dev/ttyACM0``, ``COM3``) or any
pyserial URL, e.g. ``socket://127.0.0.1:5555`` for the AVR emulator.

A program that fits in the device is uploaded once (``upload``). A larger one
is streamed (``stream``): the device's segment memory becomes a ring that a
``StreamSession`` thread keeps filled while the device plays. If the ring runs
dry the device aborts at once and reports ``EV ABORT underrun ...``.
"""

from __future__ import annotations

import itertools
import json
import queue
import threading
import time
from typing import Callable, Optional

import serial

import phantom_core as pc

TRIGGERS = ("now", "ttl", "button")
MAX_SEG_LINE = len("SEG 65535 4294967295 -2147483648 -2147483648 255\n")


class DeviceError(Exception):
    pass


class PhantomDevice:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 3.0,
                 boot_wait: float = 2.5,
                 on_event: Optional[Callable[[str], None]] = None,
                 on_line: Optional[Callable[[str, str], None]] = None):
        """``on_event(line)`` gets asynchronous "EV ..." lines, ``on_line(dir, text)``
        every line sent (">") or received ("<"); both run on the reader thread."""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.boot_wait = boot_wait
        self.on_event = on_event
        self.on_line = on_line
        self.caps: Optional[dict] = None
        self.banner: Optional[str] = None
        self._ser = None
        self._replies: "queue.Queue[str]" = queue.Queue()
        self._events: "queue.Queue[str]" = queue.Queue()
        self._lock = threading.Lock()
        self._reader = None
        self._running = False
        self._banner_seen = threading.Event()
        self._uploading = False
        self._wlock = threading.Lock()  # one writer at a time: lines never interleave
        self.session: Optional["StreamSession"] = None

    # -- connection ---------------------------------------------------------
    def open(self) -> dict:
        self._ser = serial.serial_for_url(self.port, baudrate=self.baudrate, timeout=0.05)
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, name="phantom-serial", daemon=True)
        self._reader.start()
        # Opening a real port may reset the board (DTR); wait for its banner.
        self._banner_seen.wait(self.boot_wait)
        last = None
        for _ in range(3):
            try:
                return self.hello()
            except DeviceError as exc:
                last = exc
            except Exception as exc:  # malformed CAPS, serial failure: release the port
                self.close()
                raise DeviceError(f"bad answer to HELLO on {self.port}: {exc}") from exc
        self.close()
        raise DeviceError(f"no answer to HELLO on {self.port}: {last}")

    def close(self) -> None:
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def _read_loop(self) -> None:
        buf = b""
        while self._running:
            try:
                # wait for one byte, then take whatever else has arrived (a plain
                # read(256) would wait for the whole timeout on every reply)
                chunk = self._ser.read(1)
                if chunk:
                    chunk += self._ser.read(self._ser.in_waiting)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                session = self.session
                if session is not None and session.claims(line):
                    session.feed(line)  # stream acks and EV FREE: not logged (too many)
                    if not (line.startswith("SS ") or line.startswith("EV FREE")) and self.on_line:
                        self.on_line("<", line)
                    continue
                # per-segment acks are reported through upload(progress=...)
                if self.on_line and not (self._uploading and line.startswith("OK ")):
                    self.on_line("<", line)
                if line.startswith("EV "):
                    if session is not None:
                        session.feed(line)
                    self._events.put(line)
                    if self.on_event:
                        self.on_event(line)
                elif line.startswith("READY "):
                    self.banner = line
                    self._banner_seen.set()
                else:
                    self._replies.put(line)

    # -- low level ----------------------------------------------------------
    def _send(self, text: str, log: bool = True) -> None:
        if self._ser is None:
            raise DeviceError("not connected")
        if log and self.on_line:
            self.on_line(">", text)
        with self._wlock:
            self._ser.write((text + "\n").encode("ascii"))

    def _emit_event(self, line: str) -> None:
        """Report a host-side event like a device one (EV ABORT host ...)."""
        if self.on_line:
            self.on_line("!", line)
        self._events.put(line)
        if self.on_event:
            self.on_event(line)

    def _reply(self, timeout: Optional[float] = None) -> str:
        try:
            line = self._replies.get(timeout=self.timeout if timeout is None else timeout)
        except queue.Empty:
            raise DeviceError("timeout waiting for the device") from None
        if line.startswith("ERR"):
            raise DeviceError(line[4:] if len(line) > 4 else line)
        return line

    @staticmethod
    def _drain(q: queue.Queue) -> None:
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                return

    def command(self, text: str, expect: str = "OK", timeout: Optional[float] = None) -> str:
        with self._lock:
            self._drain(self._replies)
            self._send(text)
            line = self._reply(timeout)
            if not line.startswith(expect):
                raise DeviceError(f"unexpected reply to {text.split()[0]}: {line}")
            return line

    def wait_event(self, prefix: str = "EV", timeout: float = 10.0) -> str:
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                raise DeviceError(f"timeout waiting for {prefix}")
            try:
                line = self._events.get(timeout=left)
            except queue.Empty:
                continue
            if line.startswith(prefix):
                return line

    def clear_events(self) -> None:
        self._drain(self._events)

    # -- protocol -----------------------------------------------------------
    def hello(self) -> dict:
        line = self.command("HELLO", expect="CAPS")
        self.caps = json.loads(line[5:])
        return self.caps

    def status(self) -> dict:
        line = self.command("STATUS", expect="STATUS")
        out = {}
        for tok in line.split()[1:]:
            k, _, v = tok.partition("=")
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
        return out

    def upload(self, traj: "pc.CompiledTrajectory", cycles: int = 0, trigger: str = "ttl",
               progress: Optional[Callable[[int, int], None]] = None) -> None:
        if trigger not in TRIGGERS:
            raise ValueError(f"trigger must be one of {TRIGGERS}")
        if self.session is not None:  # a resident program replaces a stream
            self.session.detach()
            self.session = None
        segs = traj.segments
        n = len(segs)
        # Keep only as many SEG lines in flight as the receive buffer holds.
        window = max(1, (self.caps or {}).get("rx_buffer", 0) // MAX_SEG_LINE)
        with self._lock:
            self._drain(self._replies)
            self._send(f"LOAD {n} {traj.tick_hz} {int(cycles)} {trigger}")
            self._reply()
            sent = acked = 0
            self._uploading = True
            try:
                while acked < n:
                    while sent < n and sent - acked < window:
                        c, i, a, a2, f = segs[sent]
                        self._send(f"SEG {c} {i} {a} {a2} {f}", log=False)
                        sent += 1
                    line = self._reply()
                    if line != f"OK {acked}":
                        raise DeviceError(f"segment {acked}: unexpected reply {line}")
                    acked += 1
                    if progress:
                        progress(acked, n)
            finally:
                self._uploading = False
            self._send(f"END {traj.checksum}")
            line = self._reply()
            if line != f"OK END {n} {traj.checksum}":
                raise DeviceError(f"END: unexpected reply {line}")

    def needs_streaming(self, traj: "pc.CompiledTrajectory") -> bool:
        return len(traj.segments) > int((self.caps or {}).get("max_segments", 0))

    def stream(self, traj: "pc.CompiledTrajectory", cycles: int = 0, trigger: str = "ttl",
               slots: Optional[int] = None,
               progress: Optional[Callable[[int, int], None]] = None) -> "StreamSession":
        """Open a stream, fill the device's ring and keep it filled on a thread.

        Arm afterwards (``arm()``), like after ``upload``. ``cycles`` = 0 plays
        until ``finish()``/``stop()``. ``slots`` limits the ring (tests). The
        session ends with the device's EV DONE or EV ABORT; host-side failures
        are reported as ``EV ABORT host <reason>`` (the device is stopped).
        """
        if trigger not in TRIGGERS:
            raise ValueError(f"trigger must be one of {TRIGGERS}")
        if not (self.caps or {}).get("stream"):
            raise DeviceError("the device cannot stream")
        if self.session is not None:
            self.session.detach()
        line = self.command(f"STREAM {traj.tick_hz} {trigger}", expect="OK STREAM")
        dev_slots = int(line.split()[2])
        session = StreamSession(self, traj, cycles, min(slots or dev_slots, dev_slots), progress)
        self.session = session
        session.prefill()
        session.start()
        return session

    def arm(self, trigger: Optional[str] = None, cycles: Optional[int] = None) -> str:
        """Arm with the trigger/cycles given at LOAD, or override them (the
        firmware takes cycles only after a trigger)."""
        if cycles is not None and trigger is None:
            raise ValueError("cycles can only be overridden together with the trigger")
        args = ""
        if trigger is not None:
            args = f" {trigger}" + (f" {int(cycles)}" if cycles is not None else "")
        self.clear_events()
        return self.command("ARM" + args)

    def stop(self) -> int:
        if self.session is not None:
            self.session.stopping = True
        line = self.command("STOP")
        if self.session is not None:
            self.session.detach()
        return int(line.split("pos=")[1])

    def finish(self) -> None:
        """Stop at the end of the current cycle (a stream sends up to there and closes)."""
        self.command("FINISH")
        if self.session is not None:
            self.session.finish()

    def zero(self) -> None:
        self.command("ZERO")

    def jog(self, steps: int, rate_hz: int) -> None:
        self.clear_events()
        self.command(f"JOG {int(steps)} {int(rate_hz)}")


class StreamSession:
    """Keeps the device's segment ring filled while a streamed program plays.

    Segment k of the stream is ``traj.segments[k % n]``; the first of every
    cycle carries FLAG_CYCLE. The host sends while the device has a free slot
    (it reports ``EV FREE <consumed>``) and at most ``rx_buffer`` bytes of SS
    lines are unacknowledged. Each SS line carries its CRC. Any refusal or
    silence of the device stops the run (``EV ABORT host <reason>``).
    """

    def __init__(self, dev: PhantomDevice, traj: "pc.CompiledTrajectory", cycles: int, slots: int,
                 progress: Optional[Callable[[int, int], None]] = None):
        self.dev = dev
        self.segments = list(traj.segments)
        self.cycles = int(cycles)
        self.slots = slots
        self.progress = progress
        n = len(self.segments)
        self.total = n * self.cycles if self.cycles > 0 else None  # None: until finished
        self.sent = 0
        self.acked = 0
        self.consumed = 0
        self.closed = False
        self.stopping = False
        self.result: Optional[str] = None  # final EV DONE / EV ABORT line
        self._inflight: list[int] = []     # byte length of unacknowledged SS lines
        self._acks: "queue.Queue[str]" = queue.Queue()
        self._cv = threading.Condition()
        self._thread: Optional[threading.Thread] = None
        # bytes of SS lines in flight; always at least one line
        self._window = int((dev.caps or {}).get("rx_buffer", 64)) - 8
        self._lines = (pc.stream_line(s, k == 0) for k, s in
                       itertools.cycle(enumerate(self.segments)))

    # -- reader thread ------------------------------------------------------
    def claims(self, line: str) -> bool:
        return (line.startswith("SS ") or line.startswith("ERR SS") or line == "OK CLOSE"
                or line.startswith("EV FREE"))

    def feed(self, line: str) -> None:
        with self._cv:
            if line.startswith("EV FREE"):
                self.consumed = max(self.consumed, int(line.split()[2]))
            elif line.startswith("EV DONE") or line.startswith("EV ABORT"):
                self.result = line
            elif not line.startswith("EV "):
                self._acks.put(line)
            self._cv.notify_all()
        if self.progress and line.startswith("EV FREE"):
            self.progress(self.sent, self.consumed)

    # -- sending ------------------------------------------------------------
    @property
    def ended(self) -> bool:
        return self.result is not None or self.stopping

    def finish(self) -> None:
        """Send up to the end of the cycle being sent, then close the stream."""
        with self._cv:
            n = len(self.segments)
            end = -(-self.sent // n) * n
            self.total = end if self.total is None else min(self.total, end)
            self._cv.notify_all()

    def _room(self) -> bool:
        """Another SS line fits in the device's receive buffer."""
        return not self._inflight or sum(self._inflight) + MAX_SEG_LINE + 8 <= self._window

    def _send_next(self) -> None:
        line = next(self._lines)
        self._inflight.append(len(line) + 1)
        self.dev._send(line, log=False)
        self.sent += 1

    def _take_ack(self, timeout: float) -> None:
        try:
            line = self._acks.get(timeout=timeout)
        except queue.Empty:
            raise DeviceError("no answer to streamed segment") from None
        if line.startswith("ERR"):
            raise DeviceError(f"device refused streamed segment {self.acked}: {line[4:]}")
        if line == "OK CLOSE":
            return
        if line != f"SS {self.acked}":
            raise DeviceError(f"streamed segment {self.acked}: unexpected reply {line}")
        self._inflight.pop(0)
        self.acked += 1

    def _pump(self, until: int) -> None:
        """Send segments up to ``until`` (exclusive) and wait for all their acks."""
        while self.acked < until:
            while self.sent < until and self.sent - self.consumed < self.slots and self._room():
                self._send_next()
            if self.acked < self.sent:
                self._take_ack(self.dev.timeout)
            else:
                return

    def prefill(self) -> None:
        """Fill the ring before ARM (synchronous)."""
        first = self.slots if self.total is None else min(self.slots, self.total)
        self._pump(first)
        if self.progress:
            self.progress(self.sent, self.consumed)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="phantom-stream", daemon=True)
        self._thread.start()

    def detach(self) -> None:
        """Stop feeding (after STOP, or when a new program replaces this one)."""
        with self._cv:
            self.stopping = True
            self._cv.notify_all()

    def _close(self) -> None:
        self.dev._send("CLOSE")
        line = self._acks.get(timeout=self.dev.timeout)
        if line != "OK CLOSE":
            raise DeviceError(f"CLOSE: unexpected reply {line}")
        self.closed = True

    def _run(self) -> None:
        try:
            while not self.ended:
                target = self.total
                if target is not None and self.sent >= target and self.acked >= self.sent:
                    self._close()
                    break
                with self._cv:
                    can_send = ((target is None or self.sent < target)
                                and self.sent - self.consumed < self.slots and self._room())
                if can_send:
                    self._send_next()
                    continue
                if self.acked < self.sent:
                    self._take_ack(self.dev.timeout)
                    continue
                with self._cv:  # wait for a free slot (EV FREE), the end, or finish()
                    self._cv.wait(timeout=0.2)
        except (DeviceError, OSError, queue.Empty) as exc:
            if self.ended:
                return
            self.stopping = True
            try:
                self.dev._send("STOP")
            except Exception:
                pass
            self.result = f"EV ABORT host {exc}"
            self.dev._emit_event(self.result)
