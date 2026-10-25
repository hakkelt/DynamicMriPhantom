"""Serial link to the segment_player firmware (arduino_firmware/segment_player).

Works with a real port (``/dev/ttyUSB0``, ``/dev/ttyACM0``, ``COM3``) or any
pyserial URL, e.g. ``socket://127.0.0.1:5555`` for the AVR emulator.
"""

from __future__ import annotations

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
                chunk = self._ser.read(256)
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
                # per-segment acks are reported through upload(progress=...)
                if self.on_line and not (self._uploading and line.startswith("OK ")):
                    self.on_line("<", line)
                if line.startswith("EV "):
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
        self._ser.write((text + "\n").encode("ascii"))

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
        line = self.command("STOP")
        return int(line.split("pos=")[1])

    def finish(self) -> None:
        self.command("FINISH")

    def zero(self) -> None:
        self.command("ZERO")

    def jog(self, steps: int, rate_hz: int) -> None:
        self.clear_events()
        self.command(f"JOG {int(steps)} {int(rate_hz)}")
