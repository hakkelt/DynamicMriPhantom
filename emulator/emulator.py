"""Python front end for the avr8js emulator (emulator/avr/run_avr.mjs).

    with AvrEmulator() as emu:
        dev = PhantomDevice(emu.url)    # socket://127.0.0.1:<port>
        ...
        emu.ttl()                       # MRI trigger pulse
        trace = emu.trace()             # [(cycle, pin, level), ...]

Command line (run a virtual board for the GUI, in real time):

    python emulator/emulator.py [--build] [--port 5555] [--fast]

then connect the GUI to ``socket://127.0.0.1:5555``. Type ``ttl`` or
``button`` (Enter) to pulse the MRI input or press the button.
"""

from __future__ import annotations

import argparse
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
F_CPU = 16_000_000
DEFAULT_HEX = os.path.join(ROOT, "build", "avr-atmega328p", "segment_player.hex")
RUNNER = os.path.join(HERE, "avr", "run_avr.mjs")


def node_available() -> bool:
    return shutil.which("node") is not None and os.path.isdir(
        os.path.join(HERE, "avr", "node_modules", "avr8js"))


def build_hex(mcu: str = "atmega328p") -> str:
    subprocess.run(["bash", os.path.join(HERE, "build_avr.sh"), mcu], check=True)
    return os.path.join(ROOT, "build", f"avr-{mcu}", "segment_player.hex")


class AvrEmulator:
    def __init__(self, hex_path: str = DEFAULT_HEX, realtime: bool = False, port: int = 0,
                 max_seconds: float = 0, trace: bool = True):
        self.hex_path = hex_path
        self.realtime = realtime
        self.port = port
        self.max_seconds = max_seconds
        self.proc = None
        self.url = None
        self._out: "queue.Queue[str]" = queue.Queue()
        self._tmp = tempfile.mkdtemp(prefix="avr-emu-")
        self.trace_path = os.path.join(self._tmp, "trace.txt") if trace else None
        self._trace = []      # events read so far
        self._trace_pos = 0   # bytes of the trace file already parsed

    def start(self) -> "AvrEmulator":
        if not os.path.exists(self.hex_path):
            raise FileNotFoundError(f"{self.hex_path} not found: run emulator/build_avr.sh first")
        cmd = ["node", RUNNER, "--hex", self.hex_path, "--port", str(self.port)]
        if self.trace_path:
            cmd += ["--trace", self.trace_path]
        if self.realtime:
            cmd.append("--realtime")
        if self.max_seconds:
            cmd += ["--max-seconds", str(self.max_seconds)]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                     bufsize=1)
        threading.Thread(target=self._pump, daemon=True).start()
        line = self._expect("LISTENING", timeout=20)
        self.port = int(line.split()[1])
        self.url = f"socket://127.0.0.1:{self.port}"
        return self

    def _pump(self) -> None:
        for line in self.proc.stdout:
            self._out.put(line.strip())

    def _expect(self, prefix: str, timeout: float = 20) -> str:
        while True:
            try:
                line = self._out.get(timeout=timeout)
            except queue.Empty:
                raise TimeoutError(f"emulator did not print {prefix}") from None
            if line.startswith(prefix):
                return line

    def _cmd(self, text: str, expect: str) -> str:
        self.proc.stdin.write(text + "\n")
        self.proc.stdin.flush()
        return self._expect(expect)

    def ttl(self, us: float = 100) -> int:
        """Raise the MRI input for ``us`` microseconds; returns the start cycle."""
        return int(self._cmd(f"ttl {us}", "TTL").split()[1])

    def button(self, ms: float = 20) -> int:
        return int(self._cmd(f"button {ms}", "BUTTON").split()[1])

    def cycles(self) -> int:
        return int(self._cmd("time", "TIME").split()[1])

    def trace(self):
        """All recorded pin changes so far: list of (cycle, pin name, level)."""
        self._cmd("flush", "FLUSHED")
        with open(self.trace_path) as fh:
            fh.seek(self._trace_pos)
            for ln in fh:
                c, pin, v = ln.split()
                self._trace.append((int(c), pin, int(v)))
            self._trace_pos = fh.tell()
        return list(self._trace)

    def stop(self) -> None:
        if self.proc is None:
            return
        try:
            self.proc.stdin.write("quit\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=10)
        except Exception:
            self.proc.kill()
        self.proc = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hex", default=DEFAULT_HEX)
    ap.add_argument("--build", action="store_true", help="build the firmware first")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fast", action="store_true", help="do not throttle to real time")
    args = ap.parse_args()
    if args.build:
        args.hex = build_hex()
    emu = AvrEmulator(args.hex, realtime=not args.fast, port=args.port, trace=False).start()
    print(f"Emulated ATmega328P running. Connect to {emu.url}")
    print("Commands: ttl | button | quit")
    try:
        for line in sys.stdin:
            cmd = line.strip()
            if cmd == "quit":
                break
            if cmd == "ttl":
                print("TTL at cycle", emu.ttl())
            elif cmd == "button":
                print("button at cycle", emu.button())
            elif cmd:
                print("unknown command")
    except KeyboardInterrupt:
        pass
    emu.stop()


if __name__ == "__main__":
    main()
