"""Host-side Device logic that needs no emulator."""
import queue

import pytest

from phantom_device import DeviceError, PhantomDevice


class FakeDevice(PhantomDevice):
    """Answers STATUS/JOG/STOP like the firmware, moving instantly."""

    def __init__(self, pos, stop_after=None):
        self.pos, self.jogs, self.stop_after = pos, [], stop_after
        self.session = None
        self._homing = self._home_stop = False
        self._events = queue.Queue()

    def command(self, line, expect="OK"):
        if line == "STATUS":
            return f"STATUS state=idle pos={self.pos}"
        if line == "STOP":
            return f"OK STOP pos={self.pos}"
        _, steps, _ = line.split()
        self.jogs.append(int(steps))
        self._events.put("EV START")
        if self.stop_after is not None and len(self.jogs) > self.stop_after:
            self.stop()  # pressed while this jog runs: it never reports DONE
        else:
            self.pos += int(steps)
            self._events.put(f"EV DONE cycles=1 pos={self.pos} overruns=0")
        return "OK JOG"


def test_home_splits_long_moves():
    dev = FakeDevice(150000)
    assert dev.home(1000) == 150000
    assert dev.jogs == [-65535, -65535, -18930]
    assert dev.pos == 0


def test_home_ends_on_stop():
    dev = FakeDevice(-150000, stop_after=1)
    with pytest.raises(DeviceError, match="stopped"):
        dev.home(1000)
    assert dev.jogs == [65535, 65535]
    assert not dev._homing
