"""End-to-end: the real firmware on both emulated boards.

* atmega328p: the firmware hex in avr8js (cycle accurate). Skipped when node +
  avr8js (emulator/avr/node_modules) or the hex are missing; the hex is built
  on demand if avr-g++ is available.
* esp32s3: the firmware's ESP32 code path compiled for the host against a
  simulated gptimer/GPIO (built on demand with g++).

Checks every STEP/DIR edge against the plan computed in Python.
"""
import time

import numpy as np
import pytest

import emulator as emumod
import phantom_core as pc
from phantom_device import DeviceError, PhantomDevice

pytestmark = pytest.mark.emulator

BOARDS = ["atmega328p", "esp32s3"]


def start_emulator(request, realtime=False):
    if request.param == "esp32s3":
        return emumod.Esp32Emulator(request.getfixturevalue("esp32_bin"), realtime=realtime).start()
    return emumod.AvrEmulator(request.getfixturevalue("hex_path"), realtime=realtime).start()


@pytest.fixture(params=BOARDS)
def rig_rt(request):
    """Like ``rig``, but the board runs in real time (streaming: the host must keep pace)."""
    emu = start_emulator(request, realtime=True)
    events = []
    dev = PhantomDevice(emu.url, timeout=10, boot_wait=2, on_event=events.append)
    dev.open()
    yield emu, dev
    dev.close()
    emu.stop()


@pytest.fixture(params=BOARDS)
def rig(request):
    emu = start_emulator(request)
    events = []
    dev = PhantomDevice(emu.url, timeout=10, boot_wait=2, on_event=events.append)
    dev.open()
    yield emu, dev
    dev.close()
    emu.stop()


def edges(trace, pin, level=None):
    return np.array([c for c, p, v in trace if p == pin and (level is None or v == level)], dtype=np.int64)


def compile_for(dev, **kw):
    design = pc.Design(**kw)
    return pc.compile_design(design, pc.Limits(), dev.caps)


def expected_steps(emu, traj, cycles):
    """Firmware step times (emulator CPU cycles from the start) and DIR levels."""
    times, kinds, dirs = pc.simulate_engine(traj.segments, cycles)
    st = times[kinds == pc.EV_STEP]
    return st * (emu.f_cpu / traj.tick_hz), dirs[kinds == pc.EV_STEP]


def check_waveform(emu, trace, traj, cycles, t0_cycle, streamed=False):
    cyc_per_us = emu.f_cpu / 1e6
    rise = edges(trace, "step", 1)
    fall = edges(trace, "step", 0)
    exp_t, exp_d = expected_steps(emu, traj, cycles)
    assert len(rise) == len(exp_t), "step count"
    offset_us = (rise - t0_cycle - exp_t) / cyc_per_us
    # constant trigger latency, sub-microsecond jitter, nothing accumulates
    jitter = offset_us - np.median(offset_us)
    if streamed and emu.board == "atmega328p":
        # streamed segments arrive while playing: each received byte runs the
        # USART interrupt (~3.5 us), which can delay a step; about 1 % of them
        assert jitter.max() < 9.0 and np.mean(jitter > 1.0) < 0.03, (jitter.max(), np.mean(jitter > 1.0))
    else:
        assert offset_us.max() - offset_us.min() < 1.0, offset_us.max() - offset_us.min()
    assert 0 < offset_us.min() and np.median(offset_us) < 10, offset_us
    # pulse width (DRV8825 >= 1.9 us, A4988 >= 1 us)
    width_us = (fall[: len(rise)] - rise) / cyc_per_us
    assert width_us.min() >= 2.0
    # DIR level at every step, and set up before it
    dchg = [(c, v) for c, p, v in trace if p == "dir"]
    dc = np.array([c for c, _ in dchg] or [0])
    dv = np.array([v for _, v in dchg] or [0])
    idx = np.searchsorted(dc, rise, side="right") - 1
    level = np.where(idx >= 0, dv[np.clip(idx, 0, None)], 0)
    assert np.array_equal(level, exp_d)
    setup_us = (rise - dc[np.clip(idx, 0, None)]) / cyc_per_us
    assert setup_us[idx >= 0].min() >= 1.0
    return offset_us


def test_caps_match_python_defaults(rig):
    emu, dev = rig
    caps = dev.caps
    if emu.board == "esp32s3":
        ref = pc.DEFAULT_CAPS["esp32s3"]
    else:
        ref = {**pc.DEFAULT_CAPS["atmega328pb"], "board": "atmega328p"}  # emulator runs a 328P build
    assert caps == ref
    assert caps["seg_bytes"] == pc.SEGMENT_BYTES


def test_ttl_triggered_playback(rig):
    emu, dev = rig
    # dwell at 16 MHz ticks needs WAIT chunking in the ISR (> 65535 ticks)
    traj = compile_for(dev, shape="sinusoid", amplitude_deg=20, length_ms=400, skew=0.3, delay_ms=150)
    dev.upload(traj, cycles=2, trigger="ttl")
    dev.arm()
    time.sleep(0.3)
    assert dev.status()["state"] == "armed"
    assert len(edges(emu.trace(), "step")) == 0  # nothing before the trigger
    t0 = emu.ttl(50)
    done = dev.wait_event("EV DONE", timeout=60)
    assert "overruns=0" in done and "pos=0" in done
    trace = emu.trace()
    mri = edges(trace, "mri", 1)
    assert mri[0] == t0
    off = check_waveform(emu, trace, traj, 2, mri[0])
    if emu.board == "atmega328p":  # cycle accurate: CAPS must state the real latency
        assert off.max() <= dev.caps["trigger_latency_us"] + 1.0
    # the exported ground truth (time since the TTL edge) is what the motor did
    gt_t, _ = pc.ground_truth(traj)
    measured = (edges(trace, "step", 1)[:len(gt_t)] - mri[0]) / emu.f_cpu
    # (the ESP32 interrupt latency is a model, not cycle accurate: allow 2 us)
    assert np.abs(measured - gt_t).max() * 1e6 < (1.0 if emu.board == "atmega328p" else 2.0)
    # the second cycle starts exactly one sequence length later
    n = len(traj.step_targets()[0])
    rise = edges(trace, "step", 1)
    exp2 = expected_steps(emu, traj, 2)[0]
    assert abs((rise[n] - rise[0]) - (exp2[n] - exp2[0])) < 2e-6 * emu.f_cpu
    print(f"trigger latency {off.mean():.2f} us, jitter {off.max() - off.min():.3f} us")


@pytest.mark.parametrize("shape,board_tick", [("triangle", None), ("gaussian", 2_000_000)])
def test_immediate_start_shapes(rig, shape, board_tick):
    emu, dev = rig
    caps = dict(dev.caps)
    if board_tick:
        caps["tick_hz"] = [board_tick]
    traj = pc.compile_design(pc.Design(shape=shape, amplitude_deg=15, length_ms=300, skew=-0.2),
                             pc.Limits(), caps)
    dev.upload(traj, cycles=2, trigger="now")
    t_arm = emu.cycles()
    dev.arm()
    done = dev.wait_event("EV DONE", timeout=60)
    assert "overruns=0" in done and "pos=0" in done
    trace = emu.trace()
    rise = edges(trace, "step", 1)
    exp_t, _ = expected_steps(emu, traj, 2)
    # find the start from the first step and check the rest relative to it
    t0 = rise[0] - exp_t[0]
    assert t0 > t_arm
    rel = (rise - t0 - exp_t) / (emu.f_cpu / 1e6)
    assert np.abs(rel).max() < 1.0


def test_only_first_ttl_counts(rig):
    emu, dev = rig
    traj = compile_for(dev, shape="triangle", amplitude_deg=10, length_ms=300)
    dev.upload(traj, cycles=1, trigger="ttl")
    dev.arm()
    t0 = emu.ttl()
    time.sleep(0.1)
    emu.ttl()  # must not restart the sequence
    dev.wait_event("EV DONE", timeout=60)
    check_waveform(emu, emu.trace(), traj, 1, t0)


def test_button_trigger(rig):
    emu, dev = rig
    traj = compile_for(dev, shape="triangle", amplitude_deg=10, length_ms=300)
    dev.upload(traj, cycles=1, trigger="button")
    dev.arm()
    time.sleep(0.2)
    assert dev.status()["state"] == "armed"
    emu.button(30)
    done = dev.wait_event("EV DONE", timeout=60)
    assert "pos=0" in done
    assert len(edges(emu.trace(), "step", 1)) == len(traj.step_targets()[0])


def test_finish_stops_at_cycle_end_and_stop_is_immediate(rig):
    emu, dev = rig
    traj = compile_for(dev, shape="sinusoid", amplitude_deg=10, length_ms=300)
    dev.upload(traj, cycles=0, trigger="now")
    dev.arm()
    time.sleep(0.5)
    st = dev.status()
    assert st["state"] == "running"
    dev.finish()
    done = dev.wait_event("EV DONE", timeout=60)
    assert "pos=0" in done
    n = len(traj.step_targets()[0])
    assert len(edges(emu.trace(), "step", 1)) % n == 0
    # STOP mid-motion keeps the position (not zero in general) and returns to ready
    dev.arm("now", 0)
    time.sleep(0.25)
    dev.stop()
    st = dev.status()
    assert st["state"] == "ready"
    dev.jog(-st["pos"], 2000) if st["pos"] else None
    if st["pos"]:
        dev.wait_event("EV DONE", timeout=60)
    assert dev.status()["pos"] == 0


def test_long_wait_last_chunk_not_missed(rig):
    """A delay of 2*0xC000+5 ticks used to end with a 5-tick timer chunk that
    was already in the past, so the step came one 16-bit timer wrap late (AVR);
    on the ESP32 it is just a long delay of the 54-bit timer."""
    emu, dev = rig
    tick = max(dev.caps["tick_hz"])
    # the WAIT merges into the step's delay: total 2*0xC000+5 ticks
    wait = 2 * 0xC000 + 5 - 1000
    segs = [(0, wait, 0, 0, pc.FLAG_WAIT), (1, 1000 << 16, 0, 0, pc.FLAG_DIR),
            (0, 2000, 0, 0, pc.FLAG_WAIT)]
    dev.command(f"LOAD {len(segs)} {tick} 1 ttl")
    for k, seg in enumerate(segs):
        dev.command("SEG " + " ".join(map(str, seg)), expect=f"OK {k}")
    dev.command(f"END {pc.segments_checksum(segs)}")
    dev.arm()
    t0 = emu.ttl()
    assert "overruns=0" in dev.wait_event("EV DONE", timeout=30)
    rise = edges(emu.trace(), "step", 1)
    late_us = (rise[0] - t0 - (wait + 1000) * emu.f_cpu / tick) / (emu.f_cpu / 1e6)
    assert 0 < late_us < 10, late_us


def test_protocol_errors(rig):
    _, dev = rig
    with pytest.raises(DeviceError):
        dev.command("LOAD 3 12345 1 now")  # unsupported tick
    with pytest.raises(DeviceError):
        dev.command(f"LOAD {dev.caps['max_segments'] + 1} 2000000 1 now")  # too many segments
    with pytest.raises(DeviceError):
        dev.command("ARM")  # no program
    dev.command("LOAD 1 2000000 1 now")
    with pytest.raises(DeviceError, match="busy"):
        dev.command("JOG 10 1000")  # must not abort the upload
    with pytest.raises(ValueError):
        dev.arm(cycles=3)  # cycles need an explicit trigger
    dev.command("SEG 10 6553600 0 0 1", expect="OK 0")
    with pytest.raises(DeviceError, match="checksum"):
        dev.command("END 1")
    with pytest.raises(DeviceError, match="unknown"):
        dev.command("FOO")
    assert dev.status()["state"] == "idle"


def test_jog_and_zero(rig):
    emu, dev = rig
    dev.jog(40, 4000)
    assert "pos=40" in dev.wait_event("EV DONE", timeout=30)
    rise = edges(emu.trace(), "step", 1)
    assert len(rise) == 40
    # the first steps overlap "OK JOG"/"EV START" being sent: the USART interrupt
    # can delay a step by ~1.5 us (it never accumulates)
    assert np.all(np.abs(np.diff(rise) - emu.f_cpu / 4000) <= 2.5e-6 * emu.f_cpu)
    dev.zero()
    assert dev.status()["pos"] == 0


# -- streaming -----------------------------------------------------------------

def stream_design():
    # 5 irregular periods: 90-100 segments, more than the ATmega's 72
    return pc.Design(shape="sinusoid", amplitude_deg=10, length_ms=300, n_periods=5, length_spread_pct=10,
                     amplitude_spread_pct=10, skew_jitter=0.1, delay_ms=50, delay_spread_pct=10)


def test_stream_plays_exactly(rig_rt):
    emu, dev = rig_rt
    traj = pc.compile_design(stream_design(), pc.Limits(), {**dev.caps, "max_segments": 72})
    assert len(traj.segments) > 72 and traj.metrics["streamed"]
    # a 16-slot ring: the host refills it many times per cycle
    session = dev.stream(traj, cycles=2, trigger="ttl", slots=16)
    assert session.sent == 16
    dev.arm()
    t0 = emu.ttl()
    done = dev.wait_event("EV ", timeout=60)
    assert done.startswith("EV START")
    done = dev.wait_event("EV DONE", timeout=60)
    assert "overruns=0" in done and "pos=0" in done
    assert "cycles=2" in done
    assert session.sent == 2 * len(traj.segments) and session.closed
    check_waveform(emu, emu.trace(), traj, 2, t0, streamed=True)
    assert dev.status()["state"] == "idle"  # a streamed program is used up


def test_stream_finish_stops_at_cycle_end(rig_rt):
    emu, dev = rig_rt
    traj = pc.compile_design(stream_design(), pc.Limits(), {**dev.caps, "max_segments": 72})
    dev.stream(traj, cycles=0, trigger="now", slots=16)
    dev.arm()
    time.sleep(traj.cycle_s * 0.5)
    dev.finish()
    done = dev.wait_event("EV DONE", timeout=60)
    assert "pos=0" in done and "overruns=0" in done
    n = len(traj.step_targets()[0])
    rise = edges(emu.trace(), "step", 1)
    assert len(rise) > 0 and len(rise) % n == 0


def test_stream_underrun_aborts_and_reports(rig):
    """The host stops feeding: the device aborts at the first missing segment,
    STEP stays low and the host is told where the motor is."""
    emu, dev = rig
    traj = pc.compile_design(stream_design(), pc.Limits(), {**dev.caps, "max_segments": 72})
    k = 20
    dev.command(f"STREAM {traj.tick_hz} now", expect="OK STREAM")
    for i, seg in enumerate(traj.segments[:k]):
        dev.command(pc.stream_line(seg, i == 0), expect=f"SS {i}")
    dev.arm()
    abort = dev.wait_event("EV ABORT", timeout=60)
    assert abort.startswith("EV ABORT underrun")
    fields = dict(tok.split("=") for tok in abort.split()[3:])
    times, kinds, dirs = pc.simulate_engine(traj.segments[:k], 1)
    steps = kinds == pc.EV_STEP
    expected_pos = int(np.sum(np.where(dirs[steps] == 1, 1, -1)))
    assert int(fields["consumed"]) == k and int(fields["pos"]) == expected_pos != 0
    trace = emu.trace()
    assert len(edges(trace, "step", 1)) == int(steps.sum())
    assert [v for _, p, v in trace if p == "step"][-1] == 0  # STEP left low
    st = dev.status()
    assert st["state"] == "idle" and st["pos"] == expected_pos
    with pytest.raises(DeviceError):
        dev.arm()  # nothing left to play


def test_stream_refuses_corrupted_segment(rig):
    _, dev = rig
    dev.command("STREAM 2000000 now", expect="OK STREAM")
    line = pc.stream_line((10, 6553600, 0, 0, 1), True)
    bad = line.rsplit(" ", 1)[0] + " " + str((int(line.rsplit(" ", 1)[1]) + 1) & 0xFFFF)
    with pytest.raises(DeviceError, match="SS crc"):
        dev.command(bad, expect="SS")
    dev.command(line, expect="SS 0")
    dev.command("CLOSE", expect="OK CLOSE")
    with pytest.raises(DeviceError, match="SS closed"):
        dev.command(line, expect="SS")


def test_stream_host_stall_is_reported_to_python(rig):
    """The PC stops feeding mid-run (stall, crash): the session learns of the
    device's abort and the position it stopped at."""
    _, dev = rig
    traj = pc.compile_design(stream_design(), pc.Limits(), {**dev.caps, "max_segments": 72})
    session = dev.stream(traj, cycles=0, trigger="now", slots=8)
    session.detach()  # the host stalls: nothing more is sent
    dev.arm()
    abort = dev.wait_event("EV ABORT", timeout=60)
    assert abort.startswith("EV ABORT underrun consumed=8 ")
    deadline = time.monotonic() + 5
    while session.result is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert session.result == abort
    assert dev.status()["pos"] == int(abort.split("pos=")[1].split()[0])
