import numpy as np
import pytest

import phantom_core as pc
from conftest import board_caps

SKEWS = (-0.6, 0.0, 0.4)


@pytest.mark.parametrize("shape", pc.SHAPES)
@pytest.mark.parametrize("skew", SKEWS)
def test_shape_ends_zero_and_peak(shape, skew):
    f = pc.shape_value(shape, [0.0, 1.0], skew)
    assert np.allclose(f, 0.0, atol=1e-12)
    a = pc.peak_position(skew)
    assert a == pytest.approx((1 + skew) / 2)
    u = np.linspace(0, 1, 20001)
    v = pc.shape_value(shape, u, skew)
    assert v.max() == pytest.approx(1.0, abs=1e-6)
    assert u[np.argmax(v)] == pytest.approx(a, abs=1e-4)
    # monotonic rise and fall
    assert np.all(np.diff(v[u < a]) >= -1e-12)
    assert np.all(np.diff(v[u > a]) <= 1e-12)


def test_sinusoid_zero_velocity_at_ends_and_peak():
    for skew in SKEWS:
        a = pc.peak_position(skew)
        d = pc.shape_derivative("sinusoid", [0.0, a, 1.0], skew)
        assert np.allclose(d, 0.0, atol=1e-9)


@pytest.mark.parametrize("shape", pc.SHAPES)
def test_skew_moves_peak(shape):
    u = np.linspace(0, 1, 10001)
    early = u[np.argmax(pc.shape_value(shape, u, -0.5))]
    late = u[np.argmax(pc.shape_value(shape, u, 0.5))]
    assert early == pytest.approx(0.25, abs=1e-3)
    assert late == pytest.approx(0.75, abs=1e-3)


@pytest.mark.parametrize("shape", pc.SHAPES)
def test_derivatives_match_numeric(shape):
    u = np.linspace(0.01, 0.99, 997)
    u = u[np.abs(u - pc.peak_position(0.3)) > 0.01]
    h = 1e-6
    num = (pc.shape_value(shape, u + h, 0.3) - pc.shape_value(shape, u - h, 0.3)) / (2 * h)
    assert np.allclose(pc.shape_derivative(shape, u, 0.3), num, rtol=1e-4, atol=1e-5)
    num2 = (pc.shape_derivative(shape, u + h, 0.3) - pc.shape_derivative(shape, u - h, 0.3)) / (2 * h)
    assert np.allclose(pc.shape_second_derivative(shape, u, 0.3), num2, rtol=1e-3, atol=1e-3)


@pytest.mark.parametrize("shape", pc.SHAPES)
@pytest.mark.parametrize("rising", (True, False))
def test_inverse(shape, rising):
    f = np.linspace(0.01, 0.99, 50)
    u = pc.shape_inverse(shape, f, 0.2, rising)
    assert np.allclose(pc.shape_value(shape, u, 0.2), f, atol=1e-9)
    a = pc.peak_position(0.2)
    assert np.all(u <= a + 1e-12) if rising else np.all(u >= a - 1e-12)


def test_gaussian_width_from_period():
    # ends sit at GAUSS_N_SIGMA standard deviations on each side
    a = pc.peak_position(0.0)
    sl, sr = pc._gauss_sigmas(a)
    assert sl == pytest.approx(0.5 / pc.GAUSS_N_SIGMA)
    assert sr == pytest.approx(0.5 / pc.GAUSS_N_SIGMA)


@pytest.mark.parametrize("shape", pc.SHAPES)
def test_step_times_cross_half_levels(shape):
    periods = [pc.Period(800, 20, 0.3, 100), pc.Period(600, -12, -0.2, 0)]
    ms = 8
    t, d = pc.step_times(shape, periods, ms)
    assert d.sum() == 0  # zero net steps per sequence
    assert np.all(np.diff(t) > 0)
    pos = np.cumsum(d)
    ideal = pc.degrees_to_steps(pc.ideal_position_deg(shape, periods, t), ms)
    # at the crossing the scaled ideal curve is on a half-integer level between
    # the positions before and after the step (amplitude rounded to whole steps)
    before = pos - d
    scale = [q / p.amplitude_deg for q, p in zip(pc.quantized_amplitudes_deg(periods, ms), periods)]
    starts = pc.period_starts(periods)
    k = np.searchsorted(starts, t, side="right") - 1
    q = ideal * np.array(scale)[k]
    assert np.allclose(q, (before + pos) / 2, atol=1e-6)
    n1 = round(20 / pc.deg_per_step(ms))
    assert np.abs(pos).max() == n1


def test_no_chatter_when_peak_on_half_level():
    ms = 1
    p = [pc.Period(1000, 2.5 * pc.DEG_PER_FULL_STEP, 0.0, 0)]  # 2.5 steps -> rounded
    t, d = pc.step_times("sinusoid", p, ms)
    assert d.sum() == 0
    # direction changes exactly once per period
    assert np.count_nonzero(np.diff(d)) == 1


def test_irregular_periods_reproducible():
    d = pc.Design(n_periods=4, length_spread_pct=20, amplitude_spread_pct=10, delay_ms=100,
                  delay_spread_pct=50, skew_jitter=0.2, seed=7)
    a = pc.generate_periods(d)
    b = pc.generate_periods(d)
    assert a == b
    assert len({p.length_ms for p in a}) == 4
    for p in a:
        assert 800 <= p.length_ms <= 1200
        assert abs(p.skew) <= 0.2 + 1e-12


@pytest.mark.parametrize("shape", pc.SHAPES)
def test_fit_error_within_tolerance(shape):
    design = pc.Design(shape=shape, amplitude_deg=25, length_ms=900, skew=0.25, delay_ms=150)
    limits = pc.Limits(tolerance_us=10)
    traj = pc.compile_design(design, limits, board_caps())
    tau, dirs = traj.step_targets()
    times, kinds, levels = pc.simulate_engine(traj.segments, 3)
    st = times[kinds == pc.EV_STEP]
    expect = np.concatenate([tau + k * traj.total_ticks for k in range(3)])
    err_us = np.abs(st - expect).max() / traj.tick_hz * 1e6
    assert err_us <= max(10.0, 1e6 / traj.tick_hz) + 1e-9
    assert traj.metrics["max_timing_error_us"] <= 10.0 + 1e-9
    # exact sequence length
    assert times[-1] == 3 * traj.total_ticks
    assert len(traj.segments) <= board_caps()["max_segments"]


def test_long_gaps_use_wait_segments():
    # slow motion at a fine tick needs WAIT segments for >60000-tick gaps
    design = pc.Design(shape="sinusoid", amplitude_deg=3.6, length_ms=4000, delay_ms=500)
    caps = board_caps("esp32s3")
    traj = pc.compile_design(design, pc.Limits(allowed_microsteps=[1]), caps)
    assert traj.tick_hz == 40_000_000
    assert any(s[4] & pc.FLAG_WAIT for s in traj.segments[:-1])
    tau, dirs = traj.step_targets()
    tp = pc.timing_params(caps, traj.tick_hz, 10)
    pc.verify_segments(traj.segments, tau, dirs, traj.total_ticks, tp)


def test_planner_prefers_largest_microstep_and_respects_driver():
    design = pc.Design(shape="triangle", amplitude_deg=20, length_ms=1000)
    t = pc.compile_design(design, pc.Limits(driver="DRV8825"), board_caps())
    assert t.microstep == 32
    t = pc.compile_design(design, pc.Limits(driver="A4988"), board_caps())
    assert t.microstep == 16


def test_planner_lowers_microstep_for_step_rate():
    # 180 deg in 100 ms: ~1/32 would need >100 kHz
    design = pc.Design(shape="sinusoid", amplitude_deg=180, length_ms=200)
    res = pc.plan(design, pc.Limits(max_speed_dps=1e9), board_caps())
    assert res.trajectory is not None
    assert res.trajectory.metrics["peak_step_rate_hz"] <= 20_000
    assert res.trajectory.microstep < 32
    assert any("step rate" in a["result"] for a in res.attempts)


def test_planner_warns_on_limits():
    design = pc.Design(shape="triangle", amplitude_deg=90, length_ms=500)
    t = pc.compile_design(design, pc.Limits(max_speed_dps=100, start_stop_speed_dps=10), board_caps("esp32s3"))
    text = " ".join(t.warnings)
    assert "peak speed" in text and "start/stop" in text and "reverses" in text


def test_file_round_trip(tmp_path):
    design = pc.Design(shape="gaussian", amplitude_deg=15, length_ms=700, n_periods=2,
                       length_spread_pct=10, seed=3)
    traj = pc.compile_design(design, pc.Limits(), board_caps())
    path = tmp_path / ("t" + pc.FILE_EXTENSION)
    traj.save(path)
    back = pc.CompiledTrajectory.load(path)
    assert back.segments == traj.segments
    assert back.periods == traj.periods
    assert back.design == traj.design
    assert back.limits == traj.limits
    assert back.microstep == traj.microstep and back.tick_hz == traj.tick_hz
    assert back.total_ticks == traj.total_ticks
    assert back.checksum == traj.checksum
    # the stored period table reproduces without the RNG
    back.design.seed = 999
    assert back.step_targets()[0].tolist() == traj.step_targets()[0].tolist()


def test_ground_truth_export(tmp_path):
    traj = pc.compile_design(pc.Design(shape="sinusoid", amplitude_deg=20, delay_ms=100), pc.Limits(),
                             board_caps())
    t, pos = pc.ground_truth(traj, latency_us=7.0)
    tau, dirs = traj.step_targets()
    assert len(t) == len(tau)
    assert np.all(np.abs(t - (tau / traj.tick_hz + 7e-6)) * 1e6 <= traj.limits.tolerance_us + 1e-6)
    assert pos[-1] == 0 and pos.max() == pytest.approx(20.0, abs=pc.deg_per_step(traj.microstep))
    path = tmp_path / "gt.csv"
    pc.export_ground_truth_csv(traj, path, latency_us=7.0)
    data = np.loadtxt(path, delimiter=",")
    assert np.allclose(data[:, 0], t, atol=1e-9) and np.allclose(data[:, 1], pos)
    assert "TTL rising edge" in path.read_text()


def test_corrupted_file_rejected(tmp_path):
    traj = pc.compile_design(pc.Design(shape="triangle"), pc.Limits(), board_caps())
    d = traj.to_dict()
    d["segments"][0][1] += 1
    with pytest.raises(ValueError):
        pc.CompiledTrajectory.from_dict(d)


def test_crc_known_value():
    assert pc.crc16_ccitt(b"123456789") == 0x29B1


def test_jumper_tables():
    assert pc.DRIVERS["DRV8825"]["table"][16] == (0, 0, 1)
    assert pc.DRIVERS["A4988"]["table"][16] == (1, 1, 1)
    assert "M0=HIGH" in pc.jumper_instructions("DRV8825", 32)
    with pytest.raises(ValueError):
        pc.jumper_instructions("A4988", 32)


def test_compatibility():
    traj = pc.compile_design(pc.Design(shape="triangle"), pc.Limits(), board_caps())
    assert pc.compatibility_problems(traj, board_caps()) == []
    small = board_caps(max_segments=2, tick_hz=[1000])
    probs = pc.compatibility_problems(traj, small)
    assert any("segments" in p for p in probs) and any("tick" in p for p in probs)


def test_curves_shapes():
    traj = pc.compile_design(pc.Design(shape="sinusoid", amplitude_deg=10), pc.Limits(), board_caps())
    c = pc.trajectory_curves(traj, n_cycles=2)
    assert len(c["t"]) == len(c["ideal_deg"]) == len(c["real_deg"])
    assert np.abs(c["error_deg"]).max() <= traj.metrics["max_position_error_deg"] + 1e-6
    assert c["real_deg"][-1] == 0


IRREGULAR = pc.Design(shape="sinusoid", amplitude_deg=30, length_ms=1000, n_periods=5, length_spread_pct=20,
                      amplitude_spread_pct=20, skew_jitter=0.2, delay_ms=50, delay_spread_pct=20)


def test_planner_streams_only_what_does_not_fit():
    caps = board_caps("atmega328pb")
    small = pc.compile_design(pc.Design(amplitude_deg=30), pc.Limits(), caps)
    assert not small.metrics["streamed"] and len(small.segments) <= caps["max_segments"]
    big = pc.compile_design(IRREGULAR, pc.Limits(), caps)
    assert big.microstep == 32 and len(big.segments) > caps["max_segments"]
    m = big.metrics
    assert m["streamed"] and m["stream_slack_ms"] >= 300 and m["stream_link_load"] <= pc.STREAM_MAX_LINK_LOAD
    assert any(w.startswith("streamed") for w in big.warnings)
    assert pc.compatibility_problems(big, caps) == []
    # without streaming the same design needs a lower microstep, or does not fit at all
    res = pc.plan(IRREGULAR, pc.Limits(allow_streaming=False), caps)
    assert res.trajectory is None or res.trajectory.microstep < 32
    assert pc.compatibility_problems(big, {**caps, "stream": False})


MANY = pc.Design(shape="sinusoid", amplitude_deg=10, length_ms=300, n_periods=40, length_spread_pct=10,
                 amplitude_spread_pct=10, skew_jitter=0.1, delay_ms=50, delay_spread_pct=10)


def test_number_of_periods_limited_only_by_the_device():
    with pytest.raises(ValueError):
        pc.generate_periods(pc.Design(n_periods=0))
    # 40 periods: streamed on the ATmega, in memory on the ESP32, every period kept
    for board, streamed in (("atmega328pb", True), ("esp32s3", False)):
        traj = pc.compile_design(MANY, pc.Limits(), board_caps(board))
        assert len(traj.periods) == 40 and traj.metrics["streamed"] == streamed
        assert pc.compatibility_problems(traj, board_caps(board)) == []
        gt_t, _ = pc.ground_truth(traj)
        assert len(gt_t) == len(traj.step_targets()[0])
    # without streaming the ATmega cannot hold them
    assert pc.plan(MANY, pc.Limits(allow_streaming=False), board_caps("atmega328pb")).trajectory is None
    # a cycle longer than 32-bit ticks allow at the tolerance is refused up front
    long = pc.Design(amplitude_deg=10, length_ms=4000, n_periods=5000)
    res = pc.plan(long, pc.Limits(), board_caps("esp32s3"))
    assert res.trajectory is None and res.attempts == [] and "one cycle can last" in res.error
    with pytest.raises(pc.FitError, match="one cycle can last"):
        pc.compile_design(long, pc.Limits(), board_caps("esp32s3"))


def test_planner_fits_each_tick_once(monkeypatch):
    """When the program does not fit in memory, the in-memory trial at a tick is
    not redone for streaming: every tick is fitted once."""
    caps = board_caps("atmega328pb")
    calls = []
    fit = pc.fit_segments
    monkeypatch.setattr(pc, "fit_segments", lambda tau, *a, **k: calls.append(tau[1]) or fit(tau, *a, **k))
    big = pc.compile_design(IRREGULAR, pc.Limits(), caps)
    assert big.metrics["streamed"] and big.microstep == 32
    fine = [t for t in caps["tick_hz"] if 1e6 / t <= pc.Limits().tolerance_us]
    assert len(calls) == len(set(calls)) == len(fine)
    # a program that fits is the same whether streaming is allowed or not
    small = pc.Design(amplitude_deg=30)
    a = pc.compile_design(small, pc.Limits(), caps)
    b = pc.compile_design(small, pc.Limits(allow_streaming=False), caps)
    assert a.segments == b.segments and a.tick_hz == b.tick_hz and not a.metrics["streamed"]


def test_stream_slack_model():
    caps = board_caps("atmega328pb")
    big = pc.compile_design(IRREGULAR, pc.Limits(), caps)
    args = (big.segments, big.tick_hz, big.total_ticks)
    base = pc.stream_slack(*args, caps)
    # a slower link, a smaller buffer or a slower host leave less margin
    assert pc.stream_slack(*args, caps, baudrate=9600)["slack_ms"] < base["slack_ms"]
    assert pc.stream_slack(*args, {**caps, "max_segments": 16})["slack_ms"] < base["slack_ms"]
    assert pc.stream_slack(*args, caps, latency_s=0.05)["slack_ms"] < base["slack_ms"]
    assert pc.stream_slack(*args, caps, baudrate=9600)["link_load"] > 1  # cannot keep up at all
    # a margin requirement the design cannot meet makes the planner refuse to stream
    strict = pc.Limits(min_stream_slack_ms=1e6, allowed_microsteps=[32])
    res = pc.plan(IRREGULAR, strict, caps)
    assert res.trajectory is None and any("streaming margin" in a["result"] for a in res.attempts)


def test_segment_entry_ticks_are_deadlines():
    """The engine takes segment k at the time of the last step before it."""
    traj = pc.compile_design(IRREGULAR, pc.Limits(), board_caps("esp32s3"))
    entry = pc.segment_entry_ticks(traj.segments)
    steps = traj.events[0][traj.events[1] == pc.EV_STEP]
    ends = np.cumsum([0 if s[4] & pc.FLAG_WAIT else s[0] for s in traj.segments])
    for k in range(1, len(traj.segments)):
        last = steps[ends[k - 1] - 1] if ends[k - 1] else 0
        assert entry[k] == pytest.approx(last, abs=2)


def test_stream_line_crc():
    seg = (12, 6553600, -3, 1, pc.FLAG_DIR)
    line = pc.stream_line(seg, cycle_start=True)
    c, i, a, a2, f, crc = map(int, line.split()[1:])
    assert f == pc.FLAG_DIR | pc.FLAG_CYCLE
    assert crc == pc.crc16_ccitt(pc.pack_segment((c, i, a, a2, f)))


# -- ideal trajectory from a CSV ------------------------------------------------

def breathing(n_breaths=3, rate_hz=50, period_s=4.0):
    """An irregular breathing-like motion, sampled, ending where it starts."""
    t = np.arange(0, n_breaths * period_s + 1e-9, 1 / rate_hz)
    x = 8 * np.sin(np.pi * t / period_s) ** 2 * (1 + 0.2 * np.sin(t)) + 3.0  # starts at 3 deg
    x[-1] = x[0]
    return t, x


def write_csv(path, text):
    path.write_text(text)
    return str(path)


def test_read_trajectory_csv_formats(tmp_path):
    want = np.array([[0, 0], [0.5, 2], [1.0, 0]])
    for name, text in [
        ("s.csv", "time_s,position_deg\n0,0\n0.5,2\n1,0\n"),
        ("ms.csv", "# my phantom\ntime_ms;position_deg\n0;0\n500;2\n1000;0\n"),
        ("bare.txt", "0 0\n0.5\t2\n1.0  0\n"),
        ("export.csv", "# phantom ground truth\n# time_s,position_deg\n0.000,0\n0.5,2\n1,0\n"),
        ("time (ms).csv", "Time (ms),Angle\n0,0\n500,2\n1000,0\n"),
    ]:
        assert np.allclose(pc.read_trajectory_csv(write_csv(tmp_path / name, text)), want), name
    for text, msg in [("t,x\n", "no samples"), ("0,0\n0.5,2\nx,y\n", "not a number"), ("0\n", "expected")]:
        with pytest.raises(ValueError, match=msg):
            pc.read_trajectory_csv(write_csv(tmp_path / "bad.csv", text))


@pytest.mark.parametrize("t,x,msg", [
    ([0, 1, 2], [0, 1, 0.5], "must end where it starts"),
    ([0, 1, 1, 2], [0, 1, 1, 0], "time must increase"),
    ([0, 1], [0, 0], "at least 3"),
])
def test_csv_trajectory_rejected(t, x, msg):
    with pytest.raises(ValueError, match=msg):
        pc.SampledPath(t, x)


def test_sampled_path_interpolates_without_overshoot():
    t, x = breathing()
    path = pc.SampledPath(t + 2.0, x)  # times and positions are relative to the first sample
    assert path.duration_s == pytest.approx(t[-1])
    assert np.allclose(path.position(t), x - x[0], atol=1e-12)
    # monotone between samples: never outside the two neighbours
    tt = np.linspace(0, t[-1], 50_001)[:-1]
    k = np.searchsorted(t, tt, side="right") - 1
    y = path.position(tt)
    lo, hi = np.minimum(x[k], x[k + 1]) - x[0], np.maximum(x[k], x[k + 1]) - x[0]
    assert np.all((y >= lo - 1e-9) & (y <= hi + 1e-9))
    # the speed matches the slope of the position, and loops smoothly
    v = path.velocity(tt)
    assert np.allclose(np.gradient(y, tt)[1:-1], v[1:-1], atol=0.05 * np.abs(v).max())
    assert path.velocity(0.0) == pytest.approx(path.velocity(t[-1] - 1e-9), abs=1e-3)
    assert path.position(t[-1] + 0.25) == pytest.approx(path.position(0.25))


@pytest.mark.parametrize("ms", [4, 32])
def test_csv_step_times(ms):
    t, x = breathing()
    path = pc.SampledPath(t, x)
    st, d = path.step_times(ms)
    assert np.all(np.diff(st) > 0) and d.sum() == 0  # ordered, zero net steps per cycle
    # the turning points (whole microsteps) are reached exactly, between them a
    # step is where the curve crosses a half step: the error stays within
    # 1/2 microstep plus the rounding of the turning points
    tt = np.linspace(0, t[-1], 200_001)[:-1]
    real = np.concatenate([[0], np.cumsum(d)])[np.searchsorted(st, tt, side="right")]
    err = np.abs(real - pc.degrees_to_steps(path.position(tt), ms))
    assert err.max() <= 0.5 + pc.degrees_to_steps(path.quantization_deg(ms), ms) + 1e-6
    # one direction change per turning point: no chatter
    turns = np.count_nonzero(np.diff(d))
    assert turns == 2 * 3 - 1


def test_csv_wiggles_below_a_microstep_make_no_steps():
    dps = pc.deg_per_step(16)
    t = np.linspace(0, 1, 101)
    x = 0.2 * dps * np.sin(2 * np.pi * 7 * t)
    x[-1] = 0
    st, _ = pc.SampledPath(t, x).step_times(16)
    assert len(st) == 0


@pytest.mark.parametrize("board", ["atmega328pb", "esp32s3"])
def test_compile_csv_trajectory(tmp_path, board):
    t, x = breathing()
    path = tmp_path / "breath.csv"
    np.savetxt(path, np.column_stack([t * 1000, x]), delimiter=",", header="time_ms,position_deg",
               comments="")
    design = pc.design_from_csv(path)
    assert design.shape == pc.CSV_SHAPE and design.source == "breath.csv"
    traj = pc.compile_design(design, pc.Limits(), board_caps(board))
    m = traj.metrics
    assert m["cycle_s"] == pytest.approx(t[-1])
    assert m["max_position_error_deg"] <= pc.deg_per_step(traj.microstep)
    assert m["max_timing_error_us"] <= 10.0
    assert m["peak_speed_dps"] == pytest.approx(np.abs(np.gradient(x, t)).max(), rel=0.05)
    assert any("turning points rounded" in w for w in traj.warnings)
    # the curves and the ground truth follow the samples
    c = pc.trajectory_curves(traj)
    assert np.abs(c["error_deg"]).max() <= pc.deg_per_step(traj.microstep)
    # saved with its samples (format version 2) and loaded back identically
    d = traj.to_dict()
    assert d["version"] == 2 and len(d["design"]["samples"]) == len(t)
    back = pc.CompiledTrajectory.from_dict(d)
    assert back.checksum == traj.checksum
    assert np.array_equal(back.step_targets()[0], traj.step_targets()[0])


def test_shape_files_stay_version_1():
    traj = pc.compile_design(pc.Design(amplitude_deg=5, length_ms=500), pc.Limits(), board_caps("esp32s3"))
    d = traj.to_dict()
    assert d["version"] == 1 and "samples" not in d["design"]
    assert pc.CompiledTrajectory.from_dict(d).design == traj.design
