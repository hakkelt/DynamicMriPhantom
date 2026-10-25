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
