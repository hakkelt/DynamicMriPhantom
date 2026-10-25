"""GUI-free core of the dynamic MRI phantom trajectory designer.

Pipeline:
    design parameters -> per-period table -> ideal position x(t)
    -> step times (crossings of half-integer microstep levels)
    -> integer timer ticks -> constant-acceleration segments
    -> bit-exact simulation of the firmware engine -> "real" trajectory.

Segment format (shared with arduino_firmware/segment_player/seg_engine.h):
    count    u16   number of steps in the segment
    interval u32   16.16 fixed-point ticks of the first step
    add      i32   16.16 fixed-point change of the interval after each step
    add2     i32   16.16 fixed-point change of ``add`` after each step
    flags    u8    bit0 DIR (1 = positive), bit1 WAIT
so the n-th step (n = 1..count) of a segment comes
    n*interval + add*n(n-1)/2 + add2*n(n-1)(n-2)/6
after the segment start (a cubic in n).
A WAIT segment is a pure delay: ``interval`` holds an integer number of ticks.

Only numpy is required.
"""

from __future__ import annotations

import binascii
import dataclasses
import functools
import json
import math
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

FORMAT_NAME = "phantom-trajectory"
PROTOCOL_VERSION = 1  # serial protocol of arduino_firmware/segment_player
FORMAT_VERSION = 1
FILE_EXTENSION = ".phtraj"

FULL_STEPS_PER_REV = 200
DEG_PER_FULL_STEP = 360.0 / FULL_STEPS_PER_REV  # 1.8 deg

FRAC_BITS = 16
FRAC_ONE = 1 << FRAC_BITS
FLAG_DIR = 0x01
FLAG_WAIT = 0x02
SEGMENT_BYTES = 15  # packed size on the device
MAX_INTERVAL_FP = 0xFFFF0000  # keeps frac + interval inside 32 bits
MAX_COUNT = 0xFFFF
U32 = 0xFFFFFFFF

SHAPES = ("sinusoid", "gaussian", "triangle")
GAUSS_N_SIGMA = 3.0  # the Gaussian width is derived from the period: ends at 3 sigma
MAX_ABS_SKEW = 0.95

EV_DELAY = 0
EV_STEP = 1
EV_END = 2

# ----------------------------------------------------------------------------
# Drivers and microstep jumpers
# ----------------------------------------------------------------------------

# Levels of the three mode pins, in the order of DRIVERS[...]["pins"].
DRIVERS = {
    "DRV8825": {
        "pins": ("M0", "M1", "M2"),
        "table": {1: (0, 0, 0), 2: (1, 0, 0), 4: (0, 1, 0), 8: (1, 1, 0),
                  16: (0, 0, 1), 32: (1, 0, 1)},
        "min_pulse_us": 1.9,
        "dir_setup_us": 0.65,
        "max_step_rate_hz": 250_000,
    },
    "A4988": {
        "pins": ("MS1", "MS2", "MS3"),
        "table": {1: (0, 0, 0), 2: (1, 0, 0), 4: (0, 1, 0), 8: (1, 1, 0),
                  16: (1, 1, 1)},
        "min_pulse_us": 1.0,
        "dir_setup_us": 0.2,
        "max_step_rate_hz": 500_000,
    },
}


def driver_microsteps(driver: str) -> list[int]:
    return sorted(DRIVERS[driver]["table"])


def jumper_instructions(driver: str, microstep: int) -> str:
    """Human-readable mode-pin setting for ``microstep`` on ``driver``."""
    info = DRIVERS[driver]
    if microstep not in info["table"]:
        raise ValueError(f"{driver} does not support 1/{microstep} microstepping "
                         f"(supported: {', '.join('1/%d' % m for m in sorted(info['table']))})")
    levels = info["table"][microstep]
    pins = ", ".join(f"{p}={'HIGH' if v else 'LOW'}" for p, v in zip(info["pins"], levels))
    return (f"Set {'/'.join(info['pins'])} on the {driver} to: {pins}  "
            f"for 1/{microstep} microstepping "
            f"(HIGH = jumper fitted on a CNC-shield style board, LOW = open).")


# ----------------------------------------------------------------------------
# Device capabilities
# ----------------------------------------------------------------------------

# Defaults matching arduino_firmware/segment_player; the real values come from
# the CAPS reply of the connected device.
DEFAULT_CAPS = {
    "atmega328pb": {
        "fw": "segment_player", "version": "1.0.0", "protocol": PROTOCOL_VERSION,
        "board": "atmega328pb", "f_cpu": 16_000_000, "timer_bits": 16,
        "tick_hz": [16_000_000, 2_000_000, 250_000, 62_500, 15_625],
        "max_segments": 72, "seg_bytes": SEGMENT_BYTES, "frac_bits": FRAC_BITS, "rx_buffer": 64,
        "max_step_rate_hz": 20_000, "min_event_us": 40.0,
        "step_pulse_us": 3.0, "dir_setup_us": 1.0, "trigger_latency_us": 7.0,
        "pins": {"dir": 2, "step": 3, "button": 5, "mri": 6},
        "ttl": True, "triggers": ["now", "ttl", "button"],
    },
    "esp32s3": {
        "fw": "segment_player", "version": "1.0.0", "protocol": PROTOCOL_VERSION,
        "board": "esp32s3", "f_cpu": 240_000_000, "timer_bits": 54,
        "tick_hz": [40_000_000, 10_000_000, 4_000_000, 2_000_000, 1_000_000],
        "max_segments": 4096, "seg_bytes": SEGMENT_BYTES, "frac_bits": FRAC_BITS, "rx_buffer": 256,
        "max_step_rate_hz": 50_000, "min_event_us": 12.0,
        "step_pulse_us": 3.0, "dir_setup_us": 1.0, "trigger_latency_us": 2.0,
        "pins": {"dir": 5, "step": 6, "button": 8, "mri": 9},
        "ttl": True, "triggers": ["now", "ttl", "button"],
    },
}


# ----------------------------------------------------------------------------
# Design parameters
# ----------------------------------------------------------------------------

@dataclass
class Design:
    shape: str = "sinusoid"
    amplitude_deg: float = 30.0
    length_ms: float = 1000.0
    skew: float = 0.0
    delay_ms: float = 0.0
    # Irregularity: K distinct periods repeated cyclically.
    n_periods: int = 1
    length_spread_pct: float = 0.0
    amplitude_spread_pct: float = 0.0
    delay_spread_pct: float = 0.0
    skew_jitter: float = 0.0
    seed: int = 1


@dataclass
class Limits:
    driver: str = "DRV8825"
    allowed_microsteps: Optional[list[int]] = None  # None = all the driver supports
    max_speed_dps: float = 720.0
    max_accel_dps2: float = 50_000.0
    start_stop_speed_dps: float = 90.0
    tolerance_us: float = 10.0


@dataclass
class Period:
    length_ms: float
    amplitude_deg: float
    skew: float
    delay_ms: float


def generate_periods(design: Design) -> list[Period]:
    """The per-period table; random irregularity is drawn from ``design.seed``."""
    if design.shape not in SHAPES:
        raise ValueError(f"unknown shape {design.shape!r}")
    k = int(design.n_periods)
    if not 1 <= k <= 5:
        raise ValueError("n_periods must be between 1 and 5")
    rng = np.random.default_rng(design.seed)
    periods = []
    for _ in range(k):
        u = rng.uniform(-1.0, 1.0, size=4)
        length = design.length_ms * (1 + design.length_spread_pct / 100 * u[0])
        amp = design.amplitude_deg * (1 + design.amplitude_spread_pct / 100 * u[1])
        delay = max(0.0, design.delay_ms * (1 + design.delay_spread_pct / 100 * u[2]))
        skew = float(np.clip(design.skew + design.skew_jitter * u[3], -MAX_ABS_SKEW, MAX_ABS_SKEW))
        if length <= 0:
            raise ValueError("period length must be positive")
        periods.append(Period(float(length), float(amp), skew, float(delay)))
    return periods


# ----------------------------------------------------------------------------
# Normalized shapes f(u), u in [0, 1], f(0) = f(1) = 0, f(peak) = 1
# ----------------------------------------------------------------------------

_GAUSS_END = math.exp(-0.5 * GAUSS_N_SIGMA ** 2)


def peak_position(skew: float) -> float:
    skew = float(np.clip(skew, -MAX_ABS_SKEW, MAX_ABS_SKEW))
    return (1.0 + skew) / 2.0


def _gauss_sigmas(a):
    return a / GAUSS_N_SIGMA, (1.0 - a) / GAUSS_N_SIGMA


def shape_value(shape: str, u, skew: float):
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    a = peak_position(skew)
    rise = u < a
    if shape == "sinusoid":
        return np.where(rise, 0.5 * (1 - np.cos(np.pi * u / a)),
                        0.5 * (1 + np.cos(np.pi * (u - a) / (1 - a))))
    if shape == "triangle":
        return np.where(rise, u / a, (1 - u) / (1 - a))
    if shape == "gaussian":
        sl, sr = _gauss_sigmas(a)
        d = np.where(rise, (a - u) / sl, (u - a) / sr)
        return (np.exp(-0.5 * d * d) - _GAUSS_END) / (1 - _GAUSS_END)
    raise ValueError(f"unknown shape {shape!r}")


def shape_derivative(shape: str, u, skew: float):
    """df/du."""
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    a = peak_position(skew)
    rise = u < a
    if shape == "sinusoid":
        return np.where(rise, 0.5 * np.pi / a * np.sin(np.pi * u / a),
                        -0.5 * np.pi / (1 - a) * np.sin(np.pi * (u - a) / (1 - a)))
    if shape == "triangle":
        return np.where(rise, 1 / a, -1 / (1 - a))
    if shape == "gaussian":
        sl, sr = _gauss_sigmas(a)
        d = np.where(rise, (a - u) / sl, (u - a) / sr)
        e = np.exp(-0.5 * d * d) / (1 - _GAUSS_END)
        return np.where(rise, e * d / sl, -e * d / sr)
    raise ValueError(f"unknown shape {shape!r}")


def shape_second_derivative(shape: str, u, skew: float):
    """d2f/du2 (0 inside the ramps of the triangle; its corners are reported separately)."""
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    a = peak_position(skew)
    rise = u < a
    if shape == "sinusoid":
        return np.where(rise, 0.5 * (np.pi / a) ** 2 * np.cos(np.pi * u / a),
                        -0.5 * (np.pi / (1 - a)) ** 2 * np.cos(np.pi * (u - a) / (1 - a)))
    if shape == "triangle":
        return np.zeros_like(u)
    if shape == "gaussian":
        sl, sr = _gauss_sigmas(a)
        s = np.where(rise, sl, sr)
        d = np.where(rise, (a - u) / sl, (u - a) / sr)
        return -np.exp(-0.5 * d * d) * (1 - d * d) / (s * s * (1 - _GAUSS_END))
    raise ValueError(f"unknown shape {shape!r}")


def shape_inverse(shape: str, f, skew: float, rising: bool):
    """u such that shape_value(u) == f on the rising or the falling branch."""
    f = np.clip(np.asarray(f, dtype=float), 0.0, 1.0)
    a = peak_position(skew)
    if shape == "sinusoid":
        if rising:
            return a / np.pi * np.arccos(1 - 2 * f)
        return a + (1 - a) / np.pi * np.arccos(2 * f - 1)
    if shape == "triangle":
        return a * f if rising else 1 - (1 - a) * f
    if shape == "gaussian":
        sl, sr = _gauss_sigmas(a)
        d = np.sqrt(-2.0 * np.log(f * (1 - _GAUSS_END) + _GAUSS_END))
        return a - d * sl if rising else a + d * sr
    raise ValueError(f"unknown shape {shape!r}")


# ----------------------------------------------------------------------------
# Ideal trajectory
# ----------------------------------------------------------------------------

def period_starts(periods: list[Period]) -> np.ndarray:
    """Start time (s) of each period and, as the last element, the sequence length."""
    d = [0.0]
    for p in periods:
        d.append(d[-1] + (p.length_ms + p.delay_ms) / 1000.0)
    return np.array(d)


def sequence_length_s(periods: list[Period]) -> float:
    return float(period_starts(periods)[-1])


def _per_period(fn, power, shape, periods, t):
    """amplitude * fn(u) / length**power inside each period's motion, 0 in the dwell."""
    starts = period_starts(periods)
    t = np.mod(np.asarray(t, dtype=float), starts[-1])
    idx = np.clip(np.searchsorted(starts, t, side="right") - 1, 0, len(periods) - 1)
    lengths = np.array([p.length_ms / 1000.0 for p in periods])
    u = (t - starts[idx]) / lengths[idx]
    moving = u <= 1.0
    out = np.zeros_like(u)
    for i, p in enumerate(periods):
        m = (idx == i) & moving
        if m.any():
            out[m] = p.amplitude_deg * fn(shape, u[m], p.skew) / lengths[i] ** power
    return out


def ideal_position_deg(shape, periods, t):
    return _per_period(shape_value, 0, shape, periods, t)


def ideal_velocity_dps(shape, periods, t):
    return _per_period(shape_derivative, 1, shape, periods, t)


def ideal_accel_dps2(shape, periods, t):
    return _per_period(shape_second_derivative, 2, shape, periods, t)


def deg_per_step(microstep: int) -> float:
    return DEG_PER_FULL_STEP / microstep


def degrees_to_steps(deg, microstep: int):
    return np.asarray(deg) / DEG_PER_FULL_STEP * microstep


def amplitude_steps(p: Period, microstep: int) -> int:
    """The period's amplitude rounded to whole microsteps (signed)."""
    n = int(round(abs(float(degrees_to_steps(p.amplitude_deg, microstep)))))
    return n if p.amplitude_deg > 0 else -n


def step_times(shape: str, periods: list[Period], microstep: int):
    """Times (s) and directions (+1/-1) of the steps of one sequence.

    The amplitude of each period is rounded to an integer number of microsteps
    N, and a step is placed where the scaled curve crosses each level k + 0.5
    (k = 0..N-1), once on the rising and once on the falling branch, so every
    period has zero net steps and no step chatter at the turning point.
    """
    starts = period_starts(periods)
    times, dirs = [], []
    for i, p in enumerate(periods):
        a = amplitude_steps(p, microstep)
        n, sign = abs(a), (1 if a > 0 else -1)
        if n == 0:
            continue
        levels = (np.arange(n) + 0.5) / n
        length = p.length_ms / 1000.0
        up = starts[i] + length * shape_inverse(shape, levels, p.skew, rising=True)
        down = starts[i] + length * shape_inverse(shape, levels[::-1], p.skew, rising=False)
        times.extend([up, down])
        dirs.extend([np.full(n, sign), np.full(n, -sign)])
    if not times:
        return np.zeros(0), np.zeros(0, dtype=int)
    return np.concatenate(times), np.concatenate(dirs).astype(int)


def quantized_amplitudes_deg(periods, microstep):
    return [amplitude_steps(p, microstep) * deg_per_step(microstep) for p in periods]


def peak_step_rate_hz(times_s, cycle_s: float) -> float:
    """Highest step rate of a looped sequence, including the gap across the cycle wrap."""
    if len(times_s) == 0:
        return 0.0
    gaps = np.diff(np.concatenate([times_s, [times_s[0] + cycle_s]]))
    return float(1.0 / gaps.min()) if gaps.min() > 0 else math.inf


# ----------------------------------------------------------------------------
# Firmware engine simulation (bit-exact mirror of seg_engine.h)
# ----------------------------------------------------------------------------

def simulate_engine(segments, cycles: int = 1, max_events: int = 10_000_000):
    """Run the integer engine exactly like the firmware.

    Returns (times, kinds, dirs) as int64 arrays: absolute event tick (from the
    trigger), event kind (EV_STEP / EV_DELAY / EV_END) and DIR level (0/1).
    ``cycles`` = 0 is not allowed here (it would never end).
    """
    if cycles <= 0:
        raise ValueError("simulate_engine needs a finite number of cycles")
    segs = [tuple(int(v) for v in s) for s in segments]
    nseg = len(segs)
    idx = 0xFFFF
    left = 0
    interval = add = add2 = frac = flags = 0
    cycle = 0
    t = 0
    times, kinds, dirs = [], [], []

    def enter():
        nonlocal idx, left, interval, add, add2, frac, flags, cycle
        idx = (idx + 1) & 0xFFFF
        if idx >= nseg:
            cycle += 1
            if cycle >= cycles:
                idx = nseg
                return False
            idx = 0
            frac = 0
        c, i, a, a2, f = segs[idx]
        flags = f
        left = 1 if f & FLAG_WAIT else c
        interval = i & U32
        add = a & U32
        add2 = a2 & U32
        return True

    while len(times) < max_events:
        delay = 0
        guard = 0
        kind = None
        while True:
            if left == 0:
                if not enter():
                    kind = EV_END
                    break
                guard += 1
                if guard > nseg + 1:
                    kind = EV_DELAY
                    break
                if left == 0:
                    continue
            if flags & FLAG_WAIT:
                delay = (delay + interval) & U32
                left = 0
                continue
            acc = (frac + interval) & U32
            delay = (delay + (acc >> 16)) & U32
            frac = acc & 0xFFFF
            interval = (interval + add) & U32
            add = (add + add2) & U32
            left -= 1
            kind = EV_STEP
            break
        t += delay
        times.append(t)
        kinds.append(kind)
        dirs.append(flags & FLAG_DIR)
        if kind == EV_END:
            break
    return (np.array(times, dtype=np.int64), np.array(kinds, dtype=np.int64),
            np.array(dirs, dtype=np.int64))


# ----------------------------------------------------------------------------
# Segment fitting
# ----------------------------------------------------------------------------

class FitError(Exception):
    pass


@dataclass
class TimingParams:
    tick_hz: float
    tol_ticks: float
    min_ticks: int       # minimum delay between two events
    dir_extra_ticks: int  # extra delay before the first step after a DIR change
    max_gap_ticks: int   # longest step interval encoded without a WAIT


def timing_params(caps: dict, tick_hz: float, tolerance_us: float) -> TimingParams:
    min_ticks = max(2, math.ceil(caps["min_event_us"] * 1e-6 * tick_hz))
    dir_extra = math.ceil(caps.get("dir_setup_us", 1.0) * 1e-6 * tick_hz)
    tol = max(1.0, tolerance_us * 1e-6 * tick_hz)
    return TimingParams(tick_hz, tol, min_ticks, dir_extra, 60_000)


_LAWSON_ITERATIONS = 6


_BASIS_CACHE_MAX_COUNT = 2048  # cache only short bases: memory stays below ~10 MB


def _make_basis(c: int):
    """Cubic basis n, n(n-1)/2, n(n-1)(n-2)/6 for n = 1..c, and its pseudo-inverse."""
    n = np.arange(1, c + 1, dtype=np.float64)
    m = np.column_stack([n, n * (n - 1) / 2, n * (n - 1) * (n - 2) / 6])[:, :min(c, 3)]
    return m, np.linalg.pinv(m)


_cached_basis = functools.lru_cache(maxsize=128)(_make_basis)


def _basis(c: int):
    return _cached_basis(c) if c <= _BASIS_CACHE_MAX_COUNT else _make_basis(c)


def _quantize_check(p, c, s0, e_prev, need_first, tau, tp: TimingParams):
    """Quantize polynomial coefficients (ticks) and verify them with integer math."""
    ifp = int(round(p[0] * FRAC_ONE))
    afp = int(round(p[1] * FRAC_ONE)) if len(p) > 1 else 0
    bfp = int(round(p[2] * FRAC_ONE)) if len(p) > 2 else 0
    if not (-(1 << 31) <= afp < (1 << 31) and -(1 << 31) <= bfp < (1 << 31)):
        return None
    if abs(afp) * c * c > 2 ** 60 or abs(bfp) * c ** 3 > 2 ** 60:
        return None
    j = np.arange(c, dtype=np.int64)
    intervals = ifp + j * afp + bfp * (j * (j - 1) // 2)
    if intervals.min() < 0 or intervals.max() > MAX_INTERVAL_FP:
        return None
    n = j + 1
    fp = s0 + n * ifp + afp * (n * (n - 1) // 2) + bfp * (n * (n - 1) * (n - 2) // 6)
    ev = fp >> FRAC_BITS
    if np.max(np.abs(ev - tau)) > tp.tol_ticks:
        return None
    if ev[0] - e_prev < need_first:
        return None
    if c > 1 and np.min(np.diff(ev)) < tp.min_ticks:
        return None
    return (c, ifp, afp, bfp), int(fp[-1]), int(ev[-1])


def _try_segment(tau, s0, e_prev, need_first, tp: TimingParams):
    """Fit one cubic segment to the targets ``tau`` starting at fixed-point time s0.

    Least squares first, then a few Lawson (iteratively reweighted) passes that
    move the fit towards the minimax solution, since the tolerance is a max error.
    """
    c = len(tau)
    m, pinv = _basis(c)
    y = tau + 0.5 - s0 / FRAC_ONE
    # Aim the first step at the earliest allowed tick when the target is just
    # before it (within tolerance); the error is still checked against tau.
    earliest = e_prev + need_first
    if tau[0] < earliest <= tau[0] + tp.tol_ticks:
        y[0] = earliest + 0.5 - s0 / FRAC_ONE
    p = pinv @ y
    w = np.ones(c)
    for it in range(_LAWSON_ITERATIONS if c > 3 else 1):
        if it:
            sw = np.sqrt(w)
            p, *_ = np.linalg.lstsq(m * sw[:, None], y * sw, rcond=None)
        r = _quantize_check(p, c, s0, e_prev, need_first, tau, tp)
        if r is not None:
            return r
        res = np.abs(m @ p - y)
        if res.max() > 4 * tp.tol_ticks:
            return None  # hopeless, do not waste iterations
        w = w * (res + 1e-12)
        w /= w.sum()
    return None


def fit_segments(tau: np.ndarray, dirs: np.ndarray, total_ticks: int, tp: TimingParams,
                 max_segments: Optional[int] = None):
    """Encode step target times ``tau`` (float ticks) into firmware segments."""
    n = len(tau)
    if n == 0:
        raise FitError("the trajectory has no steps")
    segs = []
    s = 0          # firmware time in 16.16 fixed point
    e_prev = 0     # tick of the previous event
    prev_dir = None
    i = 0
    while i < n:
        if max_segments is not None and len(segs) > max_segments:
            raise FitError(f"more than {max_segments} segments needed")
        d = int(dirs[i])
        run_end = i + 1
        while run_end < n and dirs[run_end] == d:
            run_end += 1
        need_first = tp.min_ticks
        if prev_dir is not None and d != prev_dir:
            need_first += tp.dir_extra_ticks
        gap = tau[i] - s / FRAC_ONE
        if gap > tp.max_gap_ticks:
            w = int(gap) - tp.max_gap_ticks // 2
            segs.append((0, w, 0, 0, FLAG_WAIT))
            s += w * FRAC_ONE

        def attempt(c):
            return _try_segment(tau[i:i + c], s, e_prev, need_first, tp)

        best = attempt(1)
        if best is None:
            gap_us = (tau[i] - e_prev) / tp.tick_hz * 1e6
            raise FitError(f"step {i} comes {gap_us:.1f} us after the previous event; the "
                           f"device needs at least {need_first / tp.tick_hz * 1e6:.1f} us "
                           f"(step rate too high)")
        # Longest segment that fits: double the count, then bisect.
        good, bad = 1, None
        limit = min(run_end - i, MAX_COUNT)
        while good < limit:
            c = min(2 * good, limit)
            r = attempt(c)
            if r is None:
                bad = c
                break
            best, good = r, c
        while bad is not None and bad - good > 1:
            mid = (good + bad) // 2
            r = attempt(mid)
            if r is None:
                bad = mid
            else:
                best, good = r, mid
        (cnt, ifp, afp, bfp), s, e_prev = best
        segs.append((cnt, ifp, afp, bfp, FLAG_DIR if d > 0 else 0))
        prev_dir = d
        i += good
    tail = total_ticks - e_prev
    if tail < 0:
        raise FitError("the last step lies after the end of the sequence")
    first_event = int(simulate_engine(segs, 1, max_events=1)[0][0])
    wrap_needs = tp.min_ticks + (tp.dir_extra_ticks if dirs[-1] != dirs[0] else 0)
    if tail + first_event < wrap_needs:
        raise FitError("no gap between the end of the sequence and the start of the next cycle")
    while tail > 0:
        w = min(tail, 0x7FFFFFFF)
        segs.append((0, w, 0, 0, FLAG_WAIT))
        tail -= w
    if max_segments is not None and len(segs) > max_segments:
        raise FitError(f"{len(segs)} segments needed, device holds {max_segments}")
    return segs


@dataclass
class Verification:
    events: tuple          # simulate_engine(segments, 1)
    step_ticks: np.ndarray  # firmware tick of every step of one cycle
    max_error_ticks: float
    min_gap_ticks: int     # closest spacing of two events, across the cycle wrap too


def min_event_gap(times, kinds, total_ticks: int) -> int:
    """Closest spacing of the timed events of one looped cycle (EV_END excluded)."""
    t = times[kinds != EV_END]
    if len(t) == 0:
        return total_ticks
    wrap = total_ticks - t[-1] + t[0]  # last event -> first event of the next cycle
    return int(min(np.diff(np.concatenate([[0], t])).min(), wrap))


def verify_segments(segs, tau, dirs, total_ticks, tp: TimingParams) -> Verification:
    """Simulate one cycle of the firmware and check it against the targets.

    Every cycle is identical (``frac`` is reset at the cycle start), so one
    cycle plus the gap across the wrap covers a looped sequence.
    """
    events = simulate_engine(segs, 1)
    times, kinds, levels = events
    step = kinds == EV_STEP
    st, sd = times[step], levels[step]
    if len(st) != len(tau):
        raise FitError(f"simulation gave {len(st)} steps, expected {len(tau)}")
    if kinds[-1] != EV_END or times[-1] != total_ticks:
        raise FitError("sequence length in ticks is not exact")
    err = np.abs(st - tau)
    if err.max() > tp.tol_ticks:
        raise FitError(f"timing error {err.max():.2f} ticks exceeds tolerance")
    if np.any((sd == 1) != (dirs > 0)):
        raise FitError("direction mismatch")
    gap = min_event_gap(times, kinds, total_ticks)
    if gap < tp.min_ticks:
        raise FitError("an event is closer than the minimum delay")
    return Verification(events, st, float(err.max()), gap)


# ----------------------------------------------------------------------------
# Checksum (CRC-16/CCITT-FALSE over the packed little-endian segments)
# ----------------------------------------------------------------------------

def pack_segment(seg) -> bytes:
    c, i, a, a2, f = seg
    return struct.pack("<HIiiB", c, i, a, a2, f)


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    return binascii.crc_hqx(data, crc)


def segments_checksum(segs) -> int:
    return crc16_ccitt(b"".join(pack_segment(s) for s in segs))


# ----------------------------------------------------------------------------
# Compiled trajectory
# ----------------------------------------------------------------------------

@dataclass
class CompiledTrajectory:
    design: Design
    limits: Limits
    periods: list[Period]
    microstep: int
    tick_hz: int
    segments: list[tuple[int, int, int, int, int]]
    total_ticks: int
    device_caps: dict
    metrics: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    created: str = ""

    @property
    def driver(self) -> str:
        return self.limits.driver

    @functools.cached_property
    def checksum(self) -> int:
        return segments_checksum(self.segments)

    @functools.cached_property
    def events(self):
        """simulate_engine(segments, 1): the firmware's events of one cycle."""
        return simulate_engine(self.segments, 1)

    @property
    def cycle_s(self) -> float:
        return self.total_ticks / self.tick_hz

    def step_targets(self):
        t, d = step_times(self.design.shape, self.periods, self.microstep)
        return t * self.tick_hz, d

    def to_dict(self) -> dict:
        return {
            "format": FORMAT_NAME,
            "version": FORMAT_VERSION,
            "created": self.created,
            "microstep": self.microstep,
            "driver": self.limits.driver,
            "tick_hz": self.tick_hz,
            "full_steps_per_rev": FULL_STEPS_PER_REV,
            "deg_per_full_step": DEG_PER_FULL_STEP,
            "design": dataclasses.asdict(self.design),
            "limits": dataclasses.asdict(self.limits),
            "periods": [dataclasses.asdict(p) for p in self.periods],
            "segment_format": {"fields": ["count", "interval_fp", "add_fp", "add2_fp", "flags"],
                               "frac_bits": FRAC_BITS, "flag_dir": FLAG_DIR,
                               "flag_wait": FLAG_WAIT},
            "segments": [list(s) for s in self.segments],
            "totals": {"cycle_ticks": self.total_ticks, "cycle_s": self.cycle_s,
                       "steps_per_cycle": self.metrics.get("steps_per_cycle"),
                       "n_segments": len(self.segments),
                       "memory_bytes": len(self.segments) * SEGMENT_BYTES,
                       "checksum": self.checksum},
            "metrics": self.metrics,
            "warnings": self.warnings,
            "device_caps": self.device_caps,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "CompiledTrajectory":
        if d.get("format") != FORMAT_NAME:
            raise ValueError("not a phantom trajectory file")
        if int(d.get("version", 0)) > FORMAT_VERSION:
            raise ValueError(f"file format version {d['version']} is newer than supported")
        if d.get("segment_format", {}).get("frac_bits", FRAC_BITS) != FRAC_BITS:
            raise ValueError("unsupported fixed-point format")
        traj = cls(
            design=Design(**d["design"]),
            limits=Limits(**d["limits"]),
            periods=[Period(**p) for p in d["periods"]],
            microstep=int(d["microstep"]),
            tick_hz=int(d["tick_hz"]),
            segments=[tuple(int(v) for v in s) for s in d["segments"]],
            total_ticks=int(d["totals"]["cycle_ticks"]),
            device_caps=d.get("device_caps") or {},
            metrics=d.get("metrics", {}),
            warnings=d.get("warnings", []),
            created=d.get("created", ""),
        )
        if traj.checksum != int(d["totals"]["checksum"]):
            raise ValueError("segment checksum mismatch (file corrupted?)")
        return traj

    def save(self, path) -> None:
        with open(path, "w") as fh:
            json.dump(self.to_dict(), fh, indent=1)

    @classmethod
    def load(cls, path) -> "CompiledTrajectory":
        with open(path) as fh:
            return cls.from_dict(json.load(fh))


# ----------------------------------------------------------------------------
# Metrics and curves
# ----------------------------------------------------------------------------

def _dense_time(periods, per_period=2000):
    starts = period_starts(periods)
    parts = []
    for i, p in enumerate(periods):
        parts.append(np.linspace(starts[i], starts[i] + p.length_ms / 1000.0, per_period))
        if p.delay_ms > 0:
            parts.append([starts[i + 1] - 1e-9])
    t = np.unique(np.concatenate(parts))
    return t[t < starts[-1]]


def real_position_steps(event_ticks, event_dirs, t_ticks):
    """Position (microsteps) after all steps with tick <= t."""
    signs = np.where(np.asarray(event_dirs) > 0, 1, -1)
    cum = np.concatenate([[0], np.cumsum(signs)])
    return cum[np.searchsorted(event_ticks, t_ticks, side="right")]


def corner_speeds(shape, periods):
    """|speed| (deg/s) at start, peak (both sides) and end of each period's motion."""
    out = []
    for p in periods:
        length = p.length_ms / 1000.0
        a = peak_position(p.skew)
        eps = 1e-9
        vals = shape_derivative(shape, [0.0, a - eps, a + eps, 1.0], p.skew)
        out.append(np.abs(vals) * abs(p.amplitude_deg) / length)
    return np.array(out)


def compute_metrics(traj: CompiledTrajectory, ver: Verification, tau, dirs) -> dict:
    """Metrics of a verified trajectory; ``tau``/``dirs`` are its step targets."""
    shape, periods, ms, f = traj.design.shape, traj.periods, traj.microstep, traj.tick_hz
    real_ticks = ver.step_ticks
    t = _dense_time(periods)
    vel = ideal_velocity_dps(shape, periods, t)
    acc = ideal_accel_dps2(shape, periods, t)
    dsteps = deg_per_step(ms)
    ideal = ideal_position_deg(shape, periods, t)
    real = real_position_steps(real_ticks, dirs, t * f) * dsteps
    pos_err = np.abs(ideal - real)
    # The staircase error is largest just before and after each step.
    if len(real_ticks):
        st = real_ticks / f
        before = real_position_steps(real_ticks, dirs, real_ticks - 1) * dsteps
        after = real_position_steps(real_ticks, dirs, real_ticks) * dsteps
        ideal_st = ideal_position_deg(shape, periods, st)
        pos_err_max = max(pos_err.max(), np.abs(ideal_st - before).max(), np.abs(ideal_st - after).max())
    else:
        pos_err_max = float(pos_err.max())
    corners = corner_speeds(shape, periods)
    return {
        "peak_speed_dps": float(np.abs(vel).max()),
        "peak_accel_dps2": float(np.abs(acc).max()),
        "peak_step_rate_hz": peak_step_rate_hz(tau / f, traj.cycle_s),
        "max_start_stop_speed_dps": float(corners[:, [0, 3]].max()),
        "max_turn_speed_dps": float(corners[:, [1, 2]].max()),
        "n_segments": len(traj.segments),
        "memory_bytes": len(traj.segments) * SEGMENT_BYTES,
        "steps_per_cycle": int(len(tau)),
        "cycle_s": traj.cycle_s,
        "max_timing_error_us": float(ver.max_error_ticks / f * 1e6),
        "min_event_gap_ticks": ver.min_gap_ticks,
        "max_position_error_deg": float(pos_err_max),
        "amplitude_quantization_deg": float(max(abs(q - p.amplitude_deg) for q, p in
                                                zip(quantized_amplitudes_deg(periods, ms), periods))),
    }


def trajectory_curves(traj: CompiledTrajectory, n_cycles: int = 1, per_period: int = 1500):
    """Arrays for plotting: ideal vs real position, error, speed and step rate."""
    shape, periods, f = traj.design.shape, traj.periods, traj.tick_hz
    # Every cycle is identical: repeat the one-cycle simulation.
    times, kinds, levels = traj.events
    step = kinds == EV_STEP
    st = np.concatenate([times[step] + k * traj.total_ticks for k in range(n_cycles)])
    sd = np.tile(np.where(levels[step] == 1, 1, -1), n_cycles)
    total = traj.cycle_s
    base = _dense_time(periods, per_period)
    t = np.concatenate([base + k * total for k in range(n_cycles)])
    # Add the step instants (both sides) so the staircase is drawn exactly.
    ts = st / f
    t = np.unique(np.concatenate([t, ts, ts - 1.0 / f]))
    ideal = ideal_position_deg(shape, periods, t)
    real = real_position_steps(st, sd, np.round(t * f)) * deg_per_step(traj.microstep)
    rate_t = ts[1:] if len(ts) > 1 else np.zeros(0)
    rate = 1.0 / np.diff(ts) if len(ts) > 1 else np.zeros(0)
    return {
        "t": t, "ideal_deg": ideal, "real_deg": real, "error_deg": real - ideal,
        "speed_dps": ideal_velocity_dps(shape, periods, t),
        "step_t": ts, "step_dir": sd, "rate_t": rate_t, "step_rate_hz": rate,
    }


# ----------------------------------------------------------------------------
# Ground truth
# ----------------------------------------------------------------------------

def ground_truth(traj: CompiledTrajectory, latency_us: Optional[float] = None):
    """The real motion of one cycle relative to the trigger (MRI TTL rising edge).

    Returns (t_s, position_deg): the time of every step since the trigger edge
    and the position right after it. The device's constant trigger latency
    (CAPS ``trigger_latency_us``) is included. Cycle k is the same, shifted by
    k * ``traj.cycle_s``.
    """
    if latency_us is None:
        latency_us = float(traj.device_caps.get("trigger_latency_us", 0.0))
    times, kinds, levels = traj.events
    step = kinds == EV_STEP
    t = times[step] / traj.tick_hz + latency_us * 1e-6
    pos = np.cumsum(np.where(levels[step] == 1, 1, -1)) * deg_per_step(traj.microstep)
    return t, pos


def export_ground_truth_csv(traj: CompiledTrajectory, path, latency_us: Optional[float] = None) -> None:
    """Write ``ground_truth`` as CSV (time_s, position_deg) with a commented header."""
    if latency_us is None:
        latency_us = float(traj.device_caps.get("trigger_latency_us", 0.0))
    t, pos = ground_truth(traj, latency_us)
    header = "\n".join([
        "phantom ground truth: position after every step, time from the MRI TTL rising edge",
        f"t = 0: TTL rising edge; includes the trigger latency of {latency_us:g} us",
        f"position 0 deg at t = 0; one cycle shown, it repeats every {traj.cycle_s:.9f} s",
        f"microstep 1/{traj.microstep}, {deg_per_step(traj.microstep):g} deg/step, "
        f"tick {traj.tick_hz} Hz, checksum {traj.checksum}, board {traj.device_caps.get('board', '?')}",
        "time_s,position_deg",
    ])
    np.savetxt(path, np.column_stack([t, pos]), delimiter=",", fmt=["%.9f", "%.6f"],
               header=header, comments="# ")


# ----------------------------------------------------------------------------
# Planner
# ----------------------------------------------------------------------------

@dataclass
class PlanResult:
    trajectory: Optional[CompiledTrajectory]
    attempts: list[dict]
    error: Optional[str] = None


def limit_warnings(design: Design, limits: Limits, periods, metrics: dict) -> list[str]:
    w = []
    if metrics["peak_speed_dps"] > limits.max_speed_dps:
        w.append(f"peak speed {metrics['peak_speed_dps']:.0f} deg/s exceeds the limit "
                 f"{limits.max_speed_dps:.0f} deg/s")
    if design.shape != "triangle" and metrics["peak_accel_dps2"] > limits.max_accel_dps2:
        w.append(f"peak acceleration {metrics['peak_accel_dps2']:.0f} deg/s^2 exceeds the limit "
                 f"{limits.max_accel_dps2:.0f} deg/s^2")
    if metrics["max_start_stop_speed_dps"] > limits.start_stop_speed_dps:
        w.append(f"speed at the start/end of a period is {metrics['max_start_stop_speed_dps']:.0f} "
                 f"deg/s, above the start/stop limit {limits.start_stop_speed_dps:.0f} deg/s")
    if design.shape == "triangle":
        w.append(f"triangle reverses instantly at the peak (speed jumps by up to "
                 f"{2 * metrics['max_turn_speed_dps']:.0f} deg/s); check for lost steps")
    if metrics["amplitude_quantization_deg"] > 1e-9:
        w.append(f"amplitude rounded to whole microsteps (up to "
                 f"{metrics['amplitude_quantization_deg']:.4f} deg)")
    return w


def plan(design: Design, limits: Limits, caps: dict, periods: Optional[list[Period]] = None) -> PlanResult:
    """Choose the largest microstep and finest tick that the device can play."""
    if periods is None:
        periods = generate_periods(design)
    allowed = limits.allowed_microsteps or driver_microsteps(limits.driver)
    allowed = sorted({m for m in allowed if m in DRIVERS[limits.driver]["table"]}, reverse=True)
    if not allowed:
        return PlanResult(None, [], f"no allowed microstep is supported by {limits.driver}")
    max_rate = min(caps["max_step_rate_hz"], DRIVERS[limits.driver]["max_step_rate_hz"])
    attempts = []
    total_s = sequence_length_s(periods)
    for ms in allowed:
        t, d = step_times(design.shape, periods, ms)
        if len(t) == 0:
            attempts.append({"microstep": ms, "tick_hz": None, "result": "no steps (amplitude too small)"})
            continue
        rate = peak_step_rate_hz(t, total_s)
        if rate > max_rate:
            attempts.append({"microstep": ms, "tick_hz": None,
                             "result": f"peak step rate {rate:.0f} Hz > {max_rate:.0f} Hz"})
            continue
        for tick in sorted(caps["tick_hz"], reverse=True):
            if 1e6 / tick > limits.tolerance_us:
                attempts.append({"microstep": ms, "tick_hz": tick,
                                 "result": f"tick {1e6 / tick:.1f} us coarser than the tolerance"})
                continue
            total_ticks = int(round(total_s * tick))
            if total_ticks >= 2 ** 32:
                attempts.append({"microstep": ms, "tick_hz": tick, "result": "cycle too long for 32-bit ticks"})
                continue
            tp = timing_params(caps, tick, limits.tolerance_us)
            tau = t * tick
            try:
                segs = fit_segments(tau, d, total_ticks, tp, caps["max_segments"])
                ver = verify_segments(segs, tau, d, total_ticks, tp)
            except FitError as exc:
                attempts.append({"microstep": ms, "tick_hz": tick, "result": str(exc)})
                continue
            attempts.append({"microstep": ms, "tick_hz": tick, "result": f"ok, {len(segs)} segments"})
            traj = CompiledTrajectory(design=design, limits=limits, periods=periods, microstep=ms,
                                      tick_hz=int(tick), segments=segs, total_ticks=total_ticks,
                                      device_caps=dict(caps),
                                      created=time.strftime("%Y-%m-%dT%H:%M:%S"))
            traj.__dict__["events"] = ver.events  # prime the cached simulation
            traj.metrics = compute_metrics(traj, ver, tau, d)
            traj.warnings = limit_warnings(design, limits, periods, traj.metrics)
            return PlanResult(traj, attempts)
    return PlanResult(None, attempts, "no microstep/tick combination fits the device")


def compile_design(design: Design, limits: Limits, caps: dict, periods=None) -> CompiledTrajectory:
    res = plan(design, limits, caps, periods)
    if res.trajectory is None:
        detail = "; ".join(f"1/{a['microstep']} @ {a['tick_hz']} Hz: {a['result']}" for a in res.attempts)
        raise FitError(f"{res.error}. Tried: {detail}")
    return res.trajectory


# ----------------------------------------------------------------------------
# Compatibility check against a connected device
# ----------------------------------------------------------------------------

def compatibility_problems(traj: CompiledTrajectory, caps: dict) -> list[str]:
    """Empty list if ``traj`` can be played on the device described by ``caps``
    (device capabilities only; the driver/microstep check is ``driver_problem``)."""
    p = []
    if caps.get("protocol") != PROTOCOL_VERSION:
        p.append(f"unsupported protocol version {caps.get('protocol')}")
    if caps.get("frac_bits", FRAC_BITS) != FRAC_BITS:
        p.append("device uses a different fixed-point format")
    if traj.tick_hz not in [int(x) for x in caps.get("tick_hz", [])]:
        p.append(f"tick rate {traj.tick_hz} Hz not offered by the device")
    if len(traj.segments) > caps.get("max_segments", 0):
        p.append(f"{len(traj.segments)} segments, device holds {caps.get('max_segments')}")
    rate = traj.metrics.get("peak_step_rate_hz", 0)
    if rate > caps.get("max_step_rate_hz", 0):
        p.append(f"peak step rate {rate:.0f} Hz above device limit {caps.get('max_step_rate_hz')} Hz")
    tp = timing_params(caps, traj.tick_hz, traj.limits.tolerance_us)
    gap = traj.metrics.get("min_event_gap_ticks")
    if gap is None:
        times, kinds, _ = traj.events
        gap = min_event_gap(times, kinds, traj.total_ticks)
    if gap < tp.min_ticks:
        p.append("events closer together than the device's minimum event spacing")
    return p


def driver_problem(microstep: int, driver: str) -> Optional[str]:
    if microstep not in DRIVERS[driver]["table"]:
        return f"{driver} cannot do 1/{microstep} microstepping"
    return None
