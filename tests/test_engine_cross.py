"""seg_engine.h (compiled with the host C++ compiler) vs phantom_core.simulate_engine."""
import os
import shutil
import subprocess

import numpy as np
import pytest

import phantom_core as pc
from conftest import board_caps

HERE = os.path.dirname(os.path.abspath(__file__))
CXX = shutil.which(os.environ.get("CXX", "g++")) or shutil.which("c++")

pytestmark = pytest.mark.skipif(CXX is None, reason="no host C++ compiler")


@pytest.fixture(scope="module")
def engine_host(tmp_path_factory):
    exe = tmp_path_factory.mktemp("host") / "engine_host"
    subprocess.run([CXX, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    os.path.join(HERE, "host", "engine_host.cpp"), "-o", str(exe)], check=True)
    return str(exe)


def run_host(exe, segs, cycles, *args):
    lines = [f"{len(segs)} {cycles}"] + [" ".join(str(v) for v in s) for s in segs]
    out = subprocess.run([exe, *map(str, args)], input="\n".join(lines) + "\n", capture_output=True, text=True,
                         check=True).stdout.split("\n")
    crc = int(out[0].split()[1])
    rows = np.array([[int(x) for x in ln.split()] for ln in out[1:] if ln.strip()], dtype=np.int64)
    return crc, rows


PROGRAMS = {
    "sinusoid": (pc.Design(shape="sinusoid", amplitude_deg=30, skew=0.3, delay_ms=120), "atmega328pb"),
    "gaussian_irregular": (pc.Design(shape="gaussian", amplitude_deg=20, n_periods=3, length_spread_pct=15,
                                     amplitude_spread_pct=10, skew_jitter=0.2, delay_ms=50), "esp32s3"),
    "triangle": (pc.Design(shape="triangle", amplitude_deg=45, length_ms=600, skew=-0.4), "atmega328pb"),
    "slow_waits": (pc.Design(shape="sinusoid", amplitude_deg=3.6, length_ms=4000, delay_ms=700), "esp32s3"),
}


@pytest.mark.parametrize("name", sorted(PROGRAMS))
def test_bit_identical(engine_host, name):
    design, board = PROGRAMS[name]
    traj = pc.compile_design(design, pc.Limits(), board_caps(board))
    crc, rows = run_host(engine_host, traj.segments, 3)
    times, kinds, dirs = pc.simulate_engine(traj.segments, 3)
    assert crc == traj.checksum
    assert rows.shape[0] == len(times)
    assert np.array_equal(rows[:, 0], times)
    assert np.array_equal(rows[:, 1], kinds)
    assert np.array_equal(rows[:, 2][kinds == pc.EV_STEP], dirs[kinds == pc.EV_STEP])


def test_edge_cases_bit_identical(engine_host):
    # wrap-around arithmetic, zero-count segments, WAIT-only cycle, negative add/add2
    segs = [
        (0, 1000, 0, 0, pc.FLAG_WAIT),
        (5, 0xFFFF0000, -(1 << 20), 3, 1),
        (0, 0, 0, 0, 0),
        (7, 123456, 98765, -4321, 0),
        (3, 0x00010000, 0x7FFFFFFF, 0x7FFFFFFF, 1),  # interval wraps on purpose
        (0, 77, 0, 0, pc.FLAG_WAIT),
    ]
    crc, rows = run_host(engine_host, segs, 4)
    times, kinds, dirs = pc.simulate_engine(segs, 4)
    assert crc == pc.segments_checksum(segs)
    assert np.array_equal(rows[:, 0], times) and np.array_equal(rows[:, 1], kinds)
    waits_only = [(0, 500, 0, 0, pc.FLAG_WAIT), (0, 700, 0, 0, pc.FLAG_WAIT)]
    crc, rows = run_host(engine_host, waits_only, 3)
    times, kinds, _ = pc.simulate_engine(waits_only, 3)
    assert np.array_equal(rows[:, 0], times) and np.array_equal(rows[:, 1], kinds)


@pytest.mark.parametrize("name", ["sinusoid", "gaussian_irregular"])
@pytest.mark.parametrize("slots", [3, 16])
def test_stream_bit_identical(engine_host, name, slots):
    """Streaming through a small ring plays exactly the resident program."""
    design, board = PROGRAMS[name]
    traj = pc.compile_design(design, pc.Limits(), board_caps(board))
    _, rows = run_host(engine_host, traj.segments, 3, "stream", slots, 0)
    times, kinds, _ = pc.simulate_engine(traj.segments, 3)
    assert np.array_equal(rows[:, 0], times) and np.array_equal(rows[:, 1], kinds)


def test_stream_underrun(engine_host):
    """A stream that runs dry before it is closed ends with EV_UNDERRUN after
    exactly the steps it had (the firmware aborts at that event)."""
    design, board = PROGRAMS["sinusoid"]
    segs = pc.compile_design(design, pc.Limits(), board_caps(board)).segments
    k = len(segs) // 2
    _, rows = run_host(engine_host, segs, 2, "stream", 4, k)
    times, kinds, _ = pc.simulate_engine(segs[:k], 1)
    assert rows[-1, 1] == pc.EV_UNDERRUN
    assert np.array_equal(rows[rows[:, 1] == pc.EV_STEP, 0], times[kinds == pc.EV_STEP])
