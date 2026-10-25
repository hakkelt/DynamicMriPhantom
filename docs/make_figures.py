"""Regenerate the figures in docs/img (needs the emulator for ttl_timing.png).

    npm ci --prefix emulator/avr && emulator/build_avr.sh
    python docs/make_figures.py
"""
import os
import sys
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, "python_serial"), os.path.join(ROOT, "emulator")]
import emulator as emumod  # noqa: E402
import phantom_core as pc  # noqa: E402
from phantom_device import PhantomDevice  # noqa: E402

OUT = os.path.join(ROOT, "docs", "img")
IDEAL = "#2a78d6"   # categorical slot 1
REAL = "#eb6834"    # categorical slot 2
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
SURFACE = "#fcfcfb"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": "#c3c2b7", "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "axes.titlelocation": "left", "legend.frameon": False,
    "lines.linewidth": 2,
})


def design_example():
    design = pc.Design(shape="sinusoid", amplitude_deg=30, length_ms=800, skew=0.4, delay_ms=150,
                       n_periods=2, length_spread_pct=15, amplitude_spread_pct=10, seed=4)
    traj = pc.compile_design(design, pc.Limits(), pc.DEFAULT_CAPS["atmega328pb"])
    c = pc.trajectory_curves(traj)
    t_ms = c["t"] * 1e3
    fig, axes = plt.subplots(3, 1, figsize=(8, 6.4), sharex=True, layout="constrained")
    ax = axes[0]
    ax.plot(t_ms, c["ideal_deg"], color=IDEAL, label="ideal")
    ax.step(t_ms, c["real_deg"], where="post", color=REAL, lw=1, label="real (simulated firmware)")
    ax.set_title(f"Skewed sinusoid, 2 irregular periods — 1/{traj.microstep} microstep, "
                 f"{len(traj.segments)} segments on the ATmega328PB")
    ax.set_ylabel("position (deg)")
    ax.legend(loc="upper center")
    ax = axes[1]
    ax.plot(t_ms, c["error_deg"], color=INK2, lw=0.6)
    ax.set_title(f"Real − ideal (1 microstep = {pc.deg_per_step(traj.microstep):.4f}°, "
                 f"plus amplitude rounding)")
    ax.set_ylabel("error (deg)")
    ax = axes[2]
    ax.plot(c["rate_t"] * 1e3, c["step_rate_hz"] / 1e3, ".", ms=2, color=REAL)
    ax.set_title("Step rate (one dot per step)")
    ax.set_ylabel("kHz")
    ax.set_xlabel("time from the TTL edge (ms)")
    fig.savefig(os.path.join(OUT, "design_example.png"), dpi=110)
    plt.close(fig)


def ttl_timing():
    if not (emumod.node_available() and os.path.exists(emumod.DEFAULT_HEX)):
        print("emulator not available: skipping ttl_timing.png")
        return
    with emumod.AvrEmulator() as emu:
        dev = PhantomDevice(emu.url, timeout=10)
        dev.open()
        traj = pc.compile_design(pc.Design(shape="triangle", amplitude_deg=12, length_ms=240, delay_ms=60),
                                 pc.Limits(start_stop_speed_dps=1e9), dev.caps)
        dev.upload(traj, cycles=2, trigger="ttl")
        dev.arm()
        time.sleep(0.2)
        t0 = emu.ttl(100)
        time.sleep(0.15)
        t1 = emu.ttl(100)  # a later edge, e.g. the next TR: must not restart the motion
        dev.wait_event("EV DONE", timeout=60)
        trace = emu.trace()
        dev_latency = dev.caps["trigger_latency_us"] * 1e-6
        dev.close()
    f = emumod.F_CPU

    def lane(pin):
        pts = [(c, v) for c, p, v in trace if p == pin]
        return np.array([c for c, _ in pts], dtype=float), np.array([v for _, v in pts])

    def level_at(c, v, when):
        """Pin level at cycle(s) ``when`` (0 before the first recorded edge)."""
        i = np.searchsorted(c, when, side="right") - 1
        return np.where(i >= 0, v[np.clip(i, 0, None)], 0) if len(c) else np.zeros_like(when)

    sc, sv = lane("step")
    rise = sc[sv == 1]
    dc, dv = lane("dir")
    sign = np.where(level_at(dc, dv, rise) == 1, 1, -1)
    pos = np.concatenate([[0], np.cumsum(sign)]) * pc.deg_per_step(traj.microstep)
    gt_t, _ = pc.ground_truth(traj)
    lat_us = float(np.median((rise[:len(gt_t)] - t0) / f - (gt_t - dev_latency)) * 1e6)

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), layout="constrained",
                             gridspec_kw={"height_ratios": [1.4, 1]})
    ax = axes[0]
    total = 2 * traj.cycle_s
    tt = np.linspace(-0.05, total, 4000)
    ideal = np.where(tt >= 0, pc.ideal_position_deg("triangle", traj.periods, np.clip(tt, 0, None)), 0)
    ax.plot(tt * 1e3, ideal, color=IDEAL, label="ideal")
    t_steps = (rise - t0) / f
    ax.step(np.concatenate([[-0.05], t_steps, [total]]) * 1e3, np.append(pos, pos[-1]), where="post",
            color=REAL, lw=1, label="measured STEP/DIR (emulated ATmega328P)")
    ax.axvline(0, color=INK, lw=1, ls="--")
    top = ax.get_ylim()[1]
    ax.text(3, top * 0.97, "first MRI TTL edge = t 0\n(sequence start)", color=INK, va="top")
    t1_ms = (t1 - t0) / f * 1e3
    if t1_ms < total * 1e3:
        ax.axvline(t1_ms, color=MUTED, lw=1, ls=":")
        ax.text(t1_ms + 3, top * 0.08, "later TTL edge: ignored", color=INK2, va="bottom")
    ax.set_title("The sequence starts at the first TTL edge; later edges are ignored")
    ax.set_xlabel("time from the TTL edge (ms)")
    ax.set_ylabel("position (deg)")
    ax.legend(loc="upper right")

    ax = axes[1]
    win_us = (t_steps[2] * 1e6) * 1.15
    names = [("mri", "MRI TTL"), ("dir", "DIR"), ("step", "STEP")]
    bases = [(len(names) - 1 - k) * 1.6 for k in range(len(names))]
    for k, (pin, label) in enumerate(names):
        c, v = lane(pin)
        start, end = t0 - 50 * 16, t0 + win_us * 16
        keep = (c > start) & (c < end)
        x = np.concatenate([[start], c[keep], [end]])
        y = np.concatenate([[level_at(c, v, start)], v[keep], [level_at(c, v, end)]])
        ax.step((x - t0) / 16, bases[k] + y, where="post", color=INK, lw=1.4)
    # planned step times (without latency) as ticks on the STEP lane
    planned_us = (gt_t - dev_latency) * 1e6
    shown = planned_us[planned_us < win_us]
    ax.vlines(shown, -0.35, -0.05, color=REAL, lw=2)
    ax.text(shown[0] + 8, -0.6, f"planned steps; each real edge comes {lat_us:.1f} µs later "
            "(constant trigger latency)", color=INK2, fontsize=8, va="center")
    ax.set_xlim(-50, win_us)
    ax.set_ylim(-0.9, 4.6)
    ax.set_yticks([b + 0.5 for b in bases], [label for _, label in names])
    ax.tick_params(axis="y", length=0, labelcolor=INK2)
    ax.grid(axis="y", visible=False)
    ax.set_title("First steps: each one lands on its planned time + the trigger latency")
    ax.set_xlabel("time from the TTL edge (µs)")
    fig.savefig(os.path.join(OUT, "ttl_timing.png"), dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    design_example()
    ttl_timing()
    print("figures written to", OUT)
