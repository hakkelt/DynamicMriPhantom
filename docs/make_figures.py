"""Regenerate the figures in docs/img (only ttl_timing.png needs the emulator).

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
BAND = "#efeee8"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": "#c3c2b7", "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
    "axes.titlecolor": INK, "axes.titlelocation": "left", "legend.frameon": False,
    "lines.linewidth": 2,
})


def _period_bands(ax, periods, t_scale=1e3):
    starts = pc.period_starts(periods)
    for k, p in enumerate(periods):
        ax.axvspan(starts[k] * t_scale, (starts[k] + p.length_ms / 1e3) * t_scale,
                   color=BAND, lw=0, zorder=0)
        ax.text((starts[k] + p.length_ms / 2e3) * t_scale, 0.97, f"P{k + 1}", transform=ax.get_xaxis_transform(),
                ha="center", va="top", color=INK2, fontsize=8)


def shapes():
    u = np.linspace(0, 1, 600)
    fig, axes = plt.subplots(2, 3, figsize=(8, 4.6), layout="constrained",
                             gridspec_kw={"height_ratios": [1, 1.05]})
    for ax, shape in zip(axes[0], pc.SHAPES):
        for skew, color, ls in ((0.0, IDEAL, "-"), (0.5, REAL, "-"), (-0.5, MUTED, "--")):
            ax.plot(u, pc.shape_value(shape, u, skew), color=color, ls=ls, lw=1.8, label=f"skew {skew:+.1f}")
        ax.set_title({"sinusoid": "Skewed sinusoid", "gaussian": "Skewed Gaussian",
                      "triangle": "Skewed triangle"}[shape])
        ax.set_xlabel("time / period length")
        ax.set_ylim(-0.05, 1.12)
    axes[0][0].set_ylabel("position / amplitude")
    axes[0][2].legend(loc="upper right", fontsize=7)
    gs = axes[1][0].get_gridspec()
    for ax in axes[1]:
        ax.remove()
    ax = fig.add_subplot(gs[1, :])
    design = pc.Design(shape="sinusoid", amplitude_deg=20, length_ms=600, skew=0.3, delay_ms=120, n_periods=3,
                       length_spread_pct=20, amplitude_spread_pct=20, delay_spread_pct=40, skew_jitter=0.2, seed=7)
    periods = pc.generate_periods(design)
    total = pc.sequence_length_s(periods)
    t = np.linspace(0, 2 * total, 4000)
    x = pc.ideal_position_deg("sinusoid", periods, t % total)
    ax.plot(t * 1e3, x, color=IDEAL)
    ax.set_ylim(-1, x.max() * 1.3)
    _period_bands(ax, periods)
    ax.axvline(total * 1e3, color=INK, lw=1, ls="--")
    ax.text(total * 1e3 + 30, x.max() * 1.2, "cycle 2: the same K = 3 periods again", color=INK2)
    ax.set_title("Irregularity: K distinct periods (seeded spread of length, amplitude, skew, delay), repeated")
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("deg")
    fig.savefig(os.path.join(OUT, "shapes.png"), dpi=110)
    plt.close(fig)


def step_placement():
    ms = 1
    p = pc.Period(length_ms=200, amplitude_deg=6 * pc.deg_per_step(ms), skew=0.3, delay_ms=0)
    t, d = pc.step_times("sinusoid", [p], ms)
    n = 6
    tt = np.linspace(0, 0.2, 1000)
    x = pc.ideal_position_deg("sinusoid", [p], tt) / pc.deg_per_step(ms)
    fig, ax = plt.subplots(figsize=(8, 3.4), layout="constrained")
    for k in range(n):
        ax.axhline(k + 0.5, color=GRID, lw=1, ls="--", zorder=0)
    ax.plot(tt * 1e3, x, color=IDEAL, label="ideal x(t)")
    pos = np.concatenate([[0], np.cumsum(d)])
    ax.step(np.concatenate([[0], t, [0.2]]) * 1e3, np.append(pos, pos[-1]), where="post", color=REAL, lw=1.4,
            label="steps")
    ax.plot(t * 1e3, pc.ideal_position_deg("sinusoid", [p], t) / pc.deg_per_step(ms), "o", ms=4, color=INK,
            label="step = crossing of k + ½")
    ax.set_yticks(range(n + 1))
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("microsteps")
    ax.set_title("A step is placed where the curve crosses each half-microstep level")
    ax.legend(loc="upper right")
    fig.savefig(os.path.join(OUT, "step_placement.png"), dpi=110)
    plt.close(fig)


def _example_traj():
    design = pc.Design(shape="sinusoid", amplitude_deg=30, length_ms=800, skew=0.4, delay_ms=150)
    return pc.compile_design(design, pc.Limits(), pc.DEFAULT_CAPS["atmega328pb"])


def segments():
    traj = _example_traj()
    times, kinds, _ = traj.events
    steps = times[kinds == pc.EV_STEP]
    f = traj.tick_hz
    seg_of_step = np.concatenate([np.full(s[0], k) for k, s in enumerate(traj.segments)
                                  if not s[4] & pc.FLAG_WAIT])
    gaps_us = np.diff(np.concatenate([[0], steps])) / f * 1e6
    target, _ = pc.step_times(traj.design.shape, traj.periods, traj.microstep)
    err_us = (steps - target * f) / f * 1e6
    fig, axes = plt.subplots(2, 1, figsize=(8, 5.4), sharex=True, layout="constrained",
                             gridspec_kw={"height_ratios": [1.5, 1]})
    ax = axes[0]
    cols = (IDEAL, REAL)
    for k in np.unique(seg_of_step):
        m = seg_of_step == k
        ax.plot(steps[m] / f * 1e3, gaps_us[m] / 1e3, ".", ms=2.5, color=cols[k % 2])
    ax.set_yscale("log")
    n_steps = sum(s[0] for s in traj.segments if not s[4] & pc.FLAG_WAIT)
    n_wait = sum(1 for s in traj.segments if s[4] & pc.FLAG_WAIT)
    ax.set_title(f"Interval between steps; colours alternate per segment "
                 f"({len(traj.segments) - n_wait} cubic + {n_wait} WAIT segments for {n_steps} steps)")
    ax.set_ylabel("step interval (ms, log)")
    ax = axes[1]
    tol = traj.limits.tolerance_us
    ax.axhspan(-tol, tol, color=BAND, lw=0)
    ax.plot(steps / f * 1e3, err_us, ".", ms=2, color=INK2)
    ax.set_ylim(-tol * 1.3, tol * 1.3)
    ax.set_title(f"Step time error after quantization: max {np.abs(err_us).max():.1f} µs, "
                 f"tolerance ±{tol:g} µs (band)")
    ax.set_ylabel("µs")
    ax.set_xlabel(f"time (ms); 1/{traj.microstep} microstep, tick {f / 1e6:g} MHz")
    fig.savefig(os.path.join(OUT, "segments.png"), dpi=110)
    plt.close(fig)


def planner_search():
    caps = pc.DEFAULT_CAPS["atmega328pb"]
    design = pc.Design(shape="sinusoid", amplitude_deg=30, length_ms=800, skew=0.4, delay_ms=150, n_periods=2,
                       length_spread_pct=15, amplitude_spread_pct=10, seed=4)
    lim = pc.Limits()
    periods = pc.generate_periods(design)
    total_s = pc.sequence_length_s(periods)
    ticks = [tk for tk in sorted(caps["tick_hz"], reverse=True) if 1e6 / tk <= lim.tolerance_us]
    microsteps = [32, 16, 8, 4]
    fig, ax = plt.subplots(figsize=(8, 3.6), layout="constrained")
    width = 0.8 / len(ticks)
    for j, tick in enumerate(ticks):
        counts = []
        for ms in microsteps:
            t, d = pc.step_times(design.shape, periods, ms)
            tp = pc.timing_params(caps, tick, lim.tolerance_us)
            try:
                counts.append(len(pc.fit_segments(t * tick, d, int(round(total_s * tick)), tp)))
            except pc.FitError:
                counts.append(np.nan)
        x = np.arange(len(microsteps)) + (j - (len(ticks) - 1) / 2) * width
        ax.bar(x, counts, width * 0.9, color=(IDEAL, REAL, MUTED)[j % 3],
               label=f"tick {1e6 / tick:g} µs ({tick / 1e6:g} MHz)")
        for xi, c in zip(x, counts):
            ax.text(xi, (0 if np.isnan(c) else c) + 4, "—" if np.isnan(c) else str(int(c)),
                    ha="center", fontsize=7, color=INK2)
    ax.axhline(caps["max_segments"], color=INK, lw=1, ls="--")
    ax.text(len(microsteps) - 0.55, caps["max_segments"] + 4, f"ATmega memory: {caps['max_segments']}",
            ha="right", color=INK)
    res = pc.plan(design, lim, caps)
    tr = res.trajectory
    ax.set_xticks(range(len(microsteps)), [f"1/{m}" for m in microsteps])
    ax.set_xlabel("microstep (tried from the largest)")
    ax.set_ylabel("segments needed")
    how = "streamed" if tr.metrics["streamed"] else "resident"
    ax.set_title(f"Planner: largest microstep, then finest tick that fits → 1/{tr.microstep} at "
                 f"{tr.tick_hz / 1e6:g} MHz ({len(tr.segments)} segments, {how})")
    ms_i = microsteps.index(tr.microstep)
    j = ticks.index(tr.tick_hz)
    ax.annotate("chosen", (ms_i + (j - (len(ticks) - 1) / 2) * width, len(tr.segments) + 16), ha="center",
                xytext=(0, 18), textcoords="offset points", color=INK, fontweight="bold",
                arrowprops={"arrowstyle": "->", "color": INK})
    ax.legend(loc="upper left")
    fig.savefig(os.path.join(OUT, "planner_search.png"), dpi=110)
    plt.close(fig)


def stream_margin():
    caps = pc.DEFAULT_CAPS["atmega328pb"]
    design = pc.Design(shape="sinusoid", amplitude_deg=20, length_ms=600, skew=0.3, delay_ms=100, n_periods=5,
                       length_spread_pct=15, amplitude_spread_pct=15, skew_jitter=0.15, seed=3)
    traj = pc.compile_design(design, pc.Limits(), caps)
    segs, f = traj.segments, traj.tick_hz
    slots, n, cycles = caps["max_segments"], len(segs), 3
    # the same model as pc.stream_slack, keeping every segment's arrival
    lines = [len(pc.stream_line(s, k == 0)) + 1 for k, s in enumerate(segs)]
    byte_s = 10.0 / 115200
    line_s = np.array(lines) * byte_s
    window = max(1, (int(caps["rx_buffer"]) - 8) // max(lines))
    per_line = np.maximum(line_s, (line_s + pc.STREAM_REPLY_BYTES * byte_s + pc.STREAM_LINK_LATENCY_S) / window)
    entry = pc.segment_entry_ticks(segs) / f
    deadline = np.concatenate([entry + k * traj.cycle_s for k in range(cycles)])
    arrival = np.full(n * cycles, -np.inf)
    send = -np.inf
    for k in range(slots, n * cycles):
        send = max(send + per_line[k % n], deadline[k - slots] + pc.STREAM_LINK_LATENCY_S)
        arrival[k] = send + line_s[k % n]
    tt = np.linspace(0, cycles * traj.cycle_s, 3000)
    buffered = [np.sum((arrival <= t) & (deadline > t)) for t in tt]
    slack = (deadline - arrival)[slots:]
    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True, layout="constrained")
    ax = axes[0]
    ax.plot(tt, buffered, color=IDEAL)
    ax.axhline(slots, color=MUTED, lw=1, ls="--")
    ax.set_ylim(0, slots * 1.15)
    ax.set_ylabel("segments in the ring")
    x_close = cycles * traj.cycle_s * 0.87
    ax.annotate("prefilled before the TTL", (0.05, slots), xytext=(0.3, slots * 0.55), color=INK2,
                arrowprops={"arrowstyle": "->", "color": INK2})
    ax.annotate("CLOSE after the last cycle:\nthe ring drains", (x_close, np.interp(x_close, tt, buffered)),
                xytext=(cycles * traj.cycle_s * 0.55, slots * 0.2), color=INK2,
                arrowprops={"arrowstyle": "->", "color": INK2})
    ax.set_title(f"{n} segments per cycle streamed through a {slots}-slot ring (ATmega, 115200 baud)")
    ax = axes[1]
    ax.plot(deadline[slots:], slack * 1e3, ".", ms=3, color=REAL)
    ax.axhline(300, color=INK, lw=1, ls="--")
    ax.text(0.02, 300, "  required margin 300 ms", va="bottom", color=INK, transform=ax.get_yaxis_transform())
    ax.set_ylim(0, None)
    ax.set_ylabel("margin (ms)")
    ax.set_xlabel("time from the TTL edge (s)")
    ax.set_title(f"How long before its deadline each segment arrives (min {slack.min() * 1e3:.0f} ms): "
                 "a PC stall shorter than this is harmless")
    fig.savefig(os.path.join(OUT, "stream_margin.png"), dpi=110)
    plt.close(fig)


def ground_truth():
    traj = _example_traj()
    t, pos = pc.ground_truth(traj)
    fig, ax = plt.subplots(figsize=(8, 2.8), layout="constrained")
    for k in range(3):
        ax.step(np.concatenate([[k * traj.cycle_s], t + k * traj.cycle_s]),
                np.concatenate([[0], pos]), where="post", color=REAL if k == 0 else MUTED, lw=1.2)
        ax.axvline(k * traj.cycle_s, color=GRID, lw=1)
    ax.text(0.01, pos.max() * 0.95, "exported CSV:\none cycle", color=REAL, va="top")
    ax.text(traj.cycle_s * 1.02, pos.max() * 0.95, "cycle k = same rows shifted by k × cycle length", color=INK2,
            va="top")
    ax.set_xlabel("time_s (from the TTL rising edge)")
    ax.set_ylabel("position_deg")
    ax.set_title("Ground truth: position after every step")
    fig.savefig(os.path.join(OUT, "ground_truth.png"), dpi=110)
    plt.close(fig)


# Numbered callouts on the main window screenshot (pixel positions in gui_main.png,
# which is kept from the GUI test: PHANTOM_SCREENSHOT_DIR=... pytest tests/test_gui.py).
MAIN_CALLOUTS = [(510, 42), (1062, 70), (408, 132), (600, 172), (262, 678), (900, 190), (960, 381),
                 (1000, 450), (1010, 590), (805, 633), (600, 765)]


def annotate_main():
    from PIL import Image, ImageDraw, ImageFont
    img = Image.open(os.path.join(OUT, "gui_main.png")).convert("RGB")
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 13)
    except OSError:
        font = ImageFont.load_default()
    for k, (x, y) in enumerate(MAIN_CALLOUTS, 1):
        r = 11
        draw.ellipse((x - r, y - r, x + r, y + r), fill="#c2410c", outline="#ffffff", width=2)
        draw.text((x, y), str(k), fill="#ffffff", font=font, anchor="mm")
    img.save(os.path.join(OUT, "gui_main_annotated.png"))


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
    for fig in (shapes, step_placement, segments, planner_search, stream_margin, ground_truth, design_example,
                annotate_main):
        fig()
    ttl_timing()
    print("figures written to", OUT)
