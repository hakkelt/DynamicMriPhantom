"""Trajectory designer window (Tk + matplotlib).

Shape, period parameters, irregularity, driver/microsteps and limits in; a
compiled trajectory (largest microstep + finest tick the device can play) out,
with plots of the ideal vs the real (simulated firmware) motion.

Run on its own with ``python phantom_designer.py`` or open it from phantom_gui.py.
"""

from __future__ import annotations

import dataclasses
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Optional

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import numpy as np  # noqa: E402

import phantom_core as pc  # noqa: E402
import phantom_style as ps  # noqa: E402

TARGET_DEVICE = "Connected device"
TARGETS = {"ATmega328PB (default caps)": "atmega328pb", "ESP32-S3 (default caps)": "esp32s3"}
ALL_MICROSTEPS = sorted(set().union(*(d["table"] for d in pc.DRIVERS.values())))


def _plan_with_curves(design, limits, caps):
    res = pc.plan(design, limits, caps)
    return res, (pc.trajectory_curves(res.trajectory) if res.trajectory else None)


class DesignerWindow(tk.Toplevel):
    def __init__(self, master, on_send: Optional[Callable[[pc.CompiledTrajectory], None]] = None,
                 get_caps: Optional[Callable[[], Optional[dict]]] = None,
                 initial: Optional[pc.CompiledTrajectory] = None):
        super().__init__(master)
        self.title("Trajectory designer")
        self.geometry("1320x860")
        ps.apply_theme(self)
        self.on_send = on_send
        self.get_caps = get_caps or (lambda: None)
        self.traj: Optional[pc.CompiledTrajectory] = None
        self._busy = False
        self._vars: dict[str, tk.Variable] = {}
        self._ms_vars: dict[int, tk.BooleanVar] = {}
        self._build()
        if initial is not None:
            self.set_design(initial.design, initial.limits)
        self._update_periods()

    # -- layout -------------------------------------------------------------
    def _build(self) -> None:
        pw = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        pw.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        left = ttk.Frame(pw, padding=(0, 0, 6, 0))
        right = ttk.Frame(pw, style="Card.TFrame", padding=4)
        pw.add(left, weight=0)
        pw.add(right, weight=1)

        d, lim = pc.Design(), pc.Limits()
        shape = ps.card(left, "Shape", fill=tk.X)
        self._combo(shape, "shape", "Shape", pc.SHAPES, d.shape, 0)
        self._entry(shape, "amplitude_deg", "Amplitude (deg)", d.amplitude_deg, 1)
        self._entry(shape, "length_ms", "Period length (ms)", d.length_ms, 2)
        self._entry(shape, "skew", "Skewness (-0.95..0.95)", d.skew, 3)
        self._entry(shape, "delay_ms", "Delay between periods (ms)", d.delay_ms, 4)

        irr = ps.card(left, "Irregularity (K distinct periods, repeated)", fill=tk.X, pady=(6, 0))
        self._spin(irr, "n_periods", "Distinct periods K", d.n_periods, 1, 5, 0)
        self._entry(irr, "length_spread_pct", "Length spread (%)", d.length_spread_pct, 1)
        self._entry(irr, "amplitude_spread_pct", "Amplitude spread (%)", d.amplitude_spread_pct, 2)
        self._entry(irr, "delay_spread_pct", "Delay spread (%)", d.delay_spread_pct, 3)
        self._entry(irr, "skew_jitter", "Skew jitter (abs)", d.skew_jitter, 4)
        self._entry(irr, "seed", "Seed", d.seed, 5)
        ttk.Button(irr, text="New seed", command=self._new_seed).grid(row=5, column=2, padx=2)

        drv = ps.card(left, "Driver and limits", fill=tk.X, pady=(6, 0))
        self._combo(drv, "driver", "Driver", tuple(pc.DRIVERS), lim.driver, 0)
        ttk.Label(drv, text="Microsteps", style="Card.TLabel").grid(row=1, column=0, sticky="w", padx=(0, 6))
        msf = ttk.Frame(drv, style="Card.TFrame")
        msf.grid(row=1, column=1, columnspan=2, sticky="w", pady=2)
        self._ms_checks = {}
        for m in ALL_MICROSTEPS:
            v = tk.BooleanVar(value=True)
            self._ms_vars[m] = v
            cb = ttk.Checkbutton(msf, text=f"1/{m}", variable=v)
            cb.pack(side=tk.LEFT)
            self._ms_checks[m] = cb
        self._entry(drv, "max_speed_dps", "Max speed (deg/s)", lim.max_speed_dps, 2)
        self._entry(drv, "max_accel_dps2", "Max acceleration (deg/s²)", lim.max_accel_dps2, 3)
        self._entry(drv, "start_stop_speed_dps", "Start/stop speed (deg/s)", lim.start_stop_speed_dps, 4)
        self._entry(drv, "tolerance_us", "Timing tolerance (µs)", lim.tolerance_us, 5)
        targets = ((TARGET_DEVICE,) if self.get_caps() else ()) + tuple(TARGETS)
        self._combo(drv, "target", "Compile for", targets, targets[0], 6)
        self._vars["driver"].trace_add("write", lambda *_: self._driver_changed())
        self._driver_changed()

        btns = ttk.Frame(left)
        btns.pack(fill=tk.X, pady=8)
        self.compile_btn = ttk.Button(btns, text="Compile", style="Big.Accent.TButton", command=self.compile)
        self.compile_btn.pack(side=tk.LEFT)
        self.save_btn = ttk.Button(btns, text="Save…", command=self.save, state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT, padx=4)
        self.send_btn = ttk.Button(btns, text="Use in main window →", command=self.send, state=tk.DISABLED)
        self.send_btn.pack(side=tk.LEFT)
        if self.on_send is None:
            self.send_btn.pack_forget()

        self.tabs = ttk.Notebook(left)
        self.tabs.pack(fill=tk.BOTH, expand=True)
        res = ttk.Frame(self.tabs, style="Card.TFrame", padding=8)
        tab = ttk.Frame(self.tabs, style="Card.TFrame", padding=8)
        self.tabs.add(res, text="Result")
        self.tabs.add(tab, text="Periods")
        cols = ("k", "length_ms", "amplitude_deg", "skew", "delay_ms")
        heads = ("#", "Length (ms)", "Amplitude (°)", "Skew", "Delay (ms)")
        self.table = ttk.Treeview(tab, columns=cols, show="headings", height=5)
        for c, h, w in zip(cols, heads, (30, 90, 100, 60, 80)):
            self.table.heading(c, text=h)
            self.table.column(c, width=w, anchor=tk.E)
        self.table.pack(fill=tk.BOTH, expand=True)

        self.result = ps.rich_text(res, width=58, height=10)
        sb = ttk.Scrollbar(res, orient=tk.VERTICAL, command=self.result.yview)
        self.result.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.result.pack(fill=tk.BOTH, expand=True)
        ps.set_rich(self.result, [("Set the parameters and press Compile.", "key")])

        self.fig = Figure(figsize=(8, 7.5), dpi=90, layout="constrained")
        ps.style_figure(self.fig)
        self.ax_pos = self.fig.add_subplot(4, 1, 1)
        self.ax_err = self.fig.add_subplot(4, 1, 2, sharex=self.ax_pos)
        self.ax_spd = self.fig.add_subplot(4, 1, 3, sharex=self.ax_pos)
        self.ax_rate = self.fig.add_subplot(4, 1, 4, sharex=self.ax_pos)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        tb = NavigationToolbar2Tk(self.canvas, right)
        tb.configure(background=ps.SURFACE)
        for child in tb.winfo_children():
            try:
                child.configure(background=ps.SURFACE)
            except tk.TclError:
                pass
        tb.update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._clear_plots()

        for key in ("n_periods", "length_spread_pct", "amplitude_spread_pct", "delay_spread_pct",
                    "skew_jitter", "seed", "amplitude_deg", "length_ms", "skew", "delay_ms"):
            self._vars[key].trace_add("write", lambda *_: self._update_periods())

    def _label(self, parent, text, row):
        ttk.Label(parent, text=text, style="Card.TLabel").grid(row=row, column=0, sticky="w", padx=(0, 6), pady=2)

    def _entry(self, parent, key, text, value, row):
        self._label(parent, text, row)
        v = tk.StringVar(value=str(value))
        self._vars[key] = v
        ttk.Entry(parent, textvariable=v, width=12).grid(row=row, column=1, sticky="w")

    def _spin(self, parent, key, text, value, lo, hi, row):
        self._label(parent, text, row)
        v = tk.StringVar(value=str(value))
        self._vars[key] = v
        ttk.Spinbox(parent, from_=lo, to=hi, textvariable=v, width=10).grid(row=row, column=1, sticky="w")

    def _combo(self, parent, key, text, values, value, row):
        self._label(parent, text, row)
        v = tk.StringVar(value=value)
        self._vars[key] = v
        ttk.Combobox(parent, textvariable=v, values=list(values), state="readonly",
                     width=26 if key == "target" else 12).grid(row=row, column=1, columnspan=2, sticky="w")

    # -- parameters ---------------------------------------------------------
    def _driver_changed(self) -> None:
        ok = set(pc.driver_microsteps(self._vars["driver"].get()))
        for m, cb in self._ms_checks.items():
            cb.state(["!disabled"] if m in ok else ["disabled"])
            if m not in ok:
                self._ms_vars[m].set(False)
            elif not any(self._ms_vars[x].get() for x in ok):
                self._ms_vars[m].set(True)

    def _new_seed(self) -> None:
        import random
        self._vars["seed"].set(str(random.randint(1, 999_999)))

    def _num(self, key, cast=float):
        text = self._vars[key].get().strip()
        try:
            return cast(float(text))
        except ValueError:
            raise ValueError(f"{key}: '{text}' is not a number") from None

    def design(self) -> pc.Design:
        return pc.Design(
            shape=self._vars["shape"].get(),
            amplitude_deg=self._num("amplitude_deg"),
            length_ms=self._num("length_ms"),
            skew=self._num("skew"),
            delay_ms=self._num("delay_ms"),
            n_periods=self._num("n_periods", int),
            length_spread_pct=self._num("length_spread_pct"),
            amplitude_spread_pct=self._num("amplitude_spread_pct"),
            delay_spread_pct=self._num("delay_spread_pct"),
            skew_jitter=self._num("skew_jitter"),
            seed=self._num("seed", int),
        )

    def limits(self) -> pc.Limits:
        allowed = [m for m, v in self._ms_vars.items() if v.get()]
        if not allowed:
            raise ValueError("select at least one microstep")
        return pc.Limits(
            driver=self._vars["driver"].get(),
            allowed_microsteps=allowed,
            max_speed_dps=self._num("max_speed_dps"),
            max_accel_dps2=self._num("max_accel_dps2"),
            start_stop_speed_dps=self._num("start_stop_speed_dps"),
            tolerance_us=self._num("tolerance_us"),
        )

    def set_design(self, design: pc.Design, limits: pc.Limits) -> None:
        for k, v in dataclasses.asdict(design).items():
            self._vars[k].set(str(v))
        for k in ("driver", "max_speed_dps", "max_accel_dps2", "start_stop_speed_dps", "tolerance_us"):
            self._vars[k].set(str(getattr(limits, k)))
        allowed = limits.allowed_microsteps or pc.driver_microsteps(limits.driver)
        for m, v in self._ms_vars.items():
            v.set(m in allowed)

    def caps(self) -> dict:
        target = self._vars["target"].get()
        if target == TARGET_DEVICE:
            caps = self.get_caps()
            if caps:
                return caps
        return pc.DEFAULT_CAPS[TARGETS.get(target, "atmega328pb")]

    def _update_periods(self) -> None:
        try:
            periods = pc.generate_periods(self.design())
        except (ValueError, TypeError):
            return
        self.table.delete(*self.table.get_children())
        for k, p in enumerate(periods):
            self.table.insert("", tk.END, values=(k + 1, f"{p.length_ms:.1f}", f"{p.amplitude_deg:.2f}",
                                                  f"{p.skew:+.3f}", f"{p.delay_ms:.1f}"))

    # -- compile --------------------------------------------------------------
    def compile(self) -> None:
        if self._busy:
            return
        try:
            design, limits, caps = self.design(), self.limits(), self.caps()
            pc.generate_periods(design)
        except ValueError as exc:
            messagebox.showerror("Invalid parameters", str(exc), parent=self)
            return
        self._busy = True
        self.compile_btn.configure(state=tk.DISABLED, text="Compiling…")
        self.configure(cursor="watch")

        # Tk is not thread-safe: the worker only fills a queue, the Tk thread polls it.
        done: "queue.Queue[tuple]" = queue.Queue()

        def work():
            try:
                done.put((*_plan_with_curves(design, limits, caps), None))
            except Exception as exc:  # pragma: no cover - reported in the GUI
                done.put((None, None, exc))

        def poll():
            if not self.winfo_exists():
                return
            try:
                self._compiled(*done.get_nowait())
            except queue.Empty:
                self.after(50, poll)

        threading.Thread(target=work, daemon=True).start()
        self.after(50, poll)

    def compile_sync(self) -> pc.PlanResult:
        """Compile on the calling thread (used by tests)."""
        res, curves = _plan_with_curves(self.design(), self.limits(), self.caps())
        self._compiled(res, curves, None)
        return res

    def _compiled(self, res: Optional[pc.PlanResult], curves, exc) -> None:
        self._busy = False
        self.compile_btn.configure(state=tk.NORMAL, text="Compile")
        self.configure(cursor="")
        if exc is not None:
            messagebox.showerror("Compile failed", str(exc), parent=self)
            return
        self.traj = res.trajectory
        state = tk.NORMAL if self.traj else tk.DISABLED
        self.save_btn.configure(state=state)
        self.send_btn.configure(state=state)
        self._show_result(res)
        if curves is not None:
            self._plot(curves)
        else:
            self._clear_plots()

    def _show_result(self, res: pc.PlanResult) -> None:
        t = res.trajectory
        if t is None:
            parts = [("Does not fit the device\n", "head"), (f"{res.error}\n", "error")]
        else:
            m = t.metrics
            cap = t.device_caps
            parts = [(f"Microstep 1/{t.microstep} · tick {t.tick_hz:,} Hz ({1e6 / t.tick_hz:.4g} µs)\n", "head"),
                     (pc.jumper_instructions(t.driver, t.microstep) + "\n", "key")]
            ps.kv(parts, "Segments", f"{m['n_segments']} / {cap['max_segments']}  ({m['memory_bytes']} bytes)")
            ps.kv(parts, "Cycle", f"{m['cycle_s'] * 1000:.3f} ms, {m['steps_per_cycle']} steps")
            ps.kv(parts, "Peak speed", f"{m['peak_speed_dps']:.1f} deg/s")
            ps.kv(parts, "Peak accel", f"{m['peak_accel_dps2']:.0f} deg/s²")
            ps.kv(parts, "Peak step rate", f"{m['peak_step_rate_hz']:.0f} Hz (device max {cap['max_step_rate_hz']})")
            ps.kv(parts, "Start/stop speed", f"{m['max_start_stop_speed_dps']:.1f} deg/s")
            ps.kv(parts, "Timing error", f"≤ {m['max_timing_error_us']:.2f} µs")
            ps.kv(parts, "Position error", f"≤ {m['max_position_error_deg']:.4f} deg")
            parts += [(f"Warning: {w}\n", "warn") for w in t.warnings]
        parts.append(("Tried (largest microstep, finest tick first)\n", "head"))
        for a in res.attempts:
            tick = f"{a['tick_hz']:,} Hz" if a["tick_hz"] else "-"
            parts.append((f"1/{a['microstep']:<3}{tick:>14}  {a['result']}\n", "mono"))
        ps.set_rich(self.result, parts)

    def _clear_plots(self) -> None:
        for ax in (self.ax_pos, self.ax_err, self.ax_spd, self.ax_rate):
            ax.clear()
            ps.style_axes(ax)
        for ax in (self.ax_pos, self.ax_err, self.ax_spd):
            ax.tick_params(labelbottom=False)
        self.ax_pos.set_ylabel("position (deg)")
        self.ax_err.set_ylabel("real − ideal (deg)")
        self.ax_spd.set_ylabel("speed (deg/s)")
        self.ax_rate.set_ylabel("step rate (Hz)")
        self.ax_rate.set_xlabel("time from the TTL edge (s)")
        self.canvas.draw_idle()

    def _period_bands(self, t_end: float) -> None:
        """Alternating background bands, one per period (motion + delay), numbered on top."""
        if self.traj is None:
            return
        starts = pc.period_starts(self.traj.periods)
        k = 0
        while starts[0] + k * starts[-1] < t_end:
            off = k * starts[-1]
            for i in range(len(starts) - 1):
                a, b = starts[i] + off, starts[i + 1] + off
                if a >= t_end:
                    break
                for ax in (self.ax_pos, self.ax_err, self.ax_spd, self.ax_rate):
                    ax.axvspan(a, b, color=ps.BAND_COLORS[i % 2], lw=0, zorder=0)
                self.ax_pos.annotate(f"P{i + 1}", ((a + b) / 2, 1.0), xycoords=("data", "axes fraction"),
                                     ha="center", va="bottom", fontsize=8, color=ps.MUTED)
            k += 1

    def _plot(self, c) -> None:
        self._clear_plots()
        t = c["t"]
        self._period_bands(t[-1] if len(t) else 0)
        self.ax_pos.plot(t, c["ideal_deg"], color=ps.IDEAL_COLOR, lw=1.6, label="ideal")
        self.ax_pos.step(t, c["real_deg"], where="post", color=ps.REAL_COLOR, lw=0.9, label="real (steps)")
        self.ax_pos.legend(loc="upper right", fontsize=8, frameon=False)
        # The error is a fast sawtooth: draw its envelope as a band.
        err = c["error_deg"]
        if len(t) > 1:
            edges = np.linspace(t[0], t[-1], 600)
            idx = np.clip(np.searchsorted(edges, t) - 1, 0, len(edges) - 2)
            lo = np.full(len(edges) - 1, np.nan)
            hi = np.full(len(edges) - 1, np.nan)
            np.fmin.at(lo, idx, err)
            np.fmax.at(hi, idx, err)
            mid = (edges[:-1] + edges[1:]) / 2
            self.ax_err.fill_between(mid, lo, hi, color=ps.REAL_COLOR, alpha=0.35, lw=0, label="envelope")
            if self.traj is not None:
                half = pc.deg_per_step(self.traj.microstep)
                for y in (-half, half):
                    self.ax_err.axhline(y, color=ps.LIMIT_COLOR, lw=0.8, ls="--")
                self.ax_err.annotate("± 1 microstep", (1.0, half), xycoords=("axes fraction", "data"),
                                     ha="right", va="bottom", fontsize=7, color=ps.MUTED)
        self.ax_spd.plot(t, c["speed_dps"], color=ps.IDEAL_COLOR, lw=1.3)
        if self.traj is not None:
            vmax = self.traj.limits.max_speed_dps
            for y in (-vmax, vmax):
                self.ax_spd.axhline(y, color=ps.LIMIT_COLOR, lw=0.8, ls="--")
            peak = np.abs(c["speed_dps"]).max() if len(t) else 0
            self.ax_spd.set_ylim(-max(peak, 1) * 1.25, max(peak, 1) * 1.25)
            if vmax <= peak * 1.25:
                self.ax_spd.annotate("speed limit", (1.0, vmax), xycoords=("axes fraction", "data"),
                                     ha="right", va="bottom", fontsize=7, color=ps.MUTED)
        self.ax_rate.plot(c["rate_t"], c["step_rate_hz"], ".", ms=2, color=ps.REAL_COLOR)
        self.canvas.draw_idle()

    # -- output -------------------------------------------------------------
    def save(self) -> None:
        if not self.traj:
            return
        path = filedialog.asksaveasfilename(parent=self, defaultextension=pc.FILE_EXTENSION,
                                            filetypes=[("Phantom trajectory", "*" + pc.FILE_EXTENSION)])
        if path:
            self.traj.save(path)

    def send(self) -> None:
        if self.traj and self.on_send:
            self.on_send(self.traj)


def main() -> None:
    root = tk.Tk()
    root.withdraw()
    win = DesignerWindow(root)
    win.protocol("WM_DELETE_WINDOW", root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
