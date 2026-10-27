"""Main window: connect to the phantom controller, load/upload a compiled
trajectory, confirm the microstep jumpers, arm/start/stop.

    python phantom_gui.py [--port /dev/ttyUSB0 | socket://127.0.0.1:5555]

Trajectories are made in the designer (button "Designer…", phantom_designer.py).
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from typing import Optional

import phantom_core as pc
import phantom_style as ps
from phantom_device import TRIGGERS, DeviceError, PhantomDevice

EMULATOR_URL = "socket://127.0.0.1:5555"

# The MRI TTL rising edge marks the start of the sequence and is time zero of
# the ground truth. The others only stand in for the scanner during development.
TRIGGER_LABELS = {
    "ttl": "MRI TTL (sequence start)",
    "button": "Button (debug: replaces the MRI)",
    "now": "Immediately (debug)",
}
assert set(TRIGGER_LABELS) == set(TRIGGERS)


def list_ports() -> list[str]:
    try:
        from serial.tools import list_ports as lp
        ports = [p.device for p in lp.comports()]
    except Exception:
        ports = []
    return ports + [EMULATOR_URL]


class PhantomApp:
    def __init__(self, root: tk.Tk, port: Optional[str] = None):
        self.root = root
        root.title("Dynamic MRI phantom controller")
        root.geometry("1120x800")
        root.minsize(980, 700)
        self.dev: Optional[PhantomDevice] = None
        self.traj: Optional[pc.CompiledTrajectory] = None
        self.traj_path: Optional[str] = None
        self.uploaded_checksum: Optional[int] = None  # program on the device
        self.running = False
        self.designer = None
        self.dev_state = "offline"
        self._ui_queue: "queue.Queue" = queue.Queue()
        self._build()
        if port:
            self.port_var.set(port)
        self._refresh()
        self.root.after(50, self._poll_queue)

    # -- layout -------------------------------------------------------------
    def _build(self) -> None:
        r = self.root
        ps.apply_theme(r)
        outer = ttk.Frame(r, padding=(10, 8))
        outer.pack(fill=tk.BOTH, expand=True)

        # Header: title, device line and the state badge.
        head = ttk.Frame(outer, style="Card.TFrame", padding=(14, 10))
        head.pack(fill=tk.X)
        titles = ttk.Frame(head, style="Card.TFrame")
        titles.pack(side=tk.LEFT)
        ttk.Label(titles, text="Dynamic MRI phantom", style="Title.TLabel").pack(anchor="w")
        self.dev_label = ttk.Label(titles, text="not connected", style="Subtitle.TLabel")
        self.dev_label.pack(anchor="w")
        self.state_badge = ps.Badge(head, "OFFLINE", "offline")
        self.state_badge.pack(side=tk.RIGHT, padx=(10, 0))
        conn = ttk.Frame(head, style="Card.TFrame")
        conn.pack(side=tk.RIGHT)
        ttk.Label(conn, text="Port", style="Muted.TLabel").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(conn, textvariable=self.port_var, values=list_ports(), width=28)
        self.port_box.pack(side=tk.LEFT, padx=6)
        if self.port_box["values"]:
            self.port_var.set(self.port_box["values"][0])
        ttk.Button(conn, text="Rescan", command=self._refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = ttk.Button(conn, text="Connect", style="Accent.TButton", command=self.toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(6, 0))

        body = ttk.Frame(outer)
        body.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        body.columnconfigure(0, weight=1)
        body.rowconfigure(1, weight=1)

        # Left: trajectory summary.
        tf = ps.card(body, "Trajectory")
        tf.grid(row=0, column=0, sticky="nsew")
        bar = ttk.Frame(tf, style="Card.TFrame")
        bar.pack(fill=tk.X)
        ttk.Button(bar, text="Open…", command=self.open_file).pack(side=tk.LEFT)
        self.save_btn = ttk.Button(bar, text="Save…", command=self.save_file)
        self.save_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(bar, text="Designer…", style="Accent.TButton", command=self.open_designer).pack(side=tk.LEFT)
        self.export_btn = ttk.Button(bar, text="Export ground truth…", command=self.export_ground_truth)
        self.export_btn.pack(side=tk.RIGHT)
        self.summary = ps.rich_text(tf, height=13, width=60)
        self.summary.pack(fill=tk.BOTH, expand=True, pady=(8, 4))
        self.compat_label = ttk.Label(tf, text="", wraplength=560, style="Card.TLabel")
        self.compat_label.pack(fill=tk.X)

        # Right: jumpers and run control.
        side = ttk.Frame(body)
        side.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        jf = ps.card(side, "Microstep jumpers", fill=tk.X)
        row = ttk.Frame(jf, style="Card.TFrame")
        row.pack(fill=tk.X)
        ttk.Label(row, text="Driver", style="Card.TLabel").pack(side=tk.LEFT)
        self.driver_var = tk.StringVar(value="DRV8825")
        ttk.Combobox(row, textvariable=self.driver_var, values=list(pc.DRIVERS), state="readonly",
                     width=10).pack(side=tk.LEFT, padx=6)
        self.driver_var.trace_add("write", lambda *_: self._driver_changed())
        self.jumper_canvas = tk.Canvas(jf, width=300, height=74, background=ps.SURFACE,
                                       highlightthickness=0)
        self.jumper_canvas.pack(fill=tk.X, pady=(8, 2))
        self.jumper_label = ttk.Label(jf, text="", wraplength=300, justify=tk.LEFT, style="Muted.TLabel")
        self.jumper_label.pack(fill=tk.X, pady=(0, 4))
        self.jumpers_ok = tk.BooleanVar(value=False)
        self.jumper_check = ttk.Checkbutton(jf, text="Jumpers are set as shown", variable=self.jumpers_ok,
                                            command=self._refresh)
        self.jumper_check.pack(anchor="w")

        rf = ps.card(side, "Run", fill=tk.X, pady=(8, 0))
        g = ttk.Frame(rf, style="Card.TFrame")
        g.pack(fill=tk.X)
        ttk.Label(g, text="Trigger", style="Card.TLabel").grid(row=0, column=0, sticky="w", pady=2)
        self.trigger_label = tk.StringVar(value=TRIGGER_LABELS["ttl"])
        ttk.Combobox(g, textvariable=self.trigger_label, values=list(TRIGGER_LABELS.values()),
                     state="readonly", width=30).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(g, text="Cycles", style="Card.TLabel").grid(row=1, column=0, sticky="w", pady=2)
        cyc = ttk.Frame(g, style="Card.TFrame")
        cyc.grid(row=1, column=1, sticky="w", padx=(8, 0))
        self.cycles_var = tk.StringVar(value="0")
        ttk.Spinbox(cyc, from_=0, to=1_000_000, textvariable=self.cycles_var, width=8).pack(side=tk.LEFT)
        ttk.Label(cyc, text="0 = until stopped", style="Muted.TLabel").pack(side=tk.LEFT, padx=6)
        self.trigger_warning = ttk.Label(g, text="", style="Warn.TLabel", wraplength=300)
        self.trigger_warning.grid(row=2, column=0, columnspan=2, sticky="w")
        self.trigger_label.trace_add("write", lambda *_: self._trigger_changed())

        b = ttk.Frame(rf, style="Card.TFrame")
        b.pack(fill=tk.X, pady=(8, 4))
        b.columnconfigure((0, 1), weight=1, uniform="run")
        self.upload_btn = ttk.Button(b, text="Upload", command=self.upload)
        self.upload_btn.grid(row=0, column=0, sticky="ew", padx=(0, 3))
        self.arm_btn = ttk.Button(b, text="Arm / Start", style="Accent.TButton", command=self.arm)
        self.arm_btn.grid(row=0, column=1, sticky="ew", padx=(3, 0))
        self.finish_btn = ttk.Button(b, text="Finish cycle", command=lambda: self._run("FINISH", lambda d: d.finish()))
        self.finish_btn.grid(row=1, column=0, sticky="ew", padx=(0, 3), pady=(6, 0))
        self.stop_btn = ttk.Button(b, text="STOP", style="Big.Danger.TButton", command=self.stop)
        self.stop_btn.grid(row=1, column=1, sticky="ew", padx=(3, 0), pady=(6, 0))
        self.progress = ttk.Progressbar(rf, mode="determinate")
        self.progress.pack(fill=tk.X, pady=(4, 2))
        s = ttk.Frame(rf, style="Card.TFrame")
        s.pack(fill=tk.X, pady=(2, 0))
        self.status_btn = ttk.Button(s, text="Status", command=self.status)
        self.status_btn.pack(side=tk.LEFT)
        self.state_label = ttk.Label(s, text="", style="Muted.TLabel", wraplength=230)
        self.state_label.pack(side=tk.LEFT, padx=8)

        mf = ps.card(side, "Manual move", fill=tk.X, pady=(8, 0))
        j = ttk.Frame(mf, style="Card.TFrame")
        j.pack(fill=tk.X)
        ttk.Label(j, text="Steps", style="Card.TLabel").pack(side=tk.LEFT)
        self.jog_steps = tk.StringVar(value="100")
        ttk.Entry(j, textvariable=self.jog_steps, width=7).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(j, text="Rate (Hz)", style="Card.TLabel").pack(side=tk.LEFT)
        self.jog_rate = tk.StringVar(value="1000")
        ttk.Entry(j, textvariable=self.jog_rate, width=6).pack(side=tk.LEFT, padx=4)
        self.zero_btn = ttk.Button(j, text="Set zero", command=lambda: self._run("ZERO", lambda d: d.zero()))
        self.zero_btn.pack(side=tk.RIGHT)
        self.jog_btn = ttk.Button(j, text="Jog", command=self.jog)
        self.jog_btn.pack(side=tk.RIGHT, padx=4)

        # Bottom: serial log.
        lf = ps.card(body, "Log")
        lf.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(8, 0))
        self.log_text = ScrolledText(lf, height=9, font="TkFixedFont", background=ps.CONSOLE_BG,
                                     foreground=ps.CONSOLE_FG, insertbackground=ps.CONSOLE_FG,
                                     relief="flat", borderwidth=0, highlightthickness=0, padx=6, pady=4)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        for tag, color in (("time", "#6b7280"), ("tx", "#93c5fd"), ("rx", "#86efac"), ("ev", "#fcd34d"),
                           ("err", "#fca5a5")):
            self.log_text.tag_configure(tag, foreground=color)

    def _draw_jumpers(self, driver: str, microstep: Optional[int]) -> None:
        """Mode pins as jumper headers: a filled block = jumper fitted (HIGH)."""
        c = self.jumper_canvas
        c.delete("all")
        info = pc.DRIVERS[driver]
        levels = info["table"].get(microstep) if microstep else None
        for i, pin in enumerate(info["pins"]):
            x = 40 + i * 90
            c.create_text(x + 14, 10, text=pin, fill=ps.MUTED, font=("TkDefaultFont", 9, "bold"))
            for dy in (0, 22):
                c.create_oval(x + 8, 22 + dy, x + 20, 34 + dy, outline="#9aa5b1", width=2)
            if levels is None:
                continue
            if levels[i]:
                c.create_rectangle(x + 2, 18, x + 26, 60, fill=ps.ACCENT, outline=ps.ACCENT_DARK, width=1)
                c.create_text(x + 14, 39, text="ON", fill="#ffffff", font=("TkDefaultFont", 8, "bold"))
            c.create_text(x + 14, 68, text="HIGH" if levels[i] else "open",
                          fill=ps.ACCENT if levels[i] else ps.MUTED, font=("TkDefaultFont", 8))

    # -- helpers ------------------------------------------------------------
    def log(self, text: str, tag: Optional[str] = None) -> None:
        if tag is None:
            low = text.lower()
            tag = "err" if ("failed" in low or "error" in low or text.startswith("< ERR")) else ""
        self.log_text.insert(tk.END, time.strftime("%H:%M:%S "), "time")
        self.log_text.insert(tk.END, text + "\n", tag)
        self.log_text.see(tk.END)

    def _post(self, fn, *args) -> None:
        """Run ``fn(*args)`` on the Tk thread."""
        self._ui_queue.put((fn, args))

    def _poll_queue(self) -> None:
        try:
            while True:
                fn, args = self._ui_queue.get_nowait()
                fn(*args)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _driver_changed(self) -> None:
        self.jumpers_ok.set(False)  # other driver, other pins: confirm again
        self._refresh()

    @property
    def trigger(self) -> str:
        """Protocol name of the selected trigger (ttl, button or now)."""
        return next(k for k, v in TRIGGER_LABELS.items() if v == self.trigger_label.get())

    def _trigger_changed(self) -> None:
        self.trigger_warning.configure(
            text="" if self.trigger == "ttl" else
            "Debug trigger: the motion is NOT synchronized to the MRI sequence.")

    def _refresh_ports(self) -> None:
        self.port_box["values"] = list_ports()

    def _caps(self) -> Optional[dict]:
        return self.dev.caps if self.dev else None

    def _set_state(self, state: str) -> None:
        """Device state shown in the header badge (offline/idle/loading/ready/armed/running)."""
        self.dev_state = state
        labels = {"armed": {"ttl": "ARMED · WAITING FOR TTL", "button": "ARMED · WAITING FOR BUTTON"}
                  .get(self.trigger, "ARMED")}
        self.state_badge.set(labels.get(state, state.upper()), state)

    def problems(self) -> list[str]:
        """Reasons why the current trajectory cannot be uploaded."""
        if self.traj is None:
            return ["no trajectory loaded"]
        p = [x for x in [pc.driver_problem(self.traj.microstep, self.driver_var.get())] if x]
        if self.dev is not None:
            p += pc.compatibility_problems(self.traj, self.dev.caps)
        return p

    def _refresh(self) -> None:
        connected = self.dev is not None
        t = self.traj
        probs = self.problems()
        if t is None:
            ps.set_rich(self.summary, [("No trajectory loaded.\n", "head"),
                                       ("Open a .phtraj file or create one in the designer.", "key")])
            self.jumper_label.configure(text="")
            self.compat_label.configure(text="")
            self._draw_jumpers(self.driver_var.get(), None)
        else:
            m = t.metrics
            name = os.path.basename(self.traj_path) if self.traj_path else "from the designer"
            per = ", ".join(f"{p.length_ms:.0f} ms / {p.amplitude_deg:.1f}°" for p in t.periods)
            parts = [(f"{t.design.shape.capitalize()} · {len(t.periods)} period(s)\n", "head")]
            ps.kv(parts, "File", name)
            ps.kv(parts, "Periods", f"{per}, skew {t.design.skew:+.2f}")
            ps.kv(parts, "Cycle", f"{t.cycle_s * 1000:.3f} ms · {m.get('steps_per_cycle', '?')} steps")
            ps.kv(parts, "Microstep", f"1/{t.microstep} · tick {t.tick_hz:,} Hz · {t.driver}")
            ps.kv(parts, "Segments", f"{len(t.segments)} ({m.get('memory_bytes', '?')} bytes) · "
                                     f"checksum {t.checksum}")
            ps.kv(parts, "Peak speed", f"{m.get('peak_speed_dps', 0):.1f} deg/s · "
                                       f"{m.get('peak_step_rate_hz', 0):.0f} steps/s")
            ps.kv(parts, "Max errors", f"timing {m.get('max_timing_error_us', 0):.2f} µs · "
                                       f"position {m.get('max_position_error_deg', 0):.4f} deg")
            ps.kv(parts, "Compiled for", t.device_caps.get("board", "?"))
            parts += [(f"Warning: {w}\n", "warn") for w in t.warnings]
            ps.set_rich(self.summary, parts)
            try:
                self.jumper_label.configure(text=pc.jumper_instructions(self.driver_var.get(), t.microstep))
                self._draw_jumpers(self.driver_var.get(), t.microstep)
            except ValueError as exc:
                self.jumper_label.configure(text=str(exc))
                self._draw_jumpers(self.driver_var.get(), None)
            if probs:
                self.compat_label.configure(text="NOT COMPATIBLE: " + "; ".join(probs), style="Error.TLabel")
            elif connected:
                self.compat_label.configure(text="Compatible with the connected device", style="Ok.TLabel")
            else:
                self.compat_label.configure(text="Connect a device to check compatibility",
                                            style="Muted.TLabel")

        can_upload = connected and not probs and self.jumpers_ok.get() and not self.running
        on_device = t is not None and self.uploaded_checksum == t.checksum
        self.upload_btn.configure(state=tk.NORMAL if can_upload else tk.DISABLED)
        self.arm_btn.configure(state=tk.NORMAL if (can_upload and on_device) else tk.DISABLED)
        for btn in (self.stop_btn, self.finish_btn, self.status_btn):
            btn.configure(state=tk.NORMAL if connected else tk.DISABLED)
        for btn in (self.jog_btn, self.zero_btn):
            btn.configure(state=tk.NORMAL if connected and not self.running else tk.DISABLED)
        for btn in (self.save_btn, self.export_btn):
            btn.configure(state=tk.NORMAL if t is not None else tk.DISABLED)
        self.connect_btn.configure(text="Disconnect" if connected else "Connect",
                                   style="TButton" if connected else "Accent.TButton")
        if not connected:
            self._set_state("offline")
        elif self.dev_state == "offline":
            self._set_state("idle")

    # -- trajectory ---------------------------------------------------------
    def set_trajectory(self, traj: pc.CompiledTrajectory, path: Optional[str] = None) -> None:
        self.traj = traj
        self.traj_path = path
        self.jumpers_ok.set(False)  # the jumpers must be confirmed for every new trajectory
        self.driver_var.set(traj.driver)
        self.log(f"trajectory loaded: 1/{traj.microstep} microstep, {len(traj.segments)} segments")
        self.log(pc.jumper_instructions(traj.driver, traj.microstep))
        self._refresh()

    def open_file(self) -> None:
        path = filedialog.askopenfilename(parent=self.root, filetypes=[
            ("Phantom trajectory", "*" + pc.FILE_EXTENSION), ("All files", "*")])
        if path:
            self.load_path(path)

    def load_path(self, path: str) -> None:
        try:
            traj = pc.CompiledTrajectory.load(path)
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            messagebox.showerror("Cannot open trajectory", str(exc), parent=self.root)
            return
        self.set_trajectory(traj, path)

    def save_file(self) -> None:
        if not self.traj:
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=pc.FILE_EXTENSION,
                                            filetypes=[("Phantom trajectory", "*" + pc.FILE_EXTENSION)])
        if path:
            self.traj.save(path)
            self.traj_path = path
            self.log(f"saved {path}")
            self._refresh()

    def export_ground_truth(self) -> None:
        """Position vs time since the MRI TTL edge, as played by the firmware."""
        if not self.traj:
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".csv",
                                            filetypes=[("CSV", "*.csv")])
        if path:
            caps = self.dev.caps if self.dev else self.traj.device_caps
            pc.export_ground_truth_csv(self.traj, path, caps.get("trigger_latency_us", 0.0))
            self.log(f"ground truth exported to {path}")

    def open_designer(self) -> None:
        from phantom_designer import DesignerWindow
        if self.designer is not None and self.designer.winfo_exists():
            self.designer.lift()
            return
        self.designer = DesignerWindow(self.root, on_send=self.set_trajectory, get_caps=self._caps,
                                       initial=self.traj)

    # -- device -------------------------------------------------------------
    def toggle_connect(self) -> None:
        if self.dev is not None:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            return
        self.connect_btn.configure(state=tk.DISABLED)
        self.log(f"connecting to {port} …")

        def work():
            dev = PhantomDevice(port, on_event=lambda ln: self._post(self._on_event, ln),
                                on_line=lambda d, ln: self._post(self._on_line, d, ln))
            try:
                dev.open()
                self._post(self._connected, dev, None)
            except Exception as exc:
                self._post(self._connected, None, exc)

        threading.Thread(target=work, daemon=True).start()

    def _connected(self, dev, exc) -> None:
        self.connect_btn.configure(state=tk.NORMAL)
        if exc is not None:
            self.log(f"connection failed: {exc}")
            messagebox.showerror("Connection failed", str(exc), parent=self.root)
            return
        self.dev = dev
        self.uploaded_checksum = None
        c = dev.caps
        self.dev_label.configure(
            text=f"{c['fw']} {c['version']} on {c['board']} · {c['max_segments']} segments · "
                 f"max {c['max_step_rate_hz']} steps/s · TTL {'yes' if c.get('ttl') else 'no'}")
        self._set_state("idle")
        self._refresh()

    def disconnect(self) -> None:
        if self.dev:
            self.dev.close()
        self.dev = None
        self.uploaded_checksum = None
        self.running = False
        self.dev_label.configure(text="not connected")
        self.state_label.configure(text="")
        self._refresh()

    def _on_line(self, direction: str, line: str) -> None:
        tag = "tx" if direction == ">" else ("err" if line.startswith("ERR") else
                                             ("ev" if line.startswith("EV") else "rx"))
        self.log(f"{direction} {line}", tag)

    def _on_event(self, line: str) -> None:
        if line.startswith("EV START"):
            self.running = True
            self.state_label.configure(text="running")
            self._set_state("running")
        elif line.startswith("EV DONE"):
            self.running = False
            self.state_label.configure(text="done: " + line[8:])
            self._set_state("ready")
        self._refresh()

    def _run(self, label: str, fn, done=None) -> None:
        """Run a blocking device call on a worker thread."""
        if self.dev is None:
            return
        dev = self.dev

        def work():
            try:
                result = fn(dev)
                self._post(lambda: done(result) if done else None)
            except (DeviceError, OSError, ValueError) as exc:
                self._post(self.log, f"{label} failed: {exc}", "err")
                if label == "upload":
                    self._post(self._set_state, "idle")
                self._post(self._refresh)

        threading.Thread(target=work, daemon=True).start()

    def status(self) -> None:
        def done(st):
            self.state_label.configure(text=f"cycle {st.get('cycle')} · pos {st.get('pos')} · "
                                            f"overruns {st.get('overruns')}")
            if st.get("state") in ps.STATE_COLORS:
                self._set_state(st["state"])

        self._run("STATUS", lambda d: d.status(), done)

    def _cycles(self) -> int:
        try:
            return max(0, int(self.cycles_var.get()))
        except ValueError:
            return 0

    def upload(self) -> None:
        probs = self.problems()
        if probs or not self.jumpers_ok.get():
            messagebox.showwarning("Cannot upload", "\n".join(probs) or "Confirm the jumper settings first.",
                                   parent=self.root)
            return
        traj, cycles, trigger = self.traj, self._cycles(), self.trigger
        self.progress.configure(maximum=len(traj.segments), value=0)
        self.uploaded_checksum = None  # LOAD invalidates the program on the device
        self.upload_btn.configure(state=tk.DISABLED)
        self._set_state("loading")
        self.log(f"uploading {len(traj.segments)} segments (trigger {trigger}, cycles {cycles}) …")

        def prog(i, n):
            self._post(lambda: self.progress.configure(value=i))

        def done(_):
            self.uploaded_checksum = traj.checksum
            self.log("upload complete, checksum verified by the device", "ev")
            self._set_state("ready")
            self._refresh()

        self._run("upload", lambda d: d.upload(traj, cycles, trigger, prog), done)

    def arm(self) -> None:
        if not self.jumpers_ok.get():
            messagebox.showwarning("Jumpers", "Confirm the jumper settings first.", parent=self.root)
            return
        trigger, cycles = self.trigger, self._cycles()
        def done(_):
            self.state_label.configure(text={"ttl": "waiting for the MRI TTL", "button": "waiting for the button",
                                             "now": "starting"}[trigger])
            if self.dev_state != "running":
                self._set_state("armed")

        self._run("ARM", lambda d: d.arm(trigger, cycles), done)

    def stop(self) -> None:
        def done(pos):
            self.running = False
            self.state_label.configure(text=f"stopped at {pos} steps")
            self._set_state("ready" if self.uploaded_checksum is not None else "idle")
            self._refresh()

        self._run("STOP", lambda d: d.stop(), done)

    def jog(self) -> None:
        try:
            steps, rate = int(self.jog_steps.get()), int(self.jog_rate.get())
        except ValueError:
            return
        self._run("JOG", lambda d: d.jog(steps, rate))

    def close(self) -> None:
        if self.dev:
            self.dev.close()
        self.root.destroy()


def main() -> None:
    ap = argparse.ArgumentParser(description="Dynamic MRI phantom controller")
    ap.add_argument("--port", help="serial port or pyserial URL (e.g. socket://127.0.0.1:5555)")
    ap.add_argument("trajectory", nargs="?", help="compiled trajectory (.phtraj) to open")
    args = ap.parse_args()
    root = tk.Tk()
    app = PhantomApp(root, port=args.port)
    if args.trajectory:
        app.load_path(args.trajectory)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    main()
