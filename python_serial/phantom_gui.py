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
        root.geometry("1000x760")
        self.dev: Optional[PhantomDevice] = None
        self.traj: Optional[pc.CompiledTrajectory] = None
        self.traj_path: Optional[str] = None
        self.uploaded_checksum: Optional[int] = None  # program on the device
        self.running = False
        self.designer = None
        self._ui_queue: "queue.Queue" = queue.Queue()
        self._build()
        if port:
            self.port_var.set(port)
        self._refresh()
        self.root.after(50, self._poll_queue)

    # -- layout -------------------------------------------------------------
    def _build(self) -> None:
        r = self.root
        top = ttk.LabelFrame(r, text="Device", padding=6)
        top.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, values=list_ports(), width=34)
        self.port_box.pack(side=tk.LEFT, padx=4)
        if self.port_box["values"]:
            self.port_var.set(self.port_box["values"][0])
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=4)
        self.dev_label = ttk.Label(top, text="not connected", foreground="#888")
        self.dev_label.pack(side=tk.LEFT, padx=6)

        mid = ttk.Frame(r)
        mid.pack(fill=tk.X, padx=6)
        tf = ttk.LabelFrame(mid, text="Trajectory", padding=6)
        tf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        bar = ttk.Frame(tf)
        bar.pack(fill=tk.X)
        ttk.Button(bar, text="Open…", command=self.open_file).pack(side=tk.LEFT)
        self.save_btn = ttk.Button(bar, text="Save…", command=self.save_file)
        self.save_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(bar, text="Designer…", command=self.open_designer).pack(side=tk.LEFT)
        self.export_btn = ttk.Button(bar, text="Export ground truth…", command=self.export_ground_truth)
        self.export_btn.pack(side=tk.LEFT, padx=4)
        self.summary = tk.Text(tf, height=12, width=64, wrap=tk.WORD, font=("TkFixedFont", 9))
        self.summary.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        self.compat_label = ttk.Label(tf, text="", wraplength=520)
        self.compat_label.pack(fill=tk.X)

        side = ttk.Frame(mid)
        side.pack(side=tk.LEFT, fill=tk.Y, padx=(6, 0))
        jf = ttk.LabelFrame(side, text="Microstep jumpers", padding=6)
        jf.pack(fill=tk.X)
        row = ttk.Frame(jf)
        row.pack(fill=tk.X)
        ttk.Label(row, text="Driver").pack(side=tk.LEFT)
        self.driver_var = tk.StringVar(value="DRV8825")
        ttk.Combobox(row, textvariable=self.driver_var, values=list(pc.DRIVERS), state="readonly",
                     width=10).pack(side=tk.LEFT, padx=4)
        self.driver_var.trace_add("write", lambda *_: self._driver_changed())
        self.jumper_label = ttk.Label(jf, text="", wraplength=300, justify=tk.LEFT)
        self.jumper_label.pack(fill=tk.X, pady=4)
        self.jumpers_ok = tk.BooleanVar(value=False)
        self.jumper_check = ttk.Checkbutton(jf, text="Jumpers are set as shown", variable=self.jumpers_ok,
                                            command=self._refresh)
        self.jumper_check.pack(anchor="w")

        rf = ttk.LabelFrame(side, text="Run", padding=6)
        rf.pack(fill=tk.X, pady=(6, 0))
        g = ttk.Frame(rf)
        g.pack(fill=tk.X)
        ttk.Label(g, text="Trigger").grid(row=0, column=0, sticky="w")
        self.trigger_label = tk.StringVar(value=TRIGGER_LABELS["ttl"])
        ttk.Combobox(g, textvariable=self.trigger_label, values=list(TRIGGER_LABELS.values()),
                     state="readonly", width=30).grid(row=0, column=1, sticky="w")
        self.trigger_warning = ttk.Label(g, text="", foreground="#b06000", wraplength=300)
        self.trigger_warning.grid(row=2, column=0, columnspan=2, sticky="w")
        self.trigger_label.trace_add("write", lambda *_: self._trigger_changed())
        ttk.Label(g, text="Cycles (0 = forever)").grid(row=1, column=0, sticky="w")
        self.cycles_var = tk.StringVar(value="0")
        ttk.Spinbox(g, from_=0, to=1_000_000, textvariable=self.cycles_var, width=8).grid(
            row=1, column=1, sticky="w")
        b = ttk.Frame(rf)
        b.pack(fill=tk.X, pady=4)
        self.upload_btn = ttk.Button(b, text="Upload", command=self.upload)
        self.upload_btn.pack(side=tk.LEFT)
        self.arm_btn = ttk.Button(b, text="Arm / Start", command=self.arm)
        self.arm_btn.pack(side=tk.LEFT, padx=4)
        self.finish_btn = ttk.Button(b, text="Finish", command=lambda: self._run("FINISH", lambda d: d.finish()))
        self.finish_btn.pack(side=tk.LEFT)
        self.stop_btn = ttk.Button(b, text="STOP", command=self.stop)
        self.stop_btn.pack(side=tk.LEFT, padx=4)
        self.progress = ttk.Progressbar(rf, mode="determinate")
        self.progress.pack(fill=tk.X)
        s = ttk.Frame(rf)
        s.pack(fill=tk.X, pady=(4, 0))
        self.status_btn = ttk.Button(s, text="Status", command=self.status)
        self.status_btn.pack(side=tk.LEFT)
        self.state_label = ttk.Label(s, text="")
        self.state_label.pack(side=tk.LEFT, padx=6)
        j = ttk.Frame(rf)
        j.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(j, text="Jog steps").pack(side=tk.LEFT)
        self.jog_steps = tk.StringVar(value="100")
        ttk.Entry(j, textvariable=self.jog_steps, width=7).pack(side=tk.LEFT)
        ttk.Label(j, text="Hz").pack(side=tk.LEFT, padx=(4, 0))
        self.jog_rate = tk.StringVar(value="1000")
        ttk.Entry(j, textvariable=self.jog_rate, width=6).pack(side=tk.LEFT)
        self.jog_btn = ttk.Button(j, text="Jog", command=self.jog)
        self.jog_btn.pack(side=tk.LEFT, padx=4)
        self.zero_btn = ttk.Button(j, text="Zero", command=lambda: self._run("ZERO", lambda d: d.zero()))
        self.zero_btn.pack(side=tk.LEFT)

        lf = ttk.LabelFrame(r, text="Log", padding=4)
        lf.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        self.log_text = ScrolledText(lf, height=10, font=("TkFixedFont", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # -- helpers ------------------------------------------------------------
    def log(self, text: str) -> None:
        self.log_text.insert(tk.END, time.strftime("%H:%M:%S ") + text + "\n")
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

    def _set_summary(self, text: str) -> None:
        self.summary.configure(state=tk.NORMAL)
        self.summary.delete("1.0", tk.END)
        self.summary.insert("1.0", text)
        self.summary.configure(state=tk.DISABLED)

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
            self._set_summary("No trajectory. Open a .phtraj file or use the designer.")
            self.jumper_label.configure(text="")
            self.compat_label.configure(text="")
        else:
            m = t.metrics
            name = os.path.basename(self.traj_path) if self.traj_path else "(from designer)"
            per = ", ".join(f"{p.length_ms:.0f} ms/{p.amplitude_deg:.1f}°" for p in t.periods)
            lines = [
                f"File      {name}",
                f"Shape     {t.design.shape}, skew {t.design.skew:+.2f}, {len(t.periods)} period(s): {per}",
                f"Cycle     {t.cycle_s * 1000:.3f} ms, {m.get('steps_per_cycle', '?')} steps",
                f"Microstep 1/{t.microstep}   tick {t.tick_hz:,} Hz   driver {t.driver}",
                f"Segments  {len(t.segments)} ({m.get('memory_bytes', '?')} bytes), "
                f"checksum {t.checksum}",
                f"Speed     peak {m.get('peak_speed_dps', 0):.1f} deg/s, "
                f"step rate {m.get('peak_step_rate_hz', 0):.0f} Hz",
                f"Errors    timing <= {m.get('max_timing_error_us', 0):.2f} µs, "
                f"position <= {m.get('max_position_error_deg', 0):.4f} deg",
                f"Compiled for {t.device_caps.get('board', '?')}",
            ]
            lines += [f"WARNING: {w}" for w in t.warnings]
            self._set_summary("\n".join(lines))
            try:
                self.jumper_label.configure(text=pc.jumper_instructions(self.driver_var.get(), t.microstep))
            except ValueError as exc:
                self.jumper_label.configure(text=str(exc))
            if probs:
                self.compat_label.configure(text="NOT COMPATIBLE: " + "; ".join(probs), foreground="#b00020")
            elif connected:
                self.compat_label.configure(text="Compatible with the connected device", foreground="#1b7f1b")
            else:
                self.compat_label.configure(text="connect a device to check compatibility", foreground="#888")

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
        self.connect_btn.configure(text="Disconnect" if connected else "Connect")

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
                 f"max {c['max_step_rate_hz']} steps/s · TTL {'yes' if c.get('ttl') else 'no'}",
            foreground="#1b7f1b")
        self._refresh()

    def disconnect(self) -> None:
        if self.dev:
            self.dev.close()
        self.dev = None
        self.uploaded_checksum = None
        self.running = False
        self.dev_label.configure(text="not connected", foreground="#888")
        self.state_label.configure(text="")
        self._refresh()

    def _on_line(self, direction: str, line: str) -> None:
        self.log(f"{direction} {line}")

    def _on_event(self, line: str) -> None:
        if line.startswith("EV START"):
            self.running = True
            self.state_label.configure(text="running")
        elif line.startswith("EV DONE"):
            self.running = False
            self.state_label.configure(text="done: " + line[8:])
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
                self._post(self.log, f"{label} failed: {exc}")
                self._post(self._refresh)

        threading.Thread(target=work, daemon=True).start()

    def status(self) -> None:
        self._run("STATUS", lambda d: d.status(), lambda st: self.state_label.configure(
            text=f"{st.get('state')} · cycle {st.get('cycle')} · pos {st.get('pos')} · "
                 f"overruns {st.get('overruns')}"))

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
        self.log(f"uploading {len(traj.segments)} segments (trigger {trigger}, cycles {cycles}) …")

        def prog(i, n):
            self._post(lambda: self.progress.configure(value=i))

        def done(_):
            self.uploaded_checksum = traj.checksum
            self.log("upload complete, checksum verified by the device")
            self._refresh()

        self._run("upload", lambda d: d.upload(traj, cycles, trigger, prog), done)

    def arm(self) -> None:
        if not self.jumpers_ok.get():
            messagebox.showwarning("Jumpers", "Confirm the jumper settings first.", parent=self.root)
            return
        trigger, cycles = self.trigger, self._cycles()
        self._run("ARM", lambda d: d.arm(trigger, cycles),
                  lambda line: self.state_label.configure(
                      text={"ttl": "armed, waiting for the MRI TTL", "button": "armed, waiting for the button",
                            "now": "starting"}[trigger]))

    def stop(self) -> None:
        def done(pos):
            self.running = False
            self.state_label.configure(text=f"stopped at {pos} steps")
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
