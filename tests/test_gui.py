"""GUI smoke test (run under a display, e.g. ``xvfb-run -a python -m pytest tests/test_gui.py``).

Drives the designer and the main window like a user, against the emulated
board when the emulator is available, and checks screenshots are not blank.
Set PHANTOM_SCREENSHOT_DIR to keep the screenshots.
"""
import os
import time

import pytest

tk = pytest.importorskip("tkinter")
pytestmark = pytest.mark.gui

if not os.environ.get("DISPLAY"):
    pytest.skip("no display (use xvfb-run)", allow_module_level=True)

import emulator as emumod  # noqa: E402
import phantom_core as pc  # noqa: E402
import phantom_gui  # noqa: E402
from phantom_gui import TRIGGER_LABELS, PhantomApp  # noqa: E402


def pump(root, cond=lambda: False, timeout=0.3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        root.update()
        if cond():
            return True
        time.sleep(0.01)
    return cond()


def screenshot(widget, path):
    from PIL import ImageGrab
    widget.update()
    x, y = widget.winfo_rootx(), widget.winfo_rooty()
    w, h = widget.winfo_width(), widget.winfo_height()
    img = ImageGrab.grab(bbox=(x, y, x + w, y + h))
    img.save(path)
    colors = img.convert("RGB").getcolors(maxcolors=1 << 20)
    return img, len(colors)


@pytest.fixture
def shots(tmp_path):
    d = os.environ.get("PHANTOM_SCREENSHOT_DIR") or str(tmp_path)
    os.makedirs(d, exist_ok=True)
    return d


@pytest.fixture
def app():
    root = tk.Tk()
    a = PhantomApp(root)
    pump(root)
    yield a
    if a.dev:
        a.dev.close()
    root.destroy()


def test_designer_compile_and_send(app, shots, tmp_path):
    app.open_designer()
    win = app.designer
    pump(app.root)
    win.set_design(pc.Design(shape="gaussian", amplitude_deg=25, length_ms=800, skew=0.3, delay_ms=100,
                             n_periods=2, length_spread_pct=10), pc.Limits())
    win._vars["target"].set("ATmega328PB (default caps)")
    pump(app.root)
    assert len(win.table.get_children()) == 2
    res = win.compile_sync()
    assert res.trajectory is not None
    pump(app.root, timeout=1.0)
    text = win.result.get("1.0", "end")
    assert "Tried" in text
    assert "Microstep 1/" in text and "Set M0/M1/M2" in text
    assert len(win.ax_pos.lines) == 2
    img, ncolors = screenshot(win, os.path.join(shots, "designer.png"))
    assert img.size[0] > 600 and ncolors > 50

    win.send()
    pump(app.root)
    assert app.traj is res.trajectory
    assert "Microstep\t1/" in app.summary.get("1.0", "end")
    assert str(app.upload_btn["state"]) == "disabled"  # not connected, jumpers not confirmed
    path = str(tmp_path / "x.phtraj")
    app.traj.save(path)
    app.load_path(path)
    assert app.traj.checksum == res.trajectory.checksum
    _, ncolors = screenshot(app.root, os.path.join(shots, "main_offline.png"))
    assert ncolors > 20


def test_designer_driver_limits_microsteps(app):
    app.open_designer()
    win = app.designer
    win._vars["driver"].set("A4988")
    pump(app.root)
    assert not win._ms_vars[32].get()
    assert win.limits().allowed_microsteps == [1, 2, 4, 8, 16]


def test_upload_and_run_on_emulator(app, shots, hex_path):
    emu = emumod.AvrEmulator(hex_path).start()
    try:
        app.port_var.set(emu.url)
        app.toggle_connect()
        assert pump(app.root, lambda: app.dev is not None, timeout=15), "connect"
        traj = pc.compile_design(pc.Design(shape="sinusoid", amplitude_deg=10, length_ms=300),
                                 pc.Limits(), app.dev.caps)
        app.set_trajectory(traj)
        pump(app.root)
        assert "Compatible with" in str(app.compat_label["text"])
        assert str(app.upload_btn["state"]) == "disabled"  # jumpers not confirmed yet
        app.jumpers_ok.set(True)
        app._refresh()
        assert str(app.upload_btn["state"]) == "normal"
        app.driver_var.set("A4988")  # other pins: the confirmation must be given again
        assert not app.jumpers_ok.get()
        app.driver_var.set("DRV8825")
        app.jumpers_ok.set(True)
        app._refresh()
        assert app.trigger == "ttl"  # the MRI TTL is the default
        app.trigger_label.set(TRIGGER_LABELS["now"])
        assert app.trigger == "now" and "NOT synchronized" in str(app.trigger_warning["text"])
        app.cycles_var.set("2")
        app.upload()
        assert pump(app.root, lambda: app.uploaded_checksum is not None, timeout=30), "upload"
        assert str(app.arm_btn["state"]) == "normal"
        app.arm()
        assert pump(app.root, lambda: "done" in str(app.state_label["text"]), timeout=60), "run"
        assert "pos=0" in str(app.state_label["text"])
        log = app.log_text.get("1.0", "end")
        assert "EV START" in log and "upload complete" in log
        _, ncolors = screenshot(app.root, os.path.join(shots, "main_connected.png"))
        assert ncolors > 20
    finally:
        app.disconnect()
        emu.stop()


def test_streamed_run_and_abort_alarm(app, hex_path, monkeypatch):
    """A program larger than the ATmega's memory is streamed from the GUI; an
    underrun abort turns the badge red and raises an alarm."""
    alarms = []
    monkeypatch.setattr("phantom_gui.messagebox.showerror", lambda *a, **k: alarms.append(a))
    emu = emumod.AvrEmulator(hex_path, realtime=True).start()
    try:
        app.port_var.set(emu.url)
        app.toggle_connect()
        assert pump(app.root, lambda: app.dev is not None, timeout=15), "connect"
        design = pc.Design(shape="sinusoid", amplitude_deg=10, length_ms=300, n_periods=5,
                           length_spread_pct=10, amplitude_spread_pct=10, skew_jitter=0.1, delay_ms=50)
        traj = pc.compile_design(design, pc.Limits(), app.dev.caps)
        assert traj.metrics["streamed"]
        app.set_trajectory(traj)
        app.jumpers_ok.set(True)
        app.trigger_label.set(TRIGGER_LABELS["now"])
        app.cycles_var.set("1")
        app._refresh()
        assert "Streaming" in app.summary.get("1.0", "end")
        assert str(app.upload_btn["text"]) == "Fill stream buffer"
        app.upload()
        assert pump(app.root, lambda: app.uploaded_checksum is not None, timeout=30), "fill"
        app.arm()
        assert pump(app.root, lambda: "done" in str(app.state_label["text"]), timeout=60), "run"
        assert "pos=0" in str(app.state_label["text"])
        assert app.uploaded_checksum is None  # streamed programs are used up
        assert not alarms
        app._on_event("EV ABORT underrun consumed=30 pos=57 cycles=0")
        assert str(app.state_badge["text"]) == "ABORTED" and alarms
        assert "57 microsteps" in alarms[0][1]
    finally:
        app.disconnect()
        emu.stop()


def test_port_list_shows_only_usb_ports_by_name(app, monkeypatch):
    from serial.tools import list_ports
    from serial.tools.list_ports_common import ListPortInfo

    def port(device, vid=None, pid=None, product=None):
        p = ListPortInfo(device, skip_link_detection=True)
        p.vid, p.pid, p.product = vid, pid, product
        return p

    found = [port("/dev/ttyS0"), port("/dev/ttyS1"),  # built-in UARTs, mostly nothing behind them
             port("/dev/ttyUSB0", 0x1A86, 0x7523),
             port("/dev/ttyACM1", 0x303A, 0x1001),
             port("/dev/ttyACM0", 0x2341, 0x0043, "Arduino Uno"),
             port("/dev/ttyACM2", 0x1234, 0x5678, "Some Gadget")]
    monkeypatch.setattr(list_ports, "comports", lambda: found)
    assert phantom_gui.list_ports() == [
        ("Arduino Uno — /dev/ttyACM0", "/dev/ttyACM0"),
        ("ESP32-S3 (native USB) — /dev/ttyACM1", "/dev/ttyACM1"),
        ("Arduino clone (CH340) — /dev/ttyUSB0", "/dev/ttyUSB0"),
        ("Some Gadget — /dev/ttyACM2", "/dev/ttyACM2"),
        (f"Emulator — {phantom_gui.EMULATOR_URL}", phantom_gui.EMULATOR_URL),
    ]
    app.port_var.set("")
    app._refresh_ports()
    assert app.port_var.get() == "Arduino Uno — /dev/ttyACM0"
    assert app._selected_port() == "/dev/ttyACM0"
    # a typed port or URL is used as it is and survives a rescan
    app.port_var.set("socket://127.0.0.1:6000")
    app._refresh_ports()
    assert app._selected_port() == "socket://127.0.0.1:6000"
    # the selected board was unplugged: fall back to the first one listed
    app.port_var.set("Arduino Uno — /dev/ttyACM0")
    found.pop(4)
    app._refresh_ports()
    assert app.port_var.get() == "ESP32-S3 (native USB) — /dev/ttyACM1"


def _controls(widget):
    kinds = (tk.ttk.Button, tk.ttk.Entry, tk.ttk.Combobox, tk.ttk.Spinbox, tk.ttk.Checkbutton)
    for w in widget.winfo_children():
        if isinstance(w, kinds) and w.winfo_manager():  # shown (not e.g. a hidden button)
            yield w
        yield from _controls(w)


def test_every_control_has_a_tooltip(app):
    app.open_designer()
    pump(app.root, lambda: app.designer is not None)
    toolbar = app.designer.canvas.get_tk_widget().master  # matplotlib's own toolbar has its own
    missing = [f"{w.winfo_class()} {w.cget('text') if 'text' in w.keys() else ''}"
               for root in (app.root, app.designer) for w in _controls(root)
               if not hasattr(w, "tooltip") and not str(w).startswith(str(toolbar))]
    assert missing == []
    assert app.zero_btn.cget("text") == "Mark as zero"
    tip = app.zero_btn.tooltip
    tip.show()
    pump(app.root)
    assert tip._win is not None and "end stop" in tip.text
    tip._hide()
    assert tip._win is None


def test_designer_from_csv(app, tmp_path):
    import numpy as np
    t = np.linspace(0, 2.0, 101)
    x = 5 + 7 * np.sin(np.pi * t / 1.0) ** 2 * np.where(t < 1.0, 1.0, 0.5)
    x[-1] = x[0]
    path = tmp_path / "breath.csv"
    np.savetxt(path, np.column_stack([t * 1000, x]), delimiter=",", header="time_ms,position_deg", comments="")
    app.open_designer()
    pump(app.root, lambda: app.designer is not None)
    dz = app.designer
    dz._vars["target"].set("ESP32-S3 (default caps)")
    dz.load_csv(str(path))
    pump(app.root)
    assert dz._vars["shape"].get() == pc.CSV_SHAPE
    assert "breath.csv: 101 samples, cycle 2.000 s" in dz.csv_label.cget("text")
    assert dz._fields["amplitude_deg"].instate(["disabled"]) and dz._fields["seed"].instate(["disabled"])
    assert len(dz.table.get_children()) == 1  # the whole cycle is one period
    res = dz.compile_sync()
    assert res.trajectory is not None and res.trajectory.design.source == "breath.csv"
    dz.send()
    pump(app.root)
    assert app.traj is res.trajectory
    summary = app.summary.get("1.0", "end")
    assert "Ideal trajectory from CSV" in summary and "breath.csv · 101 samples" in summary
    # back to a shape: its fields come back, and a designer opened on the CSV trajectory shows it again
    dz._vars["shape"].set("sinusoid")
    pump(app.root)
    assert not dz._fields["amplitude_deg"].instate(["disabled"])
    assert dz.design().shape == "sinusoid"
    dz.set_design(res.trajectory.design, res.trajectory.limits)
    assert dz.design() is res.trajectory.design


def test_designer_rejects_bad_csv(app, tmp_path, monkeypatch):
    path = tmp_path / "open.csv"
    path.write_text("0,0\n1,2\n2,1\n")
    app.open_designer()
    pump(app.root, lambda: app.designer is not None)
    errors = []
    import phantom_designer
    monkeypatch.setattr(phantom_designer.messagebox,
                        "showerror", lambda title, msg, **kw: errors.append(msg))
    app.designer.load_csv(str(path))
    assert errors and "must end where it starts" in errors[0]
    assert app.designer._vars["shape"].get() == "sinusoid"
