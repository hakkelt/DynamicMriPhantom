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
    assert "Microstep 1/" in text and "Set M0/M1/M2" in text
    assert len(win.ax_pos.lines) == 2
    img, ncolors = screenshot(win, os.path.join(shots, "designer.png"))
    assert img.size[0] > 600 and ncolors > 50

    win.send()
    pump(app.root)
    assert app.traj is res.trajectory
    assert "Microstep 1/" in app.summary.get("1.0", "end")
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
        assert str(app.compat_label["text"]).startswith("Compatible")
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
