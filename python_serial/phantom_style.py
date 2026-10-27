"""Shared look of the phantom windows: a light ttk theme and matching plot style.

    apply_theme(root)        # once, on the Tk root
    style_axes(ax)           # for every matplotlib axes

Built on the cross-platform ttk "clam" theme so the windows look the same on
Windows, macOS and Linux.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

BG = "#eef1f5"          # window background
SURFACE = "#ffffff"     # cards
BORDER = "#d5dbe3"
BUTTON = "#e8ecf2"      # secondary buttons
TEXT = "#1f2933"
MUTED = "#677585"
ACCENT = "#2563eb"
ACCENT_DARK = "#1d4ed8"
ACCENT_LIGHT = "#dbe7fd"
DANGER = "#dc2626"
DANGER_DARK = "#b91c1c"
OK = "#15803d"
OK_LIGHT = "#dcfce7"
WARN = "#a15c07"
WARN_LIGHT = "#fef3c7"
ERROR_LIGHT = "#fee2e2"
CONSOLE_BG = "#111827"
CONSOLE_FG = "#d1d5db"

# Plot colors (ideal vs real motion, limits)
IDEAL_COLOR = "#2563eb"
REAL_COLOR = "#ea580c"
LIMIT_COLOR = "#9ca3af"
BAND_COLORS = ("#f8fafc", "#eef2f7")

# Badge colors (foreground, background) for the device state.
STATE_COLORS = {
    "offline": (MUTED, "#e5e7eb"),
    "idle": (MUTED, "#e5e7eb"),
    "loading": (ACCENT_DARK, ACCENT_LIGHT),
    "ready": (OK, OK_LIGHT),
    "armed": (WARN, WARN_LIGHT),
    "running": ("#ffffff", OK),
    "error": (DANGER_DARK, ERROR_LIGHT),
}


def apply_theme(root: tk.Misc) -> ttk.Style:
    """Configure the ttk styles used by the phantom windows."""
    style = ttk.Style(root)
    style.theme_use("clam")
    base = tkfont.nametofont("TkDefaultFont")
    base.configure(size=10)
    for name in ("TkTextFont", "TkMenuFont"):
        tkfont.nametofont(name).configure(size=10)
    tkfont.nametofont("TkFixedFont").configure(size=9)
    family = base.actual("family")
    bold = (family, 10, "bold")
    root.configure(background=BG)
    root.option_add("*TCombobox*Listbox.background", SURFACE)
    root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)

    style.configure(".", background=BG, foreground=TEXT, bordercolor=BORDER, lightcolor=SURFACE,
                    darkcolor=BORDER, troughcolor=BG, focuscolor=ACCENT, selectbackground=ACCENT,
                    selectforeground="#ffffff", fieldbackground=SURFACE, insertcolor=TEXT)
    style.configure("TFrame", background=BG)
    style.configure("Card.TFrame", background=SURFACE)
    style.configure("TLabel", background=BG, foreground=TEXT)
    style.configure("Card.TLabel", background=SURFACE)
    style.configure("Muted.TLabel", background=SURFACE, foreground=MUTED)
    style.configure("Title.TLabel", background=SURFACE, foreground=TEXT, font=(family, 15, "bold"))
    style.configure("Subtitle.TLabel", background=SURFACE, foreground=MUTED)
    style.configure("Warn.TLabel", background=SURFACE, foreground=WARN)
    style.configure("Ok.TLabel", background=SURFACE, foreground=OK)
    style.configure("Error.TLabel", background=SURFACE, foreground=DANGER)

    style.configure("TLabelframe", background=SURFACE, bordercolor=BORDER, lightcolor=SURFACE,
                    darkcolor=SURFACE, relief="solid", borderwidth=1, padding=10)
    style.configure("TLabelframe.Label", background=BG, foreground=MUTED, font=bold)

    style.configure("TButton", background=BUTTON, foreground=TEXT, bordercolor="#c3ccd8",
                    lightcolor=BUTTON, darkcolor=BUTTON, padding=(12, 5), relief="solid")
    style.map("TButton",
              background=[("disabled", BG), ("pressed", ACCENT_LIGHT), ("active", "#dfe5ee")],
              lightcolor=[("disabled", BG), ("active", "#dfe5ee")],
              darkcolor=[("disabled", BG), ("active", "#dfe5ee")],
              foreground=[("disabled", "#a0a8b3")],
              bordercolor=[("focus", ACCENT), ("active", "#b8c2cf")])
    for name, color, dark in (("Accent", ACCENT, ACCENT_DARK), ("Danger", DANGER, DANGER_DARK)):
        style.configure(f"{name}.TButton", background=color, foreground="#ffffff", bordercolor=color,
                        lightcolor=color, darkcolor=color, font=bold)
        style.map(f"{name}.TButton",
                  background=[("disabled", "#c9d1dc"), ("pressed", dark), ("active", dark)],
                  bordercolor=[("disabled", "#c9d1dc"), ("active", dark)],
                  lightcolor=[("disabled", "#c9d1dc"), ("active", dark)],
                  darkcolor=[("disabled", "#c9d1dc"), ("active", dark)],
                  foreground=[("disabled", "#ffffff")])
    style.configure("Big.Danger.TButton", padding=(16, 9), font=(family, 11, "bold"))
    style.configure("Big.Accent.TButton", padding=(16, 9), font=(family, 11, "bold"))

    for w in ("TEntry", "TCombobox", "TSpinbox"):
        style.configure(w, fieldbackground=SURFACE, bordercolor=BORDER, lightcolor=SURFACE,
                        darkcolor=SURFACE, padding=(5, 3), arrowcolor=MUTED, background=SURFACE)
        style.map(w, bordercolor=[("focus", ACCENT)], lightcolor=[("focus", ACCENT)],
                  fieldbackground=[("readonly", SURFACE), ("disabled", BG)])
    style.configure("TCheckbutton", background=SURFACE, foreground=TEXT, indicatorbackground=SURFACE,
                    indicatorforeground=ACCENT)
    style.map("TCheckbutton", background=[("active", SURFACE)],
              indicatorbackground=[("selected", ACCENT), ("disabled", BG)],
              indicatorforeground=[("selected", "#ffffff")])
    style.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor="#e5e9f0",
                    bordercolor="#e5e9f0", lightcolor=ACCENT, darkcolor=ACCENT, thickness=6)
    style.configure("Treeview", background=SURFACE, fieldbackground=SURFACE, bordercolor=BORDER,
                    rowheight=22)
    style.configure("Treeview.Heading", background=BG, foreground=MUTED, font=bold, relief="flat",
                    bordercolor=BORDER)
    style.map("Treeview.Heading", background=[("active", BG)])
    style.configure("TNotebook", background=SURFACE, bordercolor=BORDER, tabmargins=(0, 0, 0, 0))
    style.configure("TNotebook.Tab", background=BG, foreground=MUTED, padding=(14, 4), bordercolor=BORDER,
                    lightcolor=BG, font=bold)
    style.map("TNotebook.Tab", background=[("selected", SURFACE)], foreground=[("selected", TEXT)],
              lightcolor=[("selected", SURFACE)])
    style.configure("TPanedwindow", background=BG)
    style.configure("Sash", sashthickness=6, gripcount=0, background=BG)
    style.configure("Vertical.TScrollbar", background=BG, troughcolor=SURFACE, bordercolor=SURFACE,
                    arrowcolor=MUTED, lightcolor=BG, darkcolor=BG)
    return style


def card(parent, title: str, **pack) -> ttk.LabelFrame:
    """A titled white panel."""
    f = ttk.LabelFrame(parent, text=title.upper())
    if pack:
        f.pack(**pack)
    return f


class Badge(tk.Label):
    """A small colored pill showing a state such as READY or RUNNING."""

    def __init__(self, parent, text: str = "", state: str = "offline", **kw):
        super().__init__(parent, padx=10, pady=2, font=(tkfont.nametofont("TkDefaultFont").actual("family"),
                                                         9, "bold"), **kw)
        self.set(text, state)

    def set(self, text: str, state: str) -> None:
        fg, bg = STATE_COLORS.get(state, STATE_COLORS["idle"])
        self.configure(text=text, foreground=fg, background=bg)


def rich_text(parent, **kw) -> tk.Text:
    """A read-only, borderless Text with tags for key/value reports:
    ``key`` (muted label column), ``value``, ``head``, ``warn``, ``error``, ``ok``, ``mono``."""
    family = tkfont.nametofont("TkDefaultFont").actual("family")
    t = tk.Text(parent, wrap=tk.WORD, relief="flat", borderwidth=0, highlightthickness=0,
                background=SURFACE, foreground=TEXT, font=(family, 10), padx=2, pady=2,
                tabs=("118p",), spacing1=1, spacing3=1, cursor="arrow", **kw)
    t.tag_configure("key", foreground=MUTED)
    t.tag_configure("value", foreground=TEXT, font=(family, 10, "bold"))
    t.tag_configure("head", foreground=TEXT, font=(family, 11, "bold"), spacing1=6, spacing3=3)
    t.tag_configure("warn", foreground=WARN, background=WARN_LIGHT, lmargin1=4, lmargin2=4, rmargin=4,
                    spacing1=2, spacing3=2)
    t.tag_configure("error", foreground=DANGER_DARK, background=ERROR_LIGHT, lmargin1=4, lmargin2=4,
                    spacing1=2, spacing3=2)
    t.tag_configure("ok", foreground=OK)
    t.tag_configure("mono", font="TkFixedFont", foreground=MUTED)
    t.configure(state=tk.DISABLED)
    return t


def set_rich(text: tk.Text, parts) -> None:
    """Replace the content with ``parts``: a list of (string, tag) pairs."""
    text.configure(state=tk.NORMAL)
    text.delete("1.0", tk.END)
    for s, tag in parts:
        text.insert(tk.END, s, tag or ())
    text.configure(state=tk.DISABLED)


def kv(parts: list, key: str, value: str) -> None:
    """Append one ``key: value`` row to a ``set_rich`` parts list."""
    parts += [(key + "\t", "key"), (value + "\n", "value")]


def style_figure(fig) -> None:
    fig.set_facecolor(SURFACE)


def style_axes(ax) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BORDER)
    ax.tick_params(colors=MUTED, labelsize=8, length=3)
    ax.yaxis.label.set_color(MUTED)
    ax.xaxis.label.set_color(MUTED)
    ax.yaxis.label.set_fontsize(9)
    ax.xaxis.label.set_fontsize(9)
    ax.grid(True, color="#e8ecf1", lw=0.8)
    ax.set_axisbelow(True)
