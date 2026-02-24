#!/usr/bin/env python3
from __future__ import annotations

# Tkinter UI that launches main_drive_target.py as a subprocess and shows live telemetry from its stdout
# parses UI_DATA lines from the subprocess output to update the angle and distance displays in real time

import math
import queue
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

try:
    import tkinter as tk
    from tkinter import scrolledtext
except ModuleNotFoundError as exc:
    if exc.name == "_tkinter":
        print(
            "Tkinter is not available in this Python interpreter.\n"
            "Use an interpreter with Tk support, for example:\n"
            "  python3 target_control_ui.py\n"
            "Or install Tk for Homebrew Python 3.13:\n"
            "  brew install python-tk@3.13",
            file=sys.stderr,
        )
        raise SystemExit(1)
    raise


UI_DATA_RE = re.compile(
    r"\[UI_DATA\]\s+angle_deg=(?P<angle>-?\d+(?:\.\d+)?)\s+distance_cm=(?P<dist>None|-?\d+(?:\.\d+)?)"
)
DISTANCE_CM_RE = re.compile(r"\((?P<cm>\d+(?:\.\d+)?)\s*cm\)")
DISTANCE_Z_RE = re.compile(r"\bZ=(?P<m>\d+(?:\.\d+)?)m\b")


def _hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    value = hex_color.strip().lstrip("#")
    if len(value) != 6:
        raise ValueError(f"Invalid color '{hex_color}'")
    return int(value[0:2], 16), int(value[2:4], 16), int(value[4:6], 16)


def _rgb_to_hex(rgb: tuple[int, int, int]) -> str:
    r, g, b = rgb
    return f"#{r:02x}{g:02x}{b:02x}"


def _blend(c1: str, c2: str, t: float) -> str:
    t = max(0.0, min(1.0, t))
    r1, g1, b1 = _hex_to_rgb(c1)
    r2, g2, b2 = _hex_to_rgb(c2)
    return _rgb_to_hex(
        (
            int(r1 + (r2 - r1) * t),
            int(g1 + (g2 - g1) * t),
            int(b1 + (b2 - b1) * t),
        )
    )


class TargetControlUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.project_root = Path(__file__).resolve().parent
        self.proc: Optional[subprocess.Popen[str]] = None
        self._queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self._io_thread: Optional[threading.Thread] = None

        self._current_target = tk.StringVar(value="-")
        self._status_text = tk.StringVar(value="Ready")
        self._distance_text = tk.StringVar(value="--.-")
        self._angle_text = tk.StringVar(value="--.-°")

        self._status_tone = "idle"
        self._status_accent = "#3b82f6"
        self._running = False
        self._pulse_phase = 0.0

        self._distance_known = False
        self._distance_target_cm = 0.0
        self._distance_display_cm = 0.0

        self._triangle_normal = "#ffd7ba"
        self._triangle_hover = "#ffc49c"
        self._square_normal = "#c9f5ef"
        self._square_hover = "#adede4"
        self._stop_normal = "#f7b9c4"
        self._stop_hover = "#ee9fae"
        self._triangle_text = "#1f2937"
        self._square_text = "#0f3a38"
        self._stop_text = "#6d1126"
        self._cylinder_normal = "#d8deff"
        self._cylinder_hover = "#c5ceff"
        self._cylinder_text = "#1f2a63"
        self._dev_toggle_normal = "#e6edf8"
        self._dev_toggle_hover = "#d8e2f1"
        self._dev_toggle_text = "#234063"

        self._show_dev_controls = tk.BooleanVar(value=False)

        self._status_palette = {
            "idle": ("#dbeafe", "#1e3a8a", "#3b82f6"),
            "running": ("#fef3c7", "#92400e", "#f59e0b"),
            "success": ("#dcfce7", "#166534", "#22c55e"),
            "warning": ("#fee2e2", "#991b1b", "#ef4444"),
            "error": ("#fecaca", "#7f1d1d", "#dc2626"),
        }

        self._orb_specs: list[dict[str, float | str | int]] = []

        self._build_ui()
        self._set_status("Ready", "idle")

        self.root.after(120, self._drain_queue)
        self.root.after(33, self._animate_ui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.root.title("Stereo Target Control")
        self.root.geometry("1180x760")
        self.root.minsize(920, 640)

        self.bg_canvas = tk.Canvas(self.root, highlightthickness=0, bg="#0b1020")
        self.bg_canvas.pack(fill="both", expand=True)
        self.bg_canvas.bind("<Configure>", self._on_canvas_resize)

        self._init_orbs()

        self.panel = tk.Frame(
            self.bg_canvas,
            bg="#f8fbff",
            highlightthickness=1,
            highlightbackground="#8faec8",
            bd=0,
        )
        self.panel_window = self.bg_canvas.create_window(
            0,
            0,
            window=self.panel,
            anchor="center",
        )

        hero = tk.Frame(self.panel, bg="#102347", padx=28, pady=22)
        hero.pack(fill="x")

        title = tk.Label(
            hero,
            text="Stereo Shape Driver",
            font=("Avenir Next", 32, "bold"),
            fg="#f9fbff",
            bg="#102347",
        )
        title.pack(anchor="w")

        subtitle = tk.Label(
            hero,
            text="Choose Triangle or Square target, run auto-drive, and watch live stereo distance telemetry.",
            font=("Avenir Next", 13),
            fg="#c5d4eb",
            bg="#102347",
        )
        subtitle.pack(anchor="w", pady=(6, 0))

        content = tk.Frame(self.panel, bg="#f8fbff")
        content.pack(fill="both", expand=True, padx=18, pady=16)
        content.grid_columnconfigure(0, weight=3, minsize=320)
        content.grid_columnconfigure(1, weight=5, minsize=440)
        content.grid_rowconfigure(0, weight=1)

        control_card = tk.Frame(
            content,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#c7d9e9",
            padx=16,
            pady=16,
        )
        control_card.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

        telemetry_card = tk.Frame(
            content,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#c7d9e9",
            padx=16,
            pady=16,
        )
        telemetry_card.grid(row=0, column=1, sticky="nsew")
        telemetry_card.grid_rowconfigure(4, weight=1)
        telemetry_card.grid_columnconfigure(0, weight=1)

        control_title = tk.Label(
            control_card,
            text="Target Selection",
            font=("Avenir Next", 18, "bold"),
            fg="#0f2e56",
            bg="#ffffff",
        )
        control_title.pack(anchor="w")

        control_hint = tk.Label(
            control_card,
            text=(
                "Square button launches Cube mode. Triangle button launches Pyramid mode. "
                "Enable Dev Mode to show the Cylinder button."
            ),
            font=("Avenir Next", 11),
            fg="#5f7592",
            bg="#ffffff",
            justify="left",
            wraplength=300,
        )
        control_hint.pack(anchor="w", pady=(4, 16))

        self.dev_toggle_button = tk.Checkbutton(
            control_card,
            variable=self._show_dev_controls,
            command=self._on_dev_toggle,
            indicatoron=False,
            font=("Avenir Next", 11, "bold"),
            anchor="w",
            justify="left",
            bg=self._dev_toggle_normal,
            fg=self._dev_toggle_text,
            activebackground=self._dev_toggle_hover,
            activeforeground=self._dev_toggle_text,
            selectcolor=self._dev_toggle_hover,
            relief="flat",
            cursor="hand2",
            padx=14,
            pady=8,
            bd=0,
            highlightthickness=0,
        )
        self.dev_toggle_button.pack(fill="x", pady=(0, 10))
        self._bind_hover(self.dev_toggle_button, self._dev_toggle_normal, self._dev_toggle_hover)

        self.triangle_button = tk.Button(
            control_card,
            text="▲  Triangle\nPyramid Target",
            command=lambda: self._start_run("Pyramid"),
            font=("Avenir Next", 16, "bold"),
            justify="left",
            anchor="w",
            bg=self._triangle_normal,
            fg=self._triangle_text,
            activebackground=self._triangle_hover,
            activeforeground=self._triangle_text,
            disabledforeground="#334155",
            relief="flat",
            cursor="hand2",
            padx=22,
            pady=16,
            bd=0,
            highlightthickness=0,
        )
        self.triangle_button.pack(fill="x", pady=(0, 10))
        self._bind_hover(self.triangle_button, self._triangle_normal, self._triangle_hover)

        self.square_button = tk.Button(
            control_card,
            text="■  Square\nCube Target",
            command=lambda: self._start_run("Cube"),
            font=("Avenir Next", 16, "bold"),
            justify="left",
            anchor="w",
            bg=self._square_normal,
            fg=self._square_text,
            activebackground=self._square_hover,
            activeforeground=self._square_text,
            disabledforeground="#334155",
            relief="flat",
            cursor="hand2",
            padx=22,
            pady=16,
            bd=0,
            highlightthickness=0,
        )
        self.square_button.pack(fill="x", pady=(0, 14))
        self._bind_hover(self.square_button, self._square_normal, self._square_hover)

        self.cylinder_button = tk.Button(
            control_card,
            text="●  Cylinder\nDev Target",
            command=lambda: self._start_run("Cylinder"),
            font=("Avenir Next", 16, "bold"),
            justify="left",
            anchor="w",
            bg=self._cylinder_normal,
            fg=self._cylinder_text,
            activebackground=self._cylinder_hover,
            activeforeground=self._cylinder_text,
            disabledforeground="#334155",
            relief="flat",
            cursor="hand2",
            padx=22,
            pady=16,
            bd=0,
            highlightthickness=0,
        )
        self._bind_hover(self.cylinder_button, self._cylinder_normal, self._cylinder_hover)

        self.stop_button = tk.Button(
            control_card,
            text="Stop Current Run",
            command=self._stop_run,
            font=("Avenir Next", 13, "bold"),
            bg=self._stop_normal,
            fg=self._stop_text,
            activebackground=self._stop_hover,
            activeforeground=self._stop_text,
            disabledforeground="#7b4e59",
            relief="flat",
            state="disabled",
            cursor="hand2",
            padx=14,
            pady=12,
            bd=0,
            highlightthickness=0,
        )
        self.stop_button.pack(fill="x", pady=(0, 18))
        self._bind_hover(self.stop_button, self._stop_normal, self._stop_hover)

        status_row = tk.Frame(control_card, bg="#ffffff")
        status_row.pack(fill="x")

        self.status_dot_canvas = tk.Canvas(
            status_row,
            width=18,
            height=18,
            highlightthickness=0,
            bd=0,
            bg="#ffffff",
        )
        self.status_dot_canvas.pack(side="left", padx=(0, 8))
        self.status_dot_item = self.status_dot_canvas.create_oval(
            2, 2, 16, 16, fill="#3b82f6", outline=""
        )

        self.status_pill = tk.Label(
            status_row,
            textvariable=self._status_text,
            font=("Avenir Next", 12, "bold"),
            bg="#dbeafe",
            fg="#1e3a8a",
            padx=12,
            pady=6,
        )
        self.status_pill.pack(side="left", fill="x", expand=True)

        telemetry_title = tk.Label(
            telemetry_card,
            text="Live Telemetry",
            font=("Avenir Next", 18, "bold"),
            fg="#0f2e56",
            bg="#ffffff",
        )
        telemetry_title.grid(row=0, column=0, sticky="w")

        distance_card = tk.Frame(
            telemetry_card,
            bg="#0f1f3e",
            highlightthickness=1,
            highlightbackground="#2f4f82",
            padx=16,
            pady=14,
        )
        distance_card.grid(row=1, column=0, sticky="ew", pady=(10, 12))
        distance_card.grid_columnconfigure(0, weight=1)

        distance_title = tk.Label(
            distance_card,
            text="STEREO DISTANCE",
            font=("Avenir Next", 10, "bold"),
            fg="#8eb1e5",
            bg="#0f1f3e",
        )
        distance_title.grid(row=0, column=0, sticky="w")

        value_row = tk.Frame(distance_card, bg="#0f1f3e")
        value_row.grid(row=1, column=0, sticky="w", pady=(6, 4))

        self.distance_value_label = tk.Label(
            value_row,
            textvariable=self._distance_text,
            font=("Avenir Next", 44, "bold"),
            fg="#ffffff",
            bg="#0f1f3e",
        )
        self.distance_value_label.pack(side="left")

        distance_unit = tk.Label(
            value_row,
            text="cm",
            font=("Avenir Next", 18, "bold"),
            fg="#89a7d2",
            bg="#0f1f3e",
            padx=8,
            pady=12,
        )
        distance_unit.pack(side="left")

        self.distance_meter = tk.Canvas(
            distance_card,
            height=26,
            highlightthickness=0,
            bd=0,
            bg="#0f1f3e",
        )
        self.distance_meter.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        self._meter_bg = self.distance_meter.create_rectangle(4, 4, 400, 22, fill="#293e66", outline="")
        self._meter_fill = self.distance_meter.create_rectangle(4, 4, 4, 22, fill="#38bdf8", outline="")

        meter_legend = tk.Frame(distance_card, bg="#0f1f3e")
        meter_legend.grid(row=3, column=0, sticky="ew", pady=(4, 0))
        tk.Label(
            meter_legend,
            text="Near",
            font=("Avenir Next", 9),
            fg="#89a7d2",
            bg="#0f1f3e",
        ).pack(side="left")
        tk.Label(
            meter_legend,
            text="Far",
            font=("Avenir Next", 9),
            fg="#89a7d2",
            bg="#0f1f3e",
        ).pack(side="right")

        stats_grid = tk.Frame(telemetry_card, bg="#ffffff")
        stats_grid.grid(row=2, column=0, sticky="ew", pady=(0, 12))
        stats_grid.grid_columnconfigure(0, weight=1)
        stats_grid.grid_columnconfigure(1, weight=1)

        self._make_stat_tile(stats_grid, "Current Target", self._current_target, 0, 0, "#dbeafe")
        self._make_stat_tile(stats_grid, "Detected Angle", self._angle_text, 0, 1, "#d1fae5")

        log_label = tk.Label(
            telemetry_card,
            text="System Logs",
            font=("Avenir Next", 12, "bold"),
            fg="#355279",
            bg="#ffffff",
        )
        log_label.grid(row=3, column=0, sticky="w", pady=(0, 6))

        self.log_box = scrolledtext.ScrolledText(
            telemetry_card,
            wrap="word",
            font=("Menlo", 10),
            bg="#0b132b",
            fg="#dbeafe",
            insertbackground="#dbeafe",
            relief="flat",
            height=12,
            padx=10,
            pady=10,
        )
        self.log_box.grid(row=4, column=0, sticky="nsew")
        self.log_box.configure(state="disabled")
        self.log_box.tag_configure("default", foreground="#dbeafe")
        self.log_box.tag_configure("info", foreground="#93c5fd")
        self.log_box.tag_configure("warn", foreground="#fbbf24")
        self.log_box.tag_configure("error", foreground="#f87171")
        self.log_box.tag_configure("success", foreground="#4ade80")
        self.log_box.tag_configure("ui", foreground="#e9d5ff")

        self._sync_dev_toggle_label()
        self._refresh_cylinder_visibility()
        self._set_running_state(False)

    def _init_orbs(self) -> None:
        spec_data = [
            {"x": 0.18, "y": 0.16, "r": 180, "amp_x": 36, "amp_y": 22, "speed": 0.42, "phase": 0.2, "color": "#19345f"},
            {"x": 0.84, "y": 0.22, "r": 160, "amp_x": 28, "amp_y": 18, "speed": 0.37, "phase": 1.1, "color": "#24426e"},
            {"x": 0.12, "y": 0.82, "r": 150, "amp_x": 22, "amp_y": 28, "speed": 0.33, "phase": 2.6, "color": "#1a3c68"},
            {"x": 0.88, "y": 0.76, "r": 140, "amp_x": 24, "amp_y": 20, "speed": 0.40, "phase": 3.4, "color": "#224a7a"},
        ]
        for spec in spec_data:
            item = self.bg_canvas.create_oval(0, 0, 0, 0, fill=str(spec["color"]), outline="")
            spec["item"] = item
            self._orb_specs.append(spec)

    def _make_stat_tile(
        self,
        parent: tk.Widget,
        title: str,
        var: tk.StringVar,
        row: int,
        column: int,
        bg_color: str,
    ) -> None:
        tile = tk.Frame(
            parent,
            bg=bg_color,
            highlightthickness=1,
            highlightbackground="#bfd3ea",
            padx=12,
            pady=10,
        )
        tile.grid(row=row, column=column, sticky="ew", padx=(0 if column == 0 else 6, 6 if column == 0 else 0))

        tk.Label(
            tile,
            text=title,
            font=("Avenir Next", 10, "bold"),
            fg="#31537d",
            bg=bg_color,
        ).pack(anchor="w")

        tk.Label(
            tile,
            textvariable=var,
            font=("Avenir Next", 16, "bold"),
            fg="#0f2e56",
            bg=bg_color,
            pady=2,
        ).pack(anchor="w")

    def _bind_hover(self, button: tk.Button, normal_bg: str, hover_bg: str) -> None:
        def on_enter(_: tk.Event) -> None:
            if str(button.cget("state")) == "normal":
                button.configure(bg=hover_bg)

        def on_leave(_: tk.Event) -> None:
            if str(button.cget("state")) == "normal":
                button.configure(bg=normal_bg)

        button.bind("<Enter>", on_enter)
        button.bind("<Leave>", on_leave)

    def _sync_dev_toggle_label(self) -> None:
        if self._show_dev_controls.get():
            self.dev_toggle_button.configure(text="Dev Mode: ON  •  Cylinder Visible")
        else:
            self.dev_toggle_button.configure(text="Dev Mode: OFF  •  Show Cylinder Button")

    def _refresh_cylinder_visibility(self) -> None:
        visible = bool(self._show_dev_controls.get())
        is_visible = self.cylinder_button.winfo_manager() == "pack"

        if visible and not is_visible:
            self.cylinder_button.pack(fill="x", pady=(0, 14), before=self.stop_button)
        elif not visible and is_visible:
            self.cylinder_button.pack_forget()

    def _on_dev_toggle(self) -> None:
        self._sync_dev_toggle_label()
        self._refresh_cylinder_visibility()
        state_text = "enabled" if self._show_dev_controls.get() else "hidden"
        self._append_log(f"[UI] Dev cylinder button is now {state_text}.")

    def _on_canvas_resize(self, event: tk.Event) -> None:
        width = max(1, int(event.width))
        height = max(1, int(event.height))
        self._draw_gradient(width, height)
        self._position_panel(width, height)
        self._update_distance_meter(self._distance_display_cm)

    def _draw_gradient(self, width: int, height: int) -> None:
        self.bg_canvas.delete("gradient")
        steps = 84
        top = "#0a1022"
        bottom = "#173c6a"

        for i in range(steps):
            t = i / max(1, steps - 1)
            color = _blend(top, bottom, t)
            y0 = int(height * (i / steps))
            y1 = int(height * ((i + 1) / steps))
            self.bg_canvas.create_rectangle(
                0,
                y0,
                width,
                y1,
                outline="",
                fill=color,
                tags=("gradient",),
            )

        self.bg_canvas.tag_lower("gradient")
        for orb in self._orb_specs:
            self.bg_canvas.tag_raise(int(orb["item"]))
        self.bg_canvas.tag_raise(self.panel_window)

    def _position_panel(self, width: int, height: int) -> None:
        panel_width = max(860, min(width - 80, 1220))
        self.bg_canvas.coords(self.panel_window, width / 2, height / 2)
        self.bg_canvas.itemconfigure(self.panel_window, width=panel_width)

    def _animate_ui(self) -> None:
        now = time.monotonic()
        width = max(1, self.bg_canvas.winfo_width())
        height = max(1, self.bg_canvas.winfo_height())

        for orb in self._orb_specs:
            x = float(orb["x"]) * width + math.sin(now * float(orb["speed"]) + float(orb["phase"])) * float(orb["amp_x"])
            y = float(orb["y"]) * height + math.cos(now * float(orb["speed"]) * 0.87 + float(orb["phase"])) * float(orb["amp_y"])
            r = float(orb["r"])
            self.bg_canvas.coords(int(orb["item"]), x - r, y - r, x + r, y + r)

        self._animate_status_dot()
        self._animate_distance_value()

        self.root.after(33, self._animate_ui)

    def _animate_status_dot(self) -> None:
        if self._running:
            self._pulse_phase += 0.18
            pulse = (math.sin(self._pulse_phase) + 1.0) * 0.5
            color = _blend(self._status_accent, "#ffffff", 0.20 + (0.45 * pulse))
        else:
            color = self._status_accent

        self.status_dot_canvas.itemconfigure(self.status_dot_item, fill=color)

    def _animate_distance_value(self) -> None:
        if self._distance_known:
            delta = self._distance_target_cm - self._distance_display_cm
            self._distance_display_cm += delta * 0.15
            if abs(delta) < 0.05:
                self._distance_display_cm = self._distance_target_cm
            self._distance_text.set(f"{self._distance_display_cm:0.1f}")
            self._update_distance_meter(self._distance_display_cm)
        else:
            if self._distance_display_cm > 0.1:
                self._distance_display_cm *= 0.88
                self._update_distance_meter(self._distance_display_cm)
            else:
                self._distance_display_cm = 0.0
                self._update_distance_meter(0.0)

    def _set_distance_cm(self, value: Optional[float]) -> None:
        if value is None:
            self._distance_known = False
            self._distance_target_cm = 0.0
            self._distance_display_cm = 0.0
            self._distance_text.set("--.-")
            self._update_distance_meter(0.0)
            return

        self._distance_known = True
        self._distance_target_cm = max(0.0, float(value))
        if self._distance_display_cm <= 0.0:
            self._distance_display_cm = self._distance_target_cm
        self._update_distance_meter(self._distance_display_cm)

    def _update_distance_meter(self, distance_cm: float) -> None:
        meter_w = max(220, int(self.distance_meter.winfo_width()))
        meter_h = max(24, int(self.distance_meter.winfo_height()))
        pad = 4
        usable_w = max(1, meter_w - (2 * pad))

        distance_clamped = max(0.0, min(200.0, distance_cm))
        ratio = distance_clamped / 200.0
        fill_x = pad + int(usable_w * ratio)

        self.distance_meter.coords(self._meter_bg, pad, pad, meter_w - pad, meter_h - pad)
        self.distance_meter.coords(self._meter_fill, pad, pad, fill_x, meter_h - pad)

        if ratio < 0.15:
            fill_color = "#ef4444"
        elif ratio < 0.35:
            fill_color = "#f97316"
        elif ratio < 0.60:
            fill_color = "#facc15"
        elif ratio < 0.85:
            fill_color = "#22c55e"
        else:
            fill_color = "#38bdf8"
        self.distance_meter.itemconfigure(self._meter_fill, fill=fill_color)

    def _set_status(self, text: str, tone: str) -> None:
        bg, fg, accent = self._status_palette.get(tone, self._status_palette["idle"])
        self._status_text.set(text)
        self._status_tone = tone
        self._status_accent = accent
        self.status_pill.configure(bg=bg, fg=fg)

    def _start_run(self, target: str) -> None:
        if self.proc is not None:
            self._set_status("A run is already active. Stop it first.", "warning")
            return

        self._current_target.set(target)
        self._angle_text.set("--.-°")
        self._set_distance_cm(None)
        self._set_status(f"Running scan for {target}...", "running")
        self._set_running_state(True)

        cmd = [sys.executable, "-u", "main_drive_target.py", "--target", target]
        self._append_log(f"$ {' '.join(cmd)}")
        try:
            self.proc = subprocess.Popen(
                cmd,
                cwd=self.project_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self.proc = None
            self._set_running_state(False)
            self._set_status(f"Failed to start: {exc}", "error")
            self._append_log(f"[UI] Failed to start process: {exc}")
            return

        self._io_thread = threading.Thread(target=self._read_process_output, daemon=True)
        self._io_thread.start()

    def _set_running_state(self, running: bool) -> None:
        self._running = running

        if running:
            self.dev_toggle_button.configure(
                state="disabled",
                bg="#e2e8f0",
                fg="#64748b",
            )
            self.triangle_button.configure(
                state="disabled",
                bg="#f2dcc9",
                fg="#334155",
            )
            self.square_button.configure(
                state="disabled",
                bg="#d6ebe8",
                fg="#334155",
            )
            self.cylinder_button.configure(
                state="disabled",
                bg="#dfe4f6",
                fg="#334155",
            )
            self.stop_button.configure(
                state="normal",
                bg=self._stop_normal,
                fg=self._stop_text,
            )
        else:
            self.dev_toggle_button.configure(
                state="normal",
                bg=self._dev_toggle_normal,
                fg=self._dev_toggle_text,
            )
            self.triangle_button.configure(
                state="normal",
                bg=self._triangle_normal,
                fg=self._triangle_text,
            )
            self.square_button.configure(
                state="normal",
                bg=self._square_normal,
                fg=self._square_text,
            )
            self.cylinder_button.configure(
                state="normal",
                bg=self._cylinder_normal,
                fg=self._cylinder_text,
            )
            self.stop_button.configure(
                state="disabled",
                bg=self._stop_normal,
                fg=self._stop_text,
            )

    def _read_process_output(self) -> None:
        if self.proc is None or self.proc.stdout is None:
            return

        for line in self.proc.stdout:
            self._queue.put(("line", line.rstrip("\n")))

        code = self.proc.wait()
        self._queue.put(("done", str(code)))

    def _drain_queue(self) -> None:
        while True:
            try:
                event, payload = self._queue.get_nowait()
            except queue.Empty:
                break

            if event == "line":
                self._handle_output_line(payload)
            elif event == "done":
                self._on_process_exit(int(payload))

        self.root.after(120, self._drain_queue)

    def _handle_output_line(self, line: str) -> None:
        self._append_log(line)

        if not line:
            return

        ui_match = UI_DATA_RE.search(line)
        if ui_match:
            angle = float(ui_match.group("angle"))
            dist = ui_match.group("dist")
            self._angle_text.set(f"{angle:.1f}°")
            if dist == "None":
                self._set_distance_cm(None)
            else:
                self._set_distance_cm(float(dist))
            self._set_status("Object found. Driving to target.", "success")
            return

        cm = self._extract_distance_cm(line)
        if cm is not None:
            self._set_distance_cm(cm)

        if "[Scanner] Target CONFIRMED" in line:
            self._set_status("Target confirmed by both cameras.", "success")
        elif "[INFO] No object detected" in line:
            self._set_status("No object detected.", "warning")
        elif "Traceback" in line or "[ERROR]" in line:
            self._set_status("Error while running.", "error")

    @staticmethod
    def _extract_distance_cm(line: str) -> Optional[float]:
        cm_match = DISTANCE_CM_RE.search(line)
        if cm_match:
            return float(cm_match.group("cm"))

        z_match = DISTANCE_Z_RE.search(line)
        if z_match:
            return float(z_match.group("m")) * 100.0

        return None

    def _on_process_exit(self, code: int) -> None:
        self.proc = None
        self._set_running_state(False)

        if code == 0:
            if self._status_tone == "running":
                self._set_status("Run completed.", "idle")
            self._append_log("[UI] Process finished successfully.")
        else:
            self._set_status(f"Run failed (exit code {code}).", "error")
            self._append_log(f"[UI] Process exited with code {code}.")

    def _stop_run(self) -> None:
        if self.proc is None:
            return

        self._set_status("Stopping current run...", "warning")
        self._append_log("[UI] Stopping current run...")
        try:
            self.proc.terminate()
            self.proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=2)
        except Exception as exc:
            self._set_status(f"Failed to stop: {exc}", "error")
            self._append_log(f"[UI] Failed to stop process cleanly: {exc}")

    def _append_log(self, text: str) -> None:
        tag = "default"
        upper = text.upper()
        if text.startswith("$") or text.startswith("[UI]"):
            tag = "ui"
        elif "TRACEBACK" in upper or "ERROR" in upper:
            tag = "error"
        elif "CONFIRMED" in upper or "FINISHED SUCCESSFULLY" in upper:
            tag = "success"
        elif "NO OBJECT DETECTED" in upper or "UNAVAILABLE" in upper:
            tag = "warn"
        elif text.startswith("[INFO]") or text.startswith("[Scanner]") or text.startswith("servo="):
            tag = "info"

        self.log_box.configure(state="normal")
        self.log_box.insert("end", text + "\n", tag)

        line_count = int(self.log_box.index("end-1c").split(".")[0])
        if line_count > 1400:
            self.log_box.delete("1.0", "250.0")

        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _on_close(self) -> None:
        self._stop_run()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    TargetControlUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
