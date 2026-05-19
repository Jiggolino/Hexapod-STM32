#!/usr/bin/env python3
"""
Hexapod servo calibration tool.

Usage:
    python servo_calibration.py [PORT]        e.g.  python servo_calibration.py /dev/ttyUSB0
    python servo_calibration.py COM3          (Windows)

If PORT is omitted the script tries to auto-detect the first available serial port.

STM32 protocol (115200 8N1):
    S <idx> <angle>\\n   — move servo idx (0-17) to angle
    P\\n                 — print #define trim values to UART (also shown in the log)

Servo index mapping:
    0  FR_COXA    9  FL_COXA
    1  FR_FEMUR   10 FL_FEMUR
    2  FR_TIBIA   11 FL_TIBIA
    3  MR_COXA    12 ML_COXA
    4  MR_FEMUR   13 ML_FEMUR
    5  MR_TIBIA   14 ML_TIBIA
    6  BR_COXA    15 BL_COXA
    7  BR_FEMUR   16 BL_FEMUR
    8  BR_TIBIA   17 BL_TIBIA
"""

import sys
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports

BAUD = 115200
NUM_SERVOS = 18

SERVO_NAMES = [
    "FR_COXA",  "FR_FEMUR",  "FR_TIBIA",
    "MR_COXA",  "MR_FEMUR",  "MR_TIBIA",
    "BR_COXA",  "BR_FEMUR",  "BR_TIBIA",
    "FL_COXA",  "FL_FEMUR",  "FL_TIBIA",
    "ML_COXA",  "ML_FEMUR",  "ML_TIBIA",
    "BL_COXA",  "BL_FEMUR",  "BL_TIBIA",
]

# Visual grouping: (group_label, servo_indices)
GROUPS = [
    ("Right board — FR (0-2)",  [0, 1, 2]),
    ("Right board — MR (3-5)",  [3, 4, 5]),
    ("Right board — BR (6-8)",  [6, 7, 8]),
    ("Left board — FL (9-11)",  [9, 10, 11]),
    ("Left board — ML (12-14)", [12, 13, 14]),
    ("Left board — BL (15-17)", [15, 16, 17]),
]


def auto_detect_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    # prefer USB/ACM ports
    for p in ports:
        if "USB" in p.description.upper() or "ACM" in p.device.upper():
            return p.device
    return ports[0].device


class CalibrationApp:
    def __init__(self, root, port):
        self.root = root
        self.root.title("Hexapod Servo Calibration")
        self.ser = None
        self.angles = [90.0] * NUM_SERVOS
        self._slider_vars = []
        self._angle_labels = []
        self._send_lock = threading.Lock()
        self._pending_error = None

        self._build_ui()
        self._connect(port)
        self._start_reader()

        if self._pending_error:
            self.root.after(200, lambda: messagebox.showerror("Connection error", self._pending_error))

    # ── Serial ──────────────────────────────────────────────────────────────

    def _connect(self, port):
        if port is None:
            port = auto_detect_port()
        if port is None:
            self._pending_error = "No serial port found.\nPass the port as an argument, e.g.:\n  python3 servo_calibration.py /dev/ttyACM0"
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.1)
            self._log(f"Connected to {port} @ {BAUD}\n")
        except serial.SerialException as e:
            self._pending_error = f"Could not open {port}:\n{e}\n\nMake sure CuteCom / other tools are closed."

    def _send(self, cmd: str):
        if self.ser and self.ser.is_open:
            with self._send_lock:
                self.ser.write((cmd + "\n").encode())

    def _start_reader(self):
        def reader():
            while self.ser and self.ser.is_open:
                try:
                    line = self.ser.readline().decode(errors="replace").strip()
                    if line:
                        self.root.after(0, self._log, line + "\n")
                except Exception:
                    pass
        t = threading.Thread(target=reader, daemon=True)
        t.start()

    # ── UI ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main = ttk.Frame(self.root, padding=8)
        main.grid(sticky="nsew")
        main.columnconfigure(0, weight=1)

        # Scrollable canvas for sliders
        canvas = tk.Canvas(main, borderwidth=0)
        vscroll = ttk.Scrollbar(main, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vscroll.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vscroll.grid(row=0, column=1, sticky="ns")
        main.rowconfigure(0, weight=1)

        slider_frame = ttk.Frame(canvas)
        canvas.create_window((0, 0), window=slider_frame, anchor="nw")
        slider_frame.bind("<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        # Mouse wheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        for group_label, indices in GROUPS:
            grp = ttk.LabelFrame(slider_frame, text=group_label, padding=6)
            grp.pack(fill="x", padx=4, pady=4)
            grp.columnconfigure(1, weight=1)

            for row_i, idx in enumerate(indices):
                ttk.Label(grp, text=f"{idx:2d}  {SERVO_NAMES[idx]}",
                          width=14).grid(row=row_i, column=0, sticky="w", padx=(0, 6))

                var = tk.DoubleVar(value=90.0)
                self._slider_vars.append((idx, var))

                slider = ttk.Scale(grp, from_=0, to=180, orient="horizontal",
                                   variable=var, length=400)
                slider.grid(row=row_i, column=1, sticky="ew", pady=2)
                grp.columnconfigure(1, weight=1)

                angle_lbl = ttk.Label(grp, text="90.0°", width=7)
                angle_lbl.grid(row=row_i, column=2, padx=(6, 0))
                self._angle_labels.append((idx, angle_lbl))

                # Bind after widget list is set up
                def make_cb(servo_idx, v, lbl):
                    last = [90.0]
                    def cb(*_):
                        val = round(v.get(), 1)
                        if abs(val - last[0]) < 0.09:
                            return
                        last[0] = val
                        lbl.config(text=f"{val:.1f}°")
                        self.angles[servo_idx] = val
                        self._send(f"S {servo_idx} {int(val)}")
                    return cb
                var.trace_add("write", make_cb(idx, var, angle_lbl))

        # Bottom bar
        bottom = ttk.Frame(main, padding=(0, 4))
        bottom.grid(row=1, column=0, columnspan=2, sticky="ew")

        ttk.Button(bottom, text="Reset all to 90°", command=self._reset_all).pack(side="left", padx=4)
        ttk.Button(bottom, text="Print calibration (P)", command=self._print_cal).pack(side="left", padx=4)
        ttk.Button(bottom, text="Copy to clipboard", command=self._copy_output).pack(side="left", padx=4)

        # Log
        log_frame = ttk.LabelFrame(main, text="Serial log", padding=4)
        log_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self.log_box = scrolledtext.ScrolledText(log_frame, height=8, state="disabled",
                                                  font=("Courier", 9))
        self.log_box.pack(fill="x")

        self._cal_output = ""

    def _log(self, text):
        self.log_box.config(state="normal")
        self.log_box.insert("end", text)
        self.log_box.see("end")
        self.log_box.config(state="disabled")
        # collect calibration output lines
        if text.strip().startswith("#define"):
            self._cal_output += text

    # ── Actions ─────────────────────────────────────────────────────────────

    def _reset_all(self):
        for idx, var in self._slider_vars:
            var.set(90.0)

    def _build_cal_text(self):
        a = self.angles
        def trim(i):
            return a[i] - 90.0

        lines = []
        lines.append("/* RIGHT board — FR(ch 0-2), MR(ch 3-5), BR(ch 6-8) */")
        lines.append(f"#define SERVO_TRIM_FR_COXA_DEG   {trim(0):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_FR_FEMUR_DEG  {trim(1):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_FR_TIBIA_DEG  {trim(2):>6.1f}f")
        lines.append("")
        lines.append(f"#define SERVO_TRIM_MR_COXA_DEG   {trim(3):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_MR_FEMUR_DEG  {trim(4):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_MR_TIBIA_DEG  {trim(5):>6.1f}f")
        lines.append("")
        lines.append(f"#define SERVO_TRIM_BR_COXA_DEG   {trim(6):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_BR_FEMUR_DEG  {trim(7):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_BR_TIBIA_DEG  {trim(8):>6.1f}f")
        lines.append("")
        lines.append("/* LEFT board — FL(ch 0-2), ML(ch 3-5), BL(ch 6-8) */")
        lines.append(f"#define SERVO_TRIM_FL_COXA_DEG   {trim(9):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_FL_FEMUR_DEG  {trim(10):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_FL_TIBIA_DEG  {trim(11):>6.1f}f")
        lines.append("")
        lines.append(f"#define SERVO_TRIM_ML_COXA_DEG   {trim(12):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_ML_FEMUR_DEG  {trim(13):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_ML_TIBIA_DEG  {trim(14):>6.1f}f")
        lines.append("")
        lines.append(f"#define SERVO_TRIM_BL_COXA_DEG   {trim(15):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_BL_FEMUR_DEG  {trim(16):>6.1f}f")
        lines.append(f"#define SERVO_TRIM_BL_TIBIA_DEG  {trim(17):>6.1f}f")
        return "\n".join(lines)

    def _print_cal(self):
        text = self._build_cal_text()
        # Print to terminal so it can be selected and copied easily
        print("\n" + "=" * 56)
        print(text)
        print("=" * 56 + "\n")
        # Also show in the GUI log
        self._log("\n" + text + "\n\n")
        self._cal_output = text

    def _copy_output(self):
        text = self._build_cal_text()
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo("Copied", "Calibration #defines copied to clipboard.")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    root = tk.Tk()
    root.geometry("700x800")
    app = CalibrationApp(root, port)
    root.mainloop()
    if app.ser and app.ser.is_open:
        app.ser.close()


if __name__ == "__main__":
    main()
