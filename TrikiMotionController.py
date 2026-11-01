"""
Triki Motion Controller
A configurable 3D orientation-to-keyboard controller for Żabka Triki.

The BLE UUIDs, IMU packet layout and wake/sleep commands follow the
unofficial-triki-api-py project by Wojciech B. (Wojtekb30). The physical-button
flag in byte 1 is based on independent TrikiScope protocol research:
https://github.com/Maku-hub/TrikiScope

This application uses the same documented BLE protocol through Bleak so that
the user can select a specific scanned device in the GUI.
"""

import asyncio
import json
import math
import queue
import struct
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

try:
    import pydirectinput
    pydirectinput.PAUSE = 0
except ImportError:
    pydirectinput = None


APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "triki_motion_config.json"

# Nordic UART service UUIDs used by the Triki API.
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BATTERY_UUID = "00002a19-0000-1000-8000-00805f9b34fb"

# Exact IMU wake/sleep commands documented by unofficial-triki-api-py.
WAKE_COMMAND = bytes.fromhex("20 10 00 D0 07 34 00 03")
SLEEP_COMMAND = bytes.fromhex("20 00 00 00 00 00 00")

KEY_OPTIONS = [
    "none", "w", "a", "s", "d", "up", "down", "left", "right",
    "space", "shift", "ctrl", "alt", "q", "e", "r", "f", "z", "x",
    "c", "1", "2", "3", "4", "5", "esc", "lmb", "rmb"
]


# ----------------------------- Quaternion math -----------------------------

def v_norm(v):
    return math.sqrt(sum(x * x for x in v))


def v_normalize(v):
    n = v_norm(v)
    if n < 1e-9:
        return (0.0, 0.0, 1.0)
    return tuple(x / n for x in v)


def v_cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def q_normalize(q):
    n = math.sqrt(sum(x * x for x in q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(x / n for x in q)


def q_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )


def q_rotate(q, v):
    p = (0.0, v[0], v[1], v[2])
    r = q_multiply(q_multiply(q, p), q_conjugate(q))
    return (r[1], r[2], r[3])


def q_from_two_vectors(source, target):
    """Quaternion rotating source vector onto target vector."""
    a = v_normalize(source)
    b = v_normalize(target)
    dot = sum(a[i] * b[i] for i in range(3))

    if dot < -0.999999:
        # 180-degree rotation: choose an axis perpendicular to source.
        axis = v_cross(a, (1.0, 0.0, 0.0))
        if v_norm(axis) < 1e-6:
            axis = v_cross(a, (0.0, 1.0, 0.0))
        axis = v_normalize(axis)
        return (0.0, axis[0], axis[1], axis[2])

    c = v_cross(a, b)
    return q_normalize((1.0 + dot, c[0], c[1], c[2]))


def q_to_euler_degrees(q):
    """Return roll, pitch, yaw in degrees (ZYX convention)."""
    w, x, y, z = q

    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def wrap_degrees(value):
    return (value + 180.0) % 360.0 - 180.0


# ------------------------------ IMU fusion ----------------------------------

class OrientationFilter:
    """
    6-axis complementary/Mahony-style orientation filter.

    Gyroscope integrates full 3D rotation. Accelerometer corrects roll/pitch
    drift by comparing measured gravity with gravity predicted by the current
    quaternion. Yaw has no absolute correction because the Triki data stream
    contains no magnetometer.
    """

    def __init__(self):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.reference = (1.0, 0.0, 0.0, 0.0)
        self.last_time = None
        self.calibrated = False
        self.kp = 1.8
        self.gyro_scale = 14.286  # LSM6DSL: 70 mdps/LSB

    def calibrate(self, samples):
        if len(samples) < 20:
            raise ValueError("Not enough samples for calibration.")

        acc = tuple(
            sum(getattr(s, name) for s in samples) / len(samples)
            for name in ("ax", "ay", "az")
        )
        gyro = tuple(
            sum(getattr(s, name) for s in samples) / len(samples)
            for name in ("gx", "gy", "gz")
        )

        if v_norm(acc) < 1.0:
            raise ValueError("Accelerometer signal is too small.")

        # Orient the estimated world-up vector to the gravity vector at rest.
        self.q = q_from_two_vectors(acc, (0.0, 0.0, 1.0))
        self.reference = self.q
        self.gyro_bias = gyro
        self.last_time = None
        self.calibrated = True

    def update(self, sample, gyro_scale, kp):
        if not self.calibrated:
            return (0.0, 0.0, 0.0)

        now = time.monotonic()
        if self.last_time is None:
            self.last_time = now
            return self.get_relative_euler()

        dt = max(0.001, min(now - self.last_time, 0.05))
        self.last_time = now

        self.gyro_scale = max(0.1, gyro_scale)
        self.kp = max(0.0, kp)

        # Raw gyro values are converted to radians/second after bias removal.
        gx = (sample.gx - self.gyro_bias[0]) / self.gyro_scale
        gy = (sample.gy - self.gyro_bias[1]) / self.gyro_scale
        gz = (sample.gz - self.gyro_bias[2]) / self.gyro_scale
        omega = tuple(math.radians(v) for v in (gx, gy, gz))

        # Accelerometer gives the measured gravity direction in device space.
        acc_raw = (sample.ax, sample.ay, sample.az)
        acc_len = v_norm(acc_raw)

        error = (0.0, 0.0, 0.0)
        if acc_len > 1e-6:
            measured = v_normalize(acc_raw)
            # Predicted gravity direction expressed in device coordinates.
            predicted = q_rotate(q_conjugate(self.q), (0.0, 0.0, 1.0))
            error = v_cross(measured, predicted)

        corrected = tuple(
            omega[i] + self.kp * error[i] for i in range(3)
        )

        # q_dot = 0.5 * q * omega
        omega_q = (0.0, corrected[0], corrected[1], corrected[2])
        q_dot = q_multiply(self.q, omega_q)
        self.q = q_normalize(tuple(
            self.q[i] + 0.5 * q_dot[i] * dt for i in range(4)
        ))

        return self.get_relative_euler()

    def get_relative_euler(self):
        if not self.calibrated:
            return (0.0, 0.0, 0.0)
        relative = q_multiply(q_conjugate(self.reference), self.q)
        roll, pitch, yaw = q_to_euler_degrees(q_normalize(relative))
        return (wrap_degrees(roll), wrap_degrees(pitch), wrap_degrees(yaw))


# ------------------------------ BLE packets ---------------------------------

class TrikiSample:
    __slots__ = ("ax", "ay", "az", "gx", "gy", "gz", "button", "timestamp")

    def __init__(self, values, button=False):
        self.ax, self.ay, self.az, self.gx, self.gy, self.gz = values
        self.button = bool(button)
        self.timestamp = time.monotonic()


class PacketParser:
    """Parses 14-byte IMU frames; byte 1 is the physical-button state."""

    def __init__(self, output_queue):
        self.buffer = bytearray()
        self.output_queue = output_queue

    def feed(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 2:
            if self.buffer[0] == 0x22:
                if len(self.buffer) < 14:
                    return
                packet = bytes(self.buffer[:14])
                del self.buffer[:14]
                gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z = struct.unpack(
                    "<hhhhhh", packet[2:14]
                )
                sample_values = (
                    accel_x, accel_y, accel_z,
                    gyro_x, gyro_y, gyro_z,
                )
                self.output_queue.put(
                    TrikiSample(sample_values, button=bool(packet[1] & 0x01))
                )

            elif self.buffer[0] == 0x21:
                if len(self.buffer) < 5:
                    return
                del self.buffer[:5]

            else:
                del self.buffer[0]


# --------------------------------- App --------------------------------------

class TrikiMotionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Triki Motion Controller")
        self.root.geometry("1040x920")
        self.root.minsize(920, 820)

        self.ui_queue = queue.Queue()
        self.sample_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.ble_thread = None
        self.connected = False
        self.streaming = False
        self.selected_devices = {}
        self.device_list = []
        self.parser = PacketParser(self.sample_queue)
        self.frames_received = 0
        self.button_presses = 0
        self.last_button_state = False

        self.filter = OrientationFilter()
        self.calibration_samples = []
        self.calibrating = False
        self.calibration_target = 120

        self.pressed_keys = set()
        self.axis_active = {"roll": None, "pitch": None, "yaw": None}
        self._axis_current_key = {"roll": None, "pitch": None, "yaw": None}
        self._button_down = False
        self._button_press_time = None
        self._button_hold_active = False
        self._button_current_key = None

        self.status_var = tk.StringVar(value="Disconnected")
        self.device_var = tk.StringVar()
        self.device_info_var = tk.StringVar(value="No device selected")
        self.battery_var = tk.StringVar(value="Battery: —")
        self.raw_var = tk.StringVar(value="Waiting for sensor data…")
        self.orientation_var = tk.StringVar(value="Roll: 0.0°    Pitch: 0.0°    Yaw: 0.0°")
        self.calibration_var = tk.StringVar(value="Not calibrated")
        self.enabled_var = tk.BooleanVar(value=False)

        self.gyro_scale_var = tk.DoubleVar(value=14.286)
        self.correction_var = tk.DoubleVar(value=1.8)
        self.threshold_var = tk.DoubleVar(value=12.0)
        self.release_ratio_var = tk.DoubleVar(value=0.65)

        self.button_short_var = tk.StringVar(value="lmb")
        self.button_hold_var = tk.StringVar(value="rmb")
        self.button_hold_threshold_var = tk.DoubleVar(value=0.35)

        # Alternative profile: arrows for movement, E for tap, Escape for hold.
        self.active_profile = "primary"
        self.profile_var = tk.StringVar(value="Primary")
        self.profile_status_var = tk.StringVar(value="Active profile: Primary")
        self.double_click_window_var = tk.DoubleVar(value=0.32)
        self._pending_single_deadline = None
        self._double_click_candidate = False
        self._last_button_release = None
        self.alt_button_short_var = tk.StringVar(value="e")
        self.alt_button_hold_var = tk.StringVar(value="esc")
        self.alt_axis_vars = {
            "roll": {
                "negative": tk.StringVar(value="left"),
                "positive": tk.StringVar(value="right"),
                "invert": tk.BooleanVar(value=False),
                "enabled": tk.BooleanVar(value=True),
            },
            "pitch": {
                "negative": tk.StringVar(value="down"),
                "positive": tk.StringVar(value="up"),
                "invert": tk.BooleanVar(value=False),
                "enabled": tk.BooleanVar(value=True),
            },
            "yaw": {
                "negative": tk.StringVar(value="none"),
                "positive": tk.StringVar(value="none"),
                "invert": tk.BooleanVar(value=False),
                "enabled": tk.BooleanVar(value=False),
            },
        }

        self.axis_vars = {
            "roll": {
                "negative": tk.StringVar(value="a"),
                "positive": tk.StringVar(value="d"),
                "invert": tk.BooleanVar(value=False),
                "enabled": tk.BooleanVar(value=True),
            },
            "pitch": {
                "negative": tk.StringVar(value="s"),
                "positive": tk.StringVar(value="w"),
                "invert": tk.BooleanVar(value=False),
                "enabled": tk.BooleanVar(value=True),
            },
            "yaw": {
                "negative": tk.StringVar(value="q"),
                "positive": tk.StringVar(value="e"),
                "invert": tk.BooleanVar(value=False),
                "enabled": tk.BooleanVar(value=False),
            },
        }

        self._build_ui()
        self._load_config()
        self.root.after(10, self._poll_queues)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        if BleakClient is None or pydirectinput is None:
            missing = []
            if BleakClient is None:
                missing.append("bleak")
            if pydirectinput is None:
                missing.append("pydirectinput")
            messagebox.showwarning(
                "Missing dependencies",
                "Install dependencies with:\npy -m pip install -r requirements.txt\n\n"
                "Missing: " + ", ".join(missing)
            )

    def _build_ui(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure("Title.TLabel", font=("Segoe UI", 18, "bold"))
        style.configure("Status.TLabel", font=("Segoe UI", 10, "bold"))
        style.configure("Section.TLabelframe.Label", font=("Segoe UI", 10, "bold"))

        # Scrollable application body: the window can stay compact while all
        # configuration sections remain reachable on smaller displays.
        shell = ttk.Frame(self.root)
        shell.pack(fill="both", expand=True)

        canvas = tk.Canvas(shell, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(shell, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        outer = ttk.Frame(canvas, padding=14)
        window_id = canvas.create_window((0, 0), window=outer, anchor="nw")

        def _update_scroll_region(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _fit_content_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        outer.bind("<Configure>", _update_scroll_region)
        canvas.bind("<Configure>", _fit_content_width)

        # Windows and macOS use MouseWheel; Linux commonly uses Button-4/5.
        def _mousewheel(event):
            if getattr(event, "delta", 0):
                canvas.yview_scroll(int(-event.delta / 120), "units")
            elif getattr(event, "num", None) == 4:
                canvas.yview_scroll(-3, "units")
            elif getattr(event, "num", None) == 5:
                canvas.yview_scroll(3, "units")
            return "break"

        def _bind_wheel(_event=None):
            canvas.bind_all("<MouseWheel>", _mousewheel)
            canvas.bind_all("<Button-4>", _mousewheel)
            canvas.bind_all("<Button-5>", _mousewheel)

        def _unbind_wheel(_event=None):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")

        canvas.bind("<Enter>", _bind_wheel)
        canvas.bind("<Leave>", _unbind_wheel)

        top = ttk.Frame(outer)
        top.pack(fill="x", pady=(0, 10))
        ttk.Label(top, text="TRIKI MOTION CONTROLLER",
                  style="Title.TLabel").pack(side="left")
        ttk.Label(top, textvariable=self.status_var,
                  style="Status.TLabel").pack(side="right")

        connection = ttk.LabelFrame(
            outer, text="1. Bluetooth connection", style="Section.TLabelframe",
            padding=10
        )
        connection.pack(fill="x", pady=5)

        row = ttk.Frame(connection)
        row.pack(fill="x")
        self.device_combo = ttk.Combobox(
            row, textvariable=self.device_var, state="readonly", width=60
        )
        self.device_combo.pack(side="left", fill="x", expand=True, padx=(0, 8))
        ttk.Button(row, text="Scan", command=self.scan_devices).pack(side="left", padx=3)
        self.connect_button = ttk.Button(
            row, text="Connect", command=self.connect_selected
        )
        self.connect_button.pack(side="left", padx=3)
        self.disconnect_button = ttk.Button(
            row, text="Disconnect", command=self.disconnect, state="disabled"
        )
        self.disconnect_button.pack(side="left", padx=3)

        details = ttk.Frame(connection)
        details.pack(fill="x", pady=(8, 0))
        ttk.Label(details, textvariable=self.device_info_var).pack(side="left")
        ttk.Label(details, textvariable=self.battery_var).pack(side="right")

        calibration = ttk.LabelFrame(
            outer, text="2. Calibration", style="Section.TLabelframe", padding=10
        )
        calibration.pack(fill="x", pady=5)

        calrow = ttk.Frame(calibration)
        calrow.pack(fill="x")
        self.calibrate_button = ttk.Button(
            calrow, text="Calibrate neutral orientation",
            command=self.start_calibration, state="disabled"
        )
        self.calibrate_button.pack(side="left")
        ttk.Label(calrow, textvariable=self.calibration_var).pack(
            side="left", padx=12
        )
        ttk.Label(
            calibration,
            text="Place Triki in your preferred neutral position and keep it completely still "
                 "until calibration finishes (about 2 seconds).",
            wraplength=930
        ).pack(anchor="w", pady=(7, 0))

        mapping = ttk.LabelFrame(
            outer, text="3. Axis bindings", style="Section.TLabelframe", padding=10
        )
        mapping.pack(fill="x", pady=5)

        headers = ["Orientation component", "Negative direction", "Positive direction",
                   "Invert", "Enabled"]
        for col, header in enumerate(headers):
            ttk.Label(mapping, text=header, font=("Segoe UI", 9, "bold")).grid(
                row=0, column=col, padx=8, pady=(0, 7), sticky="w"
            )

        axis_labels = {
            "roll": "Roll (tilt left / right)",
            "pitch": "Pitch (tilt forward / back)",
            "yaw": "Yaw (rotate around vertical)",
        }
        for row_index, axis in enumerate(("roll", "pitch", "yaw"), start=1):
            vars_ = self.axis_vars[axis]
            ttk.Label(mapping, text=axis_labels[axis]).grid(
                row=row_index, column=0, padx=8, pady=5, sticky="w"
            )
            ttk.Combobox(
                mapping, textvariable=vars_["negative"], values=KEY_OPTIONS,
                state="readonly", width=12
            ).grid(row=row_index, column=1, padx=8, pady=5)
            ttk.Combobox(
                mapping, textvariable=vars_["positive"], values=KEY_OPTIONS,
                state="readonly", width=12
            ).grid(row=row_index, column=2, padx=8, pady=5)
            ttk.Checkbutton(mapping, variable=vars_["invert"]).grid(
                row=row_index, column=3, padx=8
            )
            ttk.Checkbutton(mapping, variable=vars_["enabled"]).grid(
                row=row_index, column=4, padx=8
            )

        ttk.Label(
            mapping,
            text="The keys are held while the corresponding orientation angle exceeds the threshold. "
                 "Choose 'none' to disable one direction.",
            wraplength=930
        ).grid(row=4, column=0, columnspan=5, sticky="w", padx=8, pady=(8, 0))

        button_box = ttk.LabelFrame(
            outer, text="4. Physical button bindings", style="Section.TLabelframe", padding=10
        )
        button_box.pack(fill="x", pady=5)
        ttk.Label(button_box, text="Short press (tap):").grid(
            row=0, column=0, sticky="w", padx=8, pady=4
        )
        ttk.Combobox(
            button_box, textvariable=self.button_short_var, values=KEY_OPTIONS,
            state="readonly", width=12
        ).grid(row=0, column=1, padx=8, pady=4)
        ttk.Label(button_box, text="Long hold:").grid(
            row=0, column=2, sticky="w", padx=8, pady=4
        )
        ttk.Combobox(
            button_box, textvariable=self.button_hold_var, values=KEY_OPTIONS,
            state="readonly", width=12
        ).grid(row=0, column=3, padx=8, pady=4)
        ttk.Label(button_box, text="Hold threshold (seconds):").grid(
            row=0, column=4, sticky="w", padx=8, pady=4
        )
        ttk.Spinbox(
            button_box, from_=0.15, to=2.0, increment=0.05, width=6,
            textvariable=self.button_hold_threshold_var
        ).grid(row=0, column=5, padx=8, pady=4)
        ttk.Label(
            button_box,
            text="A short press sends a key tap on release. A long hold presses its key after the threshold and keeps it down until you release Triki.",
            wraplength=930
        ).grid(row=1, column=0, columnspan=6, sticky="w", padx=8, pady=(5, 0))

        alt_box = ttk.LabelFrame(
            outer, text="5. Alternative profile (double-press Triki button to switch)",
            style="Section.TLabelframe", padding=10
        )
        alt_box.pack(fill="x", pady=5)
        ttk.Label(alt_box, text="Alternative movement:").grid(
            row=0, column=0, sticky="w", padx=8, pady=3
        )
        alt_labels = {"roll": "Roll", "pitch": "Pitch", "yaw": "Yaw"}
        for row_index, axis in enumerate(("roll", "pitch", "yaw"), start=1):
            vars_ = self.alt_axis_vars[axis]
            ttk.Label(alt_box, text=alt_labels[axis]).grid(
                row=row_index, column=0, padx=8, pady=3, sticky="w"
            )
            ttk.Combobox(
                alt_box, textvariable=vars_["negative"], values=KEY_OPTIONS,
                state="readonly", width=10
            ).grid(row=row_index, column=1, padx=5, pady=3)
            ttk.Label(alt_box, text="↔").grid(row=row_index, column=2)
            ttk.Combobox(
                alt_box, textvariable=vars_["positive"], values=KEY_OPTIONS,
                state="readonly", width=10
            ).grid(row=row_index, column=3, padx=5, pady=3)
            ttk.Checkbutton(alt_box, variable=vars_["invert"]).grid(
                row=row_index, column=4, padx=5
            )
            ttk.Checkbutton(alt_box, variable=vars_["enabled"]).grid(
                row=row_index, column=5, padx=5
            )
        ttk.Label(alt_box, text="Button tap:").grid(row=4, column=0, padx=8, sticky="w")
        ttk.Combobox(
            alt_box, textvariable=self.alt_button_short_var, values=KEY_OPTIONS,
            state="readonly", width=10
        ).grid(row=4, column=1, padx=5, pady=3)
        ttk.Label(alt_box, text="Button hold:").grid(row=4, column=2, padx=8, sticky="e")
        ttk.Combobox(
            alt_box, textvariable=self.alt_button_hold_var, values=KEY_OPTIONS,
            state="readonly", width=10
        ).grid(row=4, column=3, padx=5, pady=3)
        ttk.Label(alt_box, text="Double-press window (s):").grid(
            row=5, column=0, padx=8, sticky="w"
        )
        ttk.Spinbox(
            alt_box, from_=0.20, to=0.80, increment=0.02, width=7,
            textvariable=self.double_click_window_var
        ).grid(row=5, column=1, padx=5, pady=3, sticky="w")
        ttk.Label(alt_box, textvariable=self.profile_status_var).grid(
            row=5, column=2, columnspan=4, sticky="w", padx=8
        )

        settings = ttk.LabelFrame(
            outer, text="6. Sensor and response settings",
            style="Section.TLabelframe", padding=10
        )
        settings.pack(fill="x", pady=5)

        self._add_scale(settings, "Gyro scale (raw counts per °/s)",
                        self.gyro_scale_var, 2.0, 200.0, 0)
        self._add_scale(settings, "Accelerometer correction strength",
                        self.correction_var, 0.0, 5.0, 1)
        self._add_scale(settings, "Activation threshold (degrees)",
                        self.threshold_var, 2.0, 60.0, 2)
        self._add_scale(settings, "Release threshold ratio",
                        self.release_ratio_var, 0.2, 0.95, 3)

        live = ttk.LabelFrame(
            outer, text="Live orientation", style="Section.TLabelframe", padding=10
        )
        live.pack(fill="x", pady=5)
        ttk.Label(live, textvariable=self.orientation_var,
                  font=("Consolas", 13, "bold")).pack(anchor="w", pady=3)
        ttk.Label(live, textvariable=self.raw_var,
                  font=("Consolas", 9)).pack(anchor="w", pady=3)

        footer = ttk.Frame(outer)
        footer.pack(fill="x", pady=(8, 0))
        ttk.Checkbutton(
            footer, text="Enable keyboard / mouse output", variable=self.enabled_var,
            command=self._toggle_output
        ).pack(side="left")
        ttk.Button(footer, text="Save settings", command=self._save_config).pack(
            side="right", padx=4
        )
        ttk.Button(footer, text="Release all keys", command=self._release_all).pack(
            side="right", padx=4
        )

        note = (
            "Physical-button bindings work without orientation calibration. Calibrate only "
            "to use motion axes. Triki has a 6-axis IMU (accelerometer + gyroscope), not "
            "a magnetometer; yaw is relative to calibration and can slowly drift."
        )
        ttk.Label(outer, text=note, wraplength=980, foreground="#9a6b24").pack(
            anchor="w", pady=(10, 0)
        )

    def _add_scale(self, parent, label, variable, low, high, row):
        ttk.Label(parent, text=label).grid(
            row=row, column=0, sticky="w", padx=6, pady=3
        )
        ttk.Scale(
            parent, from_=low, to=high, variable=variable, orient="horizontal"
        ).grid(row=row, column=1, sticky="ew", padx=8, pady=3)
        value_label = ttk.Label(parent, width=8)
        value_label.grid(row=row, column=2, padx=6)
        def update_label(*_):
            value_label.configure(text=f"{variable.get():.2f}")
        variable.trace_add("write", update_label)
        update_label()
        parent.columnconfigure(1, weight=1)

    # ------------------------------ Bluetooth ------------------------------

    def scan_devices(self):
        if BleakScanner is None:
            messagebox.showerror("Missing Bleak", "Run: py -m pip install bleak")
            return
        self.status_var.set("Scanning for Triki devices…")
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def _scan_worker(self):
        async def scan():
            try:
                devices = await BleakScanner.discover(timeout=6.0)
                found = [
                    (d.name or "Unknown", d.address)
                    for d in devices
                    if d.name and "triki" in d.name.lower()
                ]
                self.ui_queue.put(("scan", found))
            except Exception as exc:
                self.ui_queue.put(("error", f"Bluetooth scan failed: {exc}"))
        asyncio.run(scan())

    def connect_selected(self):
        label = self.device_var.get()
        address = None
        for name, addr in self.device_list:
            if f"{name} — {addr}" == label:
                address = addr
                break
        if not address:
            messagebox.showwarning("Select device", "Scan and select a Triki device first.")
            return
        if BleakClient is None:
            return
        self.stop_event.clear()
        while True:
            try:
                self.sample_queue.get_nowait()
            except queue.Empty:
                break
        self.parser = PacketParser(self.sample_queue)
        self.ble_thread = threading.Thread(
            target=self._ble_worker, args=(address,), daemon=True
        )
        self.ble_thread.start()
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self.status_var.set("Connecting…")

    def _ble_worker(self, address):
        async def run():
            client = None
            try:
                client = BleakClient(address)
                await client.connect()
                if not client.is_connected:
                    raise RuntimeError("BLE connection was not established.")

                await client.start_notify(
                    TX_UUID, lambda sender, data: self.parser.feed(data)
                )
                await client.write_gatt_char(RX_UUID, WAKE_COMMAND, response=False)

                battery = "—"
                try:
                    raw_battery = await client.read_gatt_char(BATTERY_UUID)
                    if raw_battery:
                        battery = f"{int(raw_battery[0])}%"
                except Exception:
                    pass

                self.ui_queue.put(("connected", (address, battery)))

                while not self.stop_event.is_set() and client.is_connected:
                    await asyncio.sleep(0.05)

                if client.is_connected:
                    try:
                        await client.write_gatt_char(
                            RX_UUID, SLEEP_COMMAND, response=False
                        )
                        await asyncio.sleep(0.2)
                        await client.stop_notify(TX_UUID)
                    except Exception:
                        pass
                    await client.disconnect()

            except Exception as exc:
                self.ui_queue.put(("error", f"Connection error: {exc}"))
                if client is not None and client.is_connected:
                    try:
                        await client.disconnect()
                    except Exception:
                        pass
            finally:
                self.ui_queue.put(("disconnected", None))

        asyncio.run(run())

    def disconnect(self):
        self.stop_event.set()
        self._release_all()
        self.status_var.set("Disconnecting…")

    # ------------------------------ Calibration ----------------------------

    def start_calibration(self):
        if not self.connected:
            messagebox.showinfo("Not connected", "Connect to Triki first.")
            return
        self._release_all()
        self.calibration_samples = []
        self.calibrating = True
        # Do not allow an old calibration to drive keys while recalibrating.
        self.filter.calibrated = False
        self.enabled_var.set(False)
        self.calibration_var.set("Calibrating… keep the device still")
        self.status_var.set("Collecting calibration samples…")

    def _process_sample(self, sample):
        self.frames_received += 1
        if sample.button and not self.last_button_state:
            self.button_presses += 1
        self.last_button_state = sample.button
        self._handle_physical_button(sample.button, sample.timestamp)

        button_text = "DOWN" if sample.button else "UP"
        self.raw_var.set(
            f"Frames: {self.frames_received} | Button: {button_text} | Presses: {self.button_presses} | "
            f"ACC  X:{sample.ax:6d}  Y:{sample.ay:6d}  Z:{sample.az:6d}   |   "
            f"GYRO X:{sample.gx:6d}  Y:{sample.gy:6d}  Z:{sample.gz:6d}"
        )

        if self.calibrating:
            self.calibration_samples.append(sample)
            count = len(self.calibration_samples)
            self.calibration_var.set(f"Calibrating… {count}/{self.calibration_target}")
            if count >= self.calibration_target:
                try:
                    self.filter.calibrate(self.calibration_samples)
                    self.filter.gyro_scale = self.gyro_scale_var.get()
                    self.filter.kp = self.correction_var.get()
                    self.calibrating = False
                    self.calibration_var.set("Calibrated ✓ — enable keyboard output below")
                    self.status_var.set("Connected — enable keyboard output to send inputs")
                except Exception as exc:
                    self.calibrating = False
                    self.filter.calibrated = False
                    self.calibration_var.set(f"Calibration failed: {exc}")
                    self.status_var.set("Calibration failed")
                return
            # Do not process old orientation or emit keys during calibration.
            return

        angles = self.filter.update(
            sample, self.gyro_scale_var.get(), self.correction_var.get()
        )
        roll, pitch, yaw = angles

        self.orientation_var.set(
            f"Roll: {roll:+7.1f}°    Pitch: {pitch:+7.1f}°    Yaw: {yaw:+7.1f}°"
        )
        # Physical-button bindings are independent of orientation calibration.
        # Only axis output requires a calibrated orientation filter.
        if self.enabled_var.get() and self.filter.calibrated and not self.calibrating:
            self._apply_bindings({"roll": roll, "pitch": pitch, "yaw": yaw})
        else:
            self._release_axis_outputs()

    # ------------------------------ Key output -----------------------------

    def _active_axis_vars(self):
        return self.axis_vars if self.active_profile == "primary" else self.alt_axis_vars

    def _active_button_vars(self):
        if self.active_profile == "primary":
            return self.button_short_var.get(), self.button_hold_var.get()
        return self.alt_button_short_var.get(), self.alt_button_hold_var.get()

    def _switch_profile(self):
        self._release_axis_outputs()
        self.active_profile = "alternative" if self.active_profile == "primary" else "primary"
        label = "Alternative" if self.active_profile == "alternative" else "Primary"
        self.profile_var.set(label)
        self.profile_status_var.set(f"Active profile: {label}")
        # A profile switch must not leave a button/motion input held.
        self._button_current_key = None
        self._reconcile_keys()

    def _expire_pending_single(self, now=None):
        if self._pending_single_deadline is None:
            return
        now = time.monotonic() if now is None else now
        if now >= self._pending_single_deadline:
            self._pending_single_deadline = None
            short_key, _ = self._active_button_vars()
            if self.enabled_var.get() and short_key != "none" and short_key not in self.pressed_keys:
                self._tap_output(short_key)

    def _apply_bindings(self, angles):
        active_vars = self._active_axis_vars()
        threshold = self.threshold_var.get()
        release_threshold = threshold * self.release_ratio_var.get()

        for axis, value in angles.items():
            config = active_vars[axis]
            if not config["enabled"].get():
                self._set_axis_key(axis, None)
                continue

            if config["invert"].get():
                value = -value

            current = self.axis_active[axis]
            negative = config["negative"].get()
            positive = config["positive"].get()

            if current == "negative":
                active = "negative" if value < -release_threshold else None
            elif current == "positive":
                active = "positive" if value > release_threshold else None
            else:
                if value <= -threshold:
                    active = "negative"
                elif value >= threshold:
                    active = "positive"
                else:
                    active = None

            self._set_axis_key(axis, active)

            key = negative if active == "negative" else positive if active == "positive" else None
            if key == "none":
                key = None


    def _set_axis_key(self, axis, active):
        config = self._active_axis_vars()[axis]
        new_key = None
        if active == "negative":
            new_key = config["negative"].get()
        elif active == "positive":
            new_key = config["positive"].get()
        if new_key == "none":
            new_key = None

        self._axis_current_key[axis] = new_key
        self.axis_active[axis] = active

        # Reconcile the complete desired key set. This also handles two axes
        # intentionally sharing the same keyboard key.
        self._reconcile_keys()

    def _reconcile_keys(self):
        desired = {key for key in self._axis_current_key.values() if key}
        if self._button_current_key:
            desired.add(self._button_current_key)
        for key in list(self.pressed_keys - desired):
            self._release_key(key)
        for key in desired - self.pressed_keys:
            self._press_key(key)

    def _handle_physical_button(self, is_down, timestamp):
        """Debounced button state machine: tap, hold, or double-press profile switch."""
        self._expire_pending_single(timestamp)

        if is_down and not self._button_down:
            self._button_down = True
            self._button_press_time = timestamp if self.enabled_var.get() else None
            self._button_hold_active = False
            self._double_click_candidate = (
                self._pending_single_deadline is not None
                and timestamp <= self._pending_single_deadline
            )
            if self._double_click_candidate:
                self._pending_single_deadline = None
            return

        if is_down and self._button_down:
            if (self._button_press_time is not None and self.enabled_var.get()
                    and not self._button_hold_active):
                threshold = max(0.15, min(2.0, self.button_hold_threshold_var.get()))
                if timestamp - self._button_press_time >= threshold:
                    self._button_hold_active = True
                    self._double_click_candidate = False
                    self._pending_single_deadline = None
                    _, hold_key = self._active_button_vars()
                    if hold_key != "none":
                        self._button_current_key = hold_key
                        self._reconcile_keys()
            return

        if not is_down and self._button_down:
            was_enabled = self.enabled_var.get()
            had_press_start = self._button_press_time is not None
            was_long_hold = self._button_hold_active
            was_double = self._double_click_candidate
            self._button_down = False
            self._button_press_time = None
            self._button_hold_active = False
            self._double_click_candidate = False
            self._button_current_key = None
            self._reconcile_keys()

            if not was_enabled or not had_press_start or was_long_hold:
                return
            if was_double:
                self._pending_single_deadline = None
                self._switch_profile()
            else:
                window = max(0.20, min(0.80, self.double_click_window_var.get()))
                self._pending_single_deadline = timestamp + window

    @staticmethod
    def _mouse_button_name(key):
        return {"lmb": "left", "rmb": "right"}.get(key)

    def _press_key(self, key):
        if pydirectinput is None or key in self.pressed_keys:
            return
        try:
            mouse_button = self._mouse_button_name(key)
            if mouse_button:
                pydirectinput.mouseDown(button=mouse_button)
            else:
                pydirectinput.keyDown(key)
            self.pressed_keys.add(key)
        except Exception as exc:
            self.ui_queue.put(("error", f"Input error ({key} down): {exc}"))

    def _release_key(self, key):
        if pydirectinput is None or key not in self.pressed_keys:
            return
        try:
            mouse_button = self._mouse_button_name(key)
            if mouse_button:
                pydirectinput.mouseUp(button=mouse_button)
            else:
                pydirectinput.keyUp(key)
        except Exception as exc:
            self.ui_queue.put(("error", f"Input error ({key} up): {exc}"))
        finally:
            self.pressed_keys.discard(key)

    def _tap_output(self, key):
        if pydirectinput is None:
            return
        mouse_button = self._mouse_button_name(key)
        if mouse_button:
            # Explicit down/up with a minimum hold time is more reliable in games
            # that poll mouse state rather than Windows click messages.
            try:
                pydirectinput.mouseDown(button=mouse_button)
                self.pressed_keys.add(key)
                time.sleep(0.055)
            except Exception as exc:
                self.ui_queue.put(("error", f"Input error ({key} down): {exc}"))
            finally:
                try:
                    pydirectinput.mouseUp(button=mouse_button)
                except Exception as exc:
                    self.ui_queue.put(("error", f"Input error ({key} up): {exc}"))
                self.pressed_keys.discard(key)
        else:
            try:
                pydirectinput.press(key)
            except Exception as exc:
                self.ui_queue.put(("error", f"Input error ({key} tap): {exc}"))

    def _release_axis_outputs(self):
        """Release motion-axis outputs without resetting physical-button tracking."""
        self.axis_active = {"roll": None, "pitch": None, "yaw": None}
        self._axis_current_key = {"roll": None, "pitch": None, "yaw": None}
        self._reconcile_keys()

    def _release_all(self):
        for key in list(self.pressed_keys):
            self._release_key(key)
        self.pressed_keys.clear()
        self.axis_active = {"roll": None, "pitch": None, "yaw": None}
        self._axis_current_key = {"roll": None, "pitch": None, "yaw": None}
        self._button_down = False
        self._button_press_time = None
        self._button_hold_active = False
        self._button_current_key = None
        self._pending_single_deadline = None
        self._double_click_candidate = False

    def _toggle_output(self):
        if not self.enabled_var.get():
            self._release_all()
        elif not self.filter.calibrated:
            # The physical button can be used immediately; calibration is only
            # required for roll/pitch/yaw axis bindings.
            self.status_var.set("Button output enabled — calibrate to enable motion axes")

    # -------------------------------- UI loop -------------------------------

    def _poll_queues(self):
        try:
            while True:
                kind, payload = self.ui_queue.get_nowait()
                if kind == "scan":
                    self.device_list = payload
                    labels = [f"{name} — {addr}" for name, addr in payload]
                    self.device_combo["values"] = labels
                    if labels:
                        self.device_combo.current(0)
                        self.status_var.set(f"Found {len(labels)} Triki device(s)")
                    else:
                        self.status_var.set("No Triki devices found")
                elif kind == "connected":
                    address, battery = payload
                    self.connected = True
                    self.streaming = True
                    self.device_info_var.set(f"Connected: {address}")
                    self.battery_var.set(f"Battery: {battery}")
                    self.calibrate_button.configure(state="normal")
                    self.status_var.set("Connected — calibrate before using controls")
                elif kind == "disconnected":
                    self.connected = False
                    self.streaming = False
                    self.calibrate_button.configure(state="disabled")
                    self.connect_button.configure(state="normal")
                    self.disconnect_button.configure(state="disabled")
                    self.device_info_var.set("Disconnected")
                    self.status_var.set("Disconnected")
                    self.calibrating = False
                    self.filter.calibrated = False
                    self.enabled_var.set(False)
                    self._release_all()
                elif kind == "error":
                    self.status_var.set(payload)
                    if "Connection error" in payload:
                        self.connect_button.configure(state="normal")
                        self.disconnect_button.configure(state="disabled")
                self.ui_queue.task_done()
        except queue.Empty:
            pass

        # Drain every queued IMU frame to preserve gyro integration timing.
        processed = 0
        while processed < 100:
            try:
                sample = self.sample_queue.get_nowait()
            except queue.Empty:
                break
            self._process_sample(sample)
            processed += 1

        self._expire_pending_single(time.monotonic())
        self.root.after(10, self._poll_queues)

    # -------------------------------- Config -------------------------------

    def _save_config(self):
        data = {
            "gyro_scale": self.gyro_scale_var.get(),
            "correction": self.correction_var.get(),
            "threshold": self.threshold_var.get(),
            "release_ratio": self.release_ratio_var.get(),
            "button_short": self.button_short_var.get(),
            "button_hold": self.button_hold_var.get(),
            "button_hold_threshold": self.button_hold_threshold_var.get(),
            "double_click_window": self.double_click_window_var.get(),
            "active_profile": self.active_profile,
            "alt_button_short": self.alt_button_short_var.get(),
            "alt_button_hold": self.alt_button_hold_var.get(),
            "alt_axes": {
                axis: {
                    "negative": vars_["negative"].get(),
                    "positive": vars_["positive"].get(),
                    "invert": vars_["invert"].get(),
                    "enabled": vars_["enabled"].get(),
                }
                for axis, vars_ in self.alt_axis_vars.items()
            },
            "axes": {
                axis: {
                    "negative": vars_["negative"].get(),
                    "positive": vars_["positive"].get(),
                    "invert": vars_["invert"].get(),
                    "enabled": vars_["enabled"].get(),
                }
                for axis, vars_ in self.axis_vars.items()
            },
        }
        try:
            CONFIG_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.status_var.set("Settings saved")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))

    def _load_config(self):
        if not CONFIG_PATH.exists():
            return
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            saved_gyro_scale = data.get("gyro_scale", 14.286)
            # 16.4 was the previous app default; migrate it to the Triki's
            # LSM6DSL scale while preserving values the user actually tuned.
            if abs(float(saved_gyro_scale) - 16.4) < 0.001:
                saved_gyro_scale = 14.286
            self.gyro_scale_var.set(saved_gyro_scale)
            self.correction_var.set(data.get("correction", 1.8))
            self.threshold_var.set(data.get("threshold", 12.0))
            self.release_ratio_var.set(data.get("release_ratio", 0.65))
            self.button_short_var.set(data.get("button_short", "space"))
            self.button_hold_var.set(data.get("button_hold", "ctrl"))
            self.button_hold_threshold_var.set(data.get("button_hold_threshold", 0.35))
            self.double_click_window_var.set(data.get("double_click_window", 0.32))
            self.active_profile = data.get("active_profile", "primary")
            self.profile_status_var.set(
                "Active profile: " + ("Alternative" if self.active_profile == "alternative" else "Primary")
            )
            self.alt_button_short_var.set(data.get("alt_button_short", "e"))
            self.alt_button_hold_var.set(data.get("alt_button_hold", "esc"))
            for axis, values in data.get("alt_axes", {}).items():
                if axis not in self.alt_axis_vars:
                    continue
                for key in ("negative", "positive", "invert", "enabled"):
                    if key in values:
                        self.alt_axis_vars[axis][key].set(values[key])
            for axis, values in data.get("axes", {}).items():
                if axis not in self.axis_vars:
                    continue
                for key in ("negative", "positive", "invert", "enabled"):
                    if key in values:
                        self.axis_vars[axis][key].set(values[key])
        except Exception:
            pass

    def _on_close(self):
        self._save_config()
        self.enabled_var.set(False)
        self._release_all()
        self.stop_event.set()
        self.root.after(100, self.root.destroy)


def main():
    root = tk.Tk()
    TrikiMotionApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
