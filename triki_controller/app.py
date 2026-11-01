"""Tkinter application: UI composition and coordination of controller services."""
import asyncio
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

try:
    import pydirectinput
    pydirectinput.PAUSE = 0
except ImportError:
    pydirectinput = None

from .constants import (APP_DIR, CONFIG_PATH, KEY_OPTIONS)
from .orientation import OrientationFilter
from .protocol import PacketParser
from .bluetooth import discover_devices, is_available, run_connection
from .profiles import default_profile, parse_sequence, migrate_legacy_config
from .config_store import load_config, save_config

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
        # Button gesture state must exist before the first Tk after() tick.
        # A single press is delayed briefly to distinguish it from a double press.
        self._pending_single_deadline = None
        self._double_click_candidate = False

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
        self.double_click_window_var = tk.DoubleVar(value=0.32)

        # User profiles are stored as snapshots; the visible controls edit the selected profile.
        self.active_profile = "1"
        self.profile_sequence_var = tk.StringVar(value="12")
        self.profile_status_var = tk.StringVar(value="Active profile: 1")
        self.profile_sequence_index = 0
        self.profiles = {}
        self.profile_combo_var = tk.StringVar(value="1")
        self.profile_combo = None

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

        self.profiles = {
            "1": default_profile("1"),
            "2": default_profile("2", alternative=True),
        }
        self._build_ui()
        self._load_config()
        self._refresh_profile_combo()
        self._load_profile_into_editor(self.active_profile)
        self.root.after(10, self._poll_queues)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        if not is_available() or pydirectinput is None:
            missing = []
            if not is_available():
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

        profile_box = ttk.LabelFrame(
            outer, text="5. Profiles and switching sequence",
            style="Section.TLabelframe", padding=10
        )
        profile_box.pack(fill="x", pady=5)
        profile_row = ttk.Frame(profile_box)
        profile_row.pack(fill="x", pady=(0, 6))
        ttk.Label(profile_row, text="Edit profile:").pack(side="left", padx=(0, 6))
        self.profile_combo = ttk.Combobox(
            profile_row, textvariable=self.profile_combo_var, state="readonly", width=16
        )
        self.profile_combo.pack(side="left", padx=4)
        self.profile_combo.bind("<<ComboboxSelected>>", self._on_profile_selected)
        ttk.Button(profile_row, text="Add profile", command=self._add_profile).pack(side="left", padx=4)
        ttk.Button(profile_row, text="Delete profile", command=self._delete_profile).pack(side="left", padx=4)
        ttk.Label(profile_box, text="Switching sequence (profile IDs):").pack(anchor="w")
        sequence_row = ttk.Frame(profile_box)
        sequence_row.pack(fill="x", pady=(3, 0))
        ttk.Entry(sequence_row, textvariable=self.profile_sequence_var, width=24).pack(side="left")
        ttk.Label(sequence_row, text="Example: 123 or 1213 — double-press Triki button advances one step").pack(
            side="left", padx=8
        )
        ttk.Label(sequence_row, text="Double-press interval (s):").pack(side="left", padx=(12, 4))
        ttk.Spinbox(
            sequence_row, from_=0.20, to=0.80, increment=0.02, width=6,
            textvariable=self.double_click_window_var
        ).pack(side="left")
        ttk.Label(profile_box, textvariable=self.profile_status_var).pack(anchor="w", pady=(6, 0))

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
        self.status_var.set("Scanning for Triki devices…")
        threading.Thread(target=self._scan_worker, daemon=True).start()

    def _scan_worker(self):
        async def scan():
            try:
                # The source implementation used a 6-second scan and filtered
                # directly on the advertised device name containing "triki".
                found = await discover_devices(timeout=6.0)
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
        if not is_available():
            messagebox.showerror(
                "Missing Bleak",
                "Install dependencies with: py -m pip install -r requirements.txt",
            )
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
        # Bluetooth transport is isolated in bluetooth.py; app.py only owns
        # the worker thread and receives UI events through ui_queue.
        asyncio.run(
            run_connection(
                address=address,
                parser=self.parser,
                stop_event=self.stop_event,
                ui_queue=self.ui_queue,
            )
        )

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
        return self.axis_vars

    def _active_button_vars(self):
        return self.button_short_var.get(), self.button_hold_var.get()

    def _snapshot_editor(self):
        return {
            "axes": {
                axis: {key: vars_[key].get() for key in ("negative", "positive", "invert", "enabled")}
                for axis, vars_ in self.axis_vars.items()
            },
            "button_short": self.button_short_var.get(),
            "button_hold": self.button_hold_var.get(),
        }

    def _load_profile_into_editor(self, profile_id):
        profile = self.profiles[profile_id]
        self.active_profile = profile_id
        for axis, values in profile.get("axes", {}).items():
            if axis not in self.axis_vars:
                continue
            for key in ("negative", "positive", "invert", "enabled"):
                if key in values:
                    self.axis_vars[axis][key].set(values[key])
        self.button_short_var.set(profile.get("button_short", "lmb"))
        self.button_hold_var.set(profile.get("button_hold", "rmb"))
        self.profile_combo_var.set(profile_id)
        self.profile_status_var.set(f"Active profile: {profile_id}")

    def _save_editor_to_profile(self):
        if self.active_profile in self.profiles:
            self.profiles[self.active_profile].update(self._snapshot_editor())

    def _refresh_profile_combo(self):
        if self.profile_combo is not None:
            self.profile_combo["values"] = sorted(self.profiles, key=lambda x: int(x))
        if self.active_profile in self.profiles:
            self.profile_combo_var.set(self.active_profile)

    def _on_profile_selected(self, _event=None):
        selected = self.profile_combo_var.get()
        if selected not in self.profiles or selected == self.active_profile:
            return
        self._save_editor_to_profile()
        self._release_all()
        self._load_profile_into_editor(selected)
        try:
            sequence = self._parse_profile_sequence()
            self.profile_sequence_index = sequence.index(selected)
        except ValueError:
            self.profile_sequence_index = 0

    def _add_profile(self):
        available = [str(i) for i in range(1, 10) if str(i) not in self.profiles]
        if not available:
            messagebox.showinfo("Profile limit", "This version supports up to 9 profiles (IDs 1–9).")
            return
        profile_id = simpledialog.askstring(
            "Add profile", "Enter a free profile ID (1–9):", parent=self.root
        )
        if profile_id is None:
            return
        profile_id = profile_id.strip()
        if profile_id not in available:
            messagebox.showwarning("Invalid profile ID", "Choose one of the free IDs: " + ", ".join(available))
            return
        self._save_editor_to_profile()
        # New profiles start as a copy of the currently edited profile.
        self.profiles[profile_id] = {**self._snapshot_editor(), "name": profile_id}
        current_sequence = self.profile_sequence_var.get().strip()
        self.profile_sequence_var.set(current_sequence + profile_id if current_sequence.isdigit() else
                                       (current_sequence + "," + profile_id if current_sequence else profile_id))
        self._refresh_profile_combo()
        self._load_profile_into_editor(profile_id)
        self.status_var.set(f"Profile {profile_id} created")

    def _delete_profile(self):
        if len(self.profiles) <= 1:
            messagebox.showinfo("Cannot delete", "At least one profile must remain.")
            return
        profile_id = self.active_profile
        if not messagebox.askyesno("Delete profile", f"Delete profile {profile_id}?"):
            return
        del self.profiles[profile_id]
        sequence = "".join(ch for ch in self.profile_sequence_var.get() if ch != profile_id)
        self.profile_sequence_var.set(sequence or sorted(self.profiles)[0])
        next_id = sorted(self.profiles, key=lambda x: int(x))[0]
        self._release_all()
        self._refresh_profile_combo()
        self._load_profile_into_editor(next_id)

    def _parse_profile_sequence(self):
        return parse_sequence(self.profile_sequence_var.get(), self.profiles)

    def _switch_profile(self):
        try:
            sequence = self._parse_profile_sequence()
        except ValueError as exc:
            self.status_var.set(str(exc))
            return
        self._save_editor_to_profile()
        if self.active_profile in sequence:
            # Keep the current queue position so repeated IDs (e.g. 1213)
            # are meaningful rather than collapsing to unique profile IDs.
            positions = [i for i, item in enumerate(sequence) if item == self.active_profile]
            next_position = next((i for i in positions if i >= self.profile_sequence_index), positions[0])
            self.profile_sequence_index = (next_position + 1) % len(sequence)
        else:
            self.profile_sequence_index = 0
        next_id = sequence[self.profile_sequence_index]
        self._release_all()
        self._load_profile_into_editor(next_id)
        self.status_var.set(f"Switched to profile {next_id}")

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
                        self.status_var.set("No Triki device found. Check that Triki is awake and scan again.")
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
        self._save_editor_to_profile()
        data = {
            "gyro_scale": self.gyro_scale_var.get(),
            "correction": self.correction_var.get(),
            "threshold": self.threshold_var.get(),
            "release_ratio": self.release_ratio_var.get(),
            "button_hold_threshold": self.button_hold_threshold_var.get(),
            "double_click_window": self.double_click_window_var.get(),
            "active_profile": self.active_profile,
            "profile_sequence": self.profile_sequence_var.get(),
            "profiles": self.profiles,
        }
        try:
            save_config(CONFIG_PATH, data)
            self.status_var.set("Settings saved")
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))

    def _load_config(self):
        try:
            data = load_config(CONFIG_PATH)
            if data is None:
                return
            self.gyro_scale_var.set(data["gyro_scale"])
            self.correction_var.set(data["correction"])
            self.threshold_var.set(data["threshold"])
            self.release_ratio_var.set(data["release_ratio"])
            self.button_hold_threshold_var.set(data["button_hold_threshold"])
            self.double_click_window_var.set(data["double_click_window"])
            self.profiles = data["profiles"]
            self.profile_sequence_var.set(data["profile_sequence"])
            requested = data["active_profile"]
            self.active_profile = requested if requested in self.profiles else sorted(self.profiles, key=lambda x: int(x))[0]
            self.profile_sequence_index = 0
        except Exception as exc:
            self.status_var.set(f"Config load failed: {exc}")

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