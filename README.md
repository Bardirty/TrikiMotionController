# Triki Motion Controller

A configurable motion-to-input application for the **Żabka Triki** BLE controller. It turns the controller's IMU data into keyboard and mouse actions, allowing Triki to be used as an unconventional game controller on a PC.

The project started as an experiment: using a small bottle-cap-shaped motion controller to play games that were never designed for this kind of input.

## Features

- **Bluetooth Low Energy connection** — scan for Triki devices, select a device, connect and stream sensor notifications.
- **Motion-based input** — map Roll, Pitch and Yaw directions to keyboard keys or mouse buttons.
- **Per-axis configuration** — enable or disable an axis, choose its negative and positive bindings, and invert its direction.
- **Configurable profiles** — create and edit up to nine profiles, each with its own axis and physical-button bindings.
- **Profile switching sequence** — define a sequence such as `123` or `1213`; a double press of the Triki button advances to the next entry.
- **Physical-button actions** — assign separate actions to a short press and a long hold.
- **Motion calibration** — capture a neutral orientation and gyro bias before using motion bindings.
- **Live diagnostics** — view the current orientation, raw accelerometer and gyroscope values, received frame count, button state and battery level (when available).
- **Persistent settings** — application settings and profiles are stored in `triki_motion_config.json`.
- **Scrollable interface** — the full configuration UI remains accessible in a smaller window.

## How it works

1. Triki connects to the application over **Bluetooth Low Energy (BLE)**.
2. The application receives IMU notifications and decodes the controller's motion frames.
3. Accelerometer and gyroscope measurements are passed to a 6-axis orientation filter.
4. The resulting Roll, Pitch and Yaw angles are compared with configurable activation and release thresholds.
5. Active directions are translated into keyboard or mouse input.

The orientation filter integrates gyroscope rotation and uses gravity from the accelerometer to correct Roll and Pitch drift. Triki's data stream does not provide a magnetometer, so **Yaw is relative to the calibration position and may drift over time**.

## Core / protocol foundation

This application uses the BLE protocol and communication flow documented by **Wojciech B. (Wojtekb30)** in:

[**unofficial-triki-api-py**](https://github.com/Wojtekb30/unofficial-triki-api-py)

That project provides an unofficial Python/Bleak API for streaming accelerometer and gyroscope data from Żabka Triki, and documents the Nordic UART Service characteristics and the controller's wake/sleep commands.

Triki Motion Controller is a separate desktop application. Its current modular implementation uses **Bleak directly** for device discovery and connection; it does not require importing `TrikiPy` from the original repository as a Python package. The protocol reference and communication sequence are credited to the project above. The original `unofficial-triki-api-py` repository is distributed under the MIT License; see its [license file](https://github.com/Wojtekb30/unofficial-triki-api-py/blob/main/LICENSE) for the original terms and copyright notice.

## Requirements

- Python **3.9 or newer**
- A Bluetooth adapter with Bluetooth Low Energy support
- Windows is the primary target platform
- A Żabka Triki device

Python dependencies:

- [`bleak`](https://github.com/hbldh/bleak) — Bluetooth Low Energy communication
- [`pydirectinput`](https://github.com/learncodebygaming/pydirectinput) — keyboard and mouse input simulation
- `tkinter` — graphical interface (normally included with the standard Windows Python installer)

## Installation

Clone or download this repository, then open a terminal in the project directory.

Create and activate a virtual environment (recommended):

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
```

Install the dependencies:

```powershell
py -m pip install -r requirements.txt
```

## Run

Start the application from the repository root:

```powershell
py TrikiMotionController.py
```

### First connection

1. Make sure Bluetooth is enabled on your PC and Triki is nearby.
2. If Triki is currently connected to another device (for example, a phone), disconnect it there first.
3. Press **Scan** and select the Triki device from the list.
4. Press **Connect** and wait for the connection status to update.
5. Keep Triki still in the desired neutral position and press **Calibrate neutral orientation**.
6. Enable **keyboard / mouse output** when you are ready to send input to other applications.

The current discovery implementation filters advertisements by device name: the name must contain `triki` (case-insensitive). Devices without a name or with a different advertised name are not shown.

## Profiles and switching sequences

Each profile stores its own:

- Roll, Pitch and Yaw bindings;
- per-axis inversion and enabled state;
- physical-button short-press action;
- physical-button long-hold action.

Up to nine profiles can be configured. The switching sequence determines the order in which profiles become active when you double-press the Triki button.

Examples:

| Sequence | Switching order |
| --- | --- |
| `123` | 1 → 2 → 3 → 1 → ... |
| `1213` | 1 → 2 → 1 → 3 → 1 → ... |
| `213` | 2 → 1 → 3 → 2 → ... |

You can also separate profile IDs with commas or spaces (for example, `1, 2, 1, 3`). Every ID in the sequence must refer to an existing profile.

A regular short press and a long hold remain available for the actions configured in the active profile. The double-press interval can be adjusted in the application.

## Project structure

```text
TrikiMotionController/
├── TrikiMotionController.py       # Application entry point
├── requirements.txt               # Python dependencies
├── triki_motion_config.json       # User settings (created/updated at runtime)
└── triki_controller/
    ├── app.py                      # Tkinter UI and application coordination
    ├── bluetooth.py                # BLE discovery and connection lifecycle
    ├── protocol.py                 # Triki packet parser and IMU sample model
    ├── orientation.py              # IMU calibration and orientation filter
    ├── math3d.py                   # Vector and quaternion mathematics
    ├── profiles.py                 # Profile defaults, migration and sequence validation
    ├── config_store.py             # Configuration persistence
    └── constants.py                # BLE UUIDs, commands and shared constants
```

## Building a Windows executable

Install PyInstaller in the project environment:

```powershell
py -m pip install pyinstaller
```

Build from the repository root:

```powershell
py -m PyInstaller --noconfirm --clean --windowed --name TrikiMotionController --collect-all bleak TrikiMotionController.py
```

The output will be placed in `dist/TrikiMotionController/`. A PyInstaller `onedir` build is generally easier to troubleshoot for a BLE desktop application: distribute the complete output directory, not only the `.exe` file. Test the packaged build on the target Windows machine, especially BLE discovery and connection.

## Troubleshooting

- **No Triki device appears:** check that Bluetooth is enabled, Triki is awake and not connected to another host. The current scanner only lists devices whose advertised name contains `triki`.
- **Connection fails:** try disconnecting Triki from a phone or another computer, then scan again.
- **Motion input does not work:** confirm that calibration completed and keyboard/mouse output is enabled. Check the selected profile, axis enable checkboxes and activation threshold.
- **Yaw slowly changes while the device is still:** this is an expected limitation of 6-axis orientation tracking without a magnetometer. Recalibrate when needed.
- **Keys or mouse buttons remain held:** use **Release all keys** in the application.

## Acknowledgements

- **[Wojtekb30 / unofficial-triki-api-py](https://github.com/Wojtekb30/unofficial-triki-api-py)** — for the unofficial Python/Bleak API, BLE protocol research and communication reference used as the foundation for Triki connectivity.
- **[Bleak](https://github.com/hbldh/bleak)** — cross-platform BLE library for Python.
- **[PyDirectInput](https://github.com/learncodebygaming/pydirectinput)** — keyboard and mouse input simulation.

## Disclaimer

This is an unofficial, independent project. It is not affiliated with, endorsed by, or sponsored by Żabka Polska, Caps Apps or their partners. “Żabka” and “Triki” are trademarks of their respective owners. Use this software at your own risk.
