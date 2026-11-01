"""Application constants and supported output bindings."""
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = APP_DIR / "triki_motion_config.json"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
BATTERY_UUID = "00002a19-0000-1000-8000-00805f9b34fb"
WAKE_COMMAND = bytes.fromhex("20 10 00 D0 07 34 00 03")
SLEEP_COMMAND = bytes.fromhex("20 00 00 00 00 00 00")
KEY_OPTIONS = [
    "none", "w", "a", "s", "d", "up", "down", "left", "right",
    "space", "shift", "ctrl", "alt", "q", "e", "r", "f", "z", "x",
    "c", "1", "2", "3", "4", "5", "esc", "lmb", "rmb"
]
