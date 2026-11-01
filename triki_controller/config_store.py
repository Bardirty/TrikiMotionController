"""JSON persistence for controller-wide settings and user profiles."""
import json
from pathlib import Path
from .profiles import default_profile, migrate_legacy_config

def load_config(path):
    path = Path(path)
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    saved_gyro_scale = data.get("gyro_scale", 14.286)
    if abs(float(saved_gyro_scale) - 16.4) < 0.001:
        saved_gyro_scale = 14.286
    profiles = data.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        profiles = migrate_legacy_config(data)
    return {
        "gyro_scale": saved_gyro_scale,
        "correction": data.get("correction", 1.8),
        "threshold": data.get("threshold", 12.0),
        "release_ratio": data.get("release_ratio", 0.65),
        "button_hold_threshold": data.get("button_hold_threshold", 0.35),
        "double_click_window": data.get("double_click_window", 0.32),
        "active_profile": str(data.get("active_profile", "1")),
        "profile_sequence": data.get("profile_sequence", "12"),
        "profiles": profiles,
    }

def save_config(path, data):
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
