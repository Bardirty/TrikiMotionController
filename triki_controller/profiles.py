"""Profile defaults, migration and switching-sequence validation."""
import re

def default_profile(profile_id, alternative=False):
    if alternative:
        axes = {
            "roll": {"negative": "left", "positive": "right", "invert": False, "enabled": True},
            "pitch": {"negative": "down", "positive": "up", "invert": False, "enabled": True},
            "yaw": {"negative": "none", "positive": "none", "invert": False, "enabled": False},
        }
        short, hold = "e", "esc"
    else:
        axes = {
            "roll": {"negative": "a", "positive": "d", "invert": False, "enabled": True},
            "pitch": {"negative": "s", "positive": "w", "invert": False, "enabled": True},
            "yaw": {"negative": "q", "positive": "e", "invert": False, "enabled": False},
        }
        short, hold = "lmb", "rmb"
    return {"name": str(profile_id), "axes": axes, "button_short": short, "button_hold": hold}

def parse_sequence(raw, profiles):
    raw = str(raw).strip()
    if not raw:
        raise ValueError("Switching sequence is empty.")
    if any(ch in raw for ch in ",; "):
        sequence = [part.strip() for part in re.split(r"[,;\s]+", raw) if part.strip()]
    else:
        sequence = list(raw)
    if not sequence:
        raise ValueError("Switching sequence is empty.")
    missing = [item for item in sequence if item not in profiles]
    if missing:
        raise ValueError("Sequence references missing profile(s): " + ", ".join(missing))
    return sequence

def migrate_legacy_config(data):
    first = default_profile("1")
    first["button_short"] = data.get("button_short", "lmb")
    first["button_hold"] = data.get("button_hold", "rmb")
    for axis, values in data.get("axes", {}).items():
        if axis in first["axes"] and isinstance(values, dict):
            first["axes"][axis].update(values)
    second = default_profile("2", alternative=True)
    second["button_short"] = data.get("alt_button_short", "e")
    second["button_hold"] = data.get("alt_button_hold", "esc")
    for axis, values in data.get("alt_axes", {}).items():
        if axis in second["axes"] and isinstance(values, dict):
            second["axes"][axis].update(values)
    return {"1": first, "2": second}
