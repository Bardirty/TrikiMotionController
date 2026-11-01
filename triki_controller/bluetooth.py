"""BLE discovery and connection lifecycle for the Triki controller.

The discovery and connection sequence intentionally mirrors the known-good
monolithic implementation supplied by the user: scan with BleakScanner.discover,
keep devices whose advertised name contains 'triki', connect by address, subscribe
to the Nordic UART TX characteristic, then send the wake command.
"""
from __future__ import annotations

import asyncio
import queue
import threading
from typing import Any

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # Friendly error is raised by the public functions below.
    BleakClient = None
    BleakScanner = None

from .constants import (
    BATTERY_UUID,
    RX_UUID,
    SLEEP_COMMAND,
    TX_UUID,
    WAKE_COMMAND,
)


def is_available() -> bool:
    """Return whether Bleak is installed."""
    return BleakClient is not None and BleakScanner is not None


async def discover_devices(timeout: float = 6.0) -> list[tuple[str, str]]:
    """Find Triki devices using the same name filter as the known-good app."""
    if BleakScanner is None:
        raise RuntimeError("Bleak is not installed. Run: py -m pip install -r requirements.txt")

    devices = await BleakScanner.discover(timeout=timeout)
    found: list[tuple[str, str]] = []
    seen_addresses: set[str] = set()

    for device in devices:
        name = getattr(device, "name", None)
        address = getattr(device, "address", None)
        if not name or not address:
            continue
        if "triki" not in name.lower():
            continue
        if address in seen_addresses:
            continue
        seen_addresses.add(address)
        found.append((name, address))

    return found


async def run_connection(
    address: str,
    parser: Any,
    stop_event: threading.Event,
    ui_queue: "queue.Queue",
) -> None:
    """Connect, start notifications, wake Triki and stream until stopped."""
    if BleakClient is None:
        raise RuntimeError("Bleak is not installed. Run: py -m pip install -r requirements.txt")

    client = None
    notify_started = False
    try:
        # Keep the same direct-address connection path as the working source.
        client = BleakClient(address)
        await client.connect()
        if not client.is_connected:
            raise RuntimeError("BLE connection was not established.")

        await client.start_notify(
            TX_UUID,
            lambda sender, data: parser.feed(data),
        )
        notify_started = True

        await client.write_gatt_char(RX_UUID, WAKE_COMMAND, response=False)

        battery = "—"
        try:
            raw_battery = await client.read_gatt_char(BATTERY_UUID)
            if raw_battery:
                battery = f"{int(raw_battery[0])}%"
        except Exception:
            # Battery characteristic is optional on some firmware versions.
            pass

        ui_queue.put(("connected", (address, battery)))

        while not stop_event.is_set() and client.is_connected:
            await asyncio.sleep(0.05)

    except Exception as exc:
        ui_queue.put(("error", f"Connection error: {exc}"))

    finally:
        if client is not None:
            try:
                if client.is_connected:
                    try:
                        await client.write_gatt_char(
                            RX_UUID, SLEEP_COMMAND, response=False
                        )
                        await asyncio.sleep(0.2)
                    except Exception:
                        pass
                    if notify_started:
                        try:
                            await client.stop_notify(TX_UUID)
                        except Exception:
                            pass
                    await client.disconnect()
            except Exception:
                # Cleanup must not prevent the UI from being released.
                pass
        ui_queue.put(("disconnected", None))
