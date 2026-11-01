"""Triki BLE frame parser and normalized sensor sample."""
import queue
import struct
import time

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
