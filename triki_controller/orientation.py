"""IMU orientation fusion and calibration."""
import math
import time
from .math3d import (v_norm, v_normalize, v_cross, q_normalize, q_conjugate, q_multiply, q_rotate, q_from_two_vectors, q_to_euler_degrees, wrap_degrees)

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
