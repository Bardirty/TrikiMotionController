"""Quaternion and vector math helpers."""
import math

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
