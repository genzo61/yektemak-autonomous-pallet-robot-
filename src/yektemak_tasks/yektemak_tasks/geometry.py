"""Kucuk 2D geometri yardimcilari (ROS bagimsiz, test edilebilir)."""

import math


def normalize_angle(a):
    """Aciyi [-pi, pi] araligina getirir."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    """(x, y, z, w) dondurur."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def transform_point(pose, px, py):
    """pose=(x, y, yaw) cercevesindeki (px, py) noktasini ust cerceveye tasir."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * px - s * py, y + s * px + c * py


def to_local(pose, wx, wy):
    """Ust cercevedeki (wx, wy) noktasini pose=(x, y, yaw) cercevesine tasir."""
    x, y, yaw = pose
    dx, dy = wx - x, wy - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def clamp(v, lo, hi):
    return max(lo, min(hi, v))
