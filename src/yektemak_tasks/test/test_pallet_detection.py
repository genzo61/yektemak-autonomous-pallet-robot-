"""Palet tespiti icin sentetik scan testleri.

Basit bir 2D isin izleyici ile paletin iki rayini (kutu) ve arka duvari
cizer, LiDAR olcumu uretir ve algoritmanin paleti dogru buldugunu kontrol eder.
"""

import math
import random

from yektemak_tasks.pallet_detection import detect_pallet

ANGLE_MIN = -math.radians(100)
N = 401
INC = (2 * math.radians(100)) / (N - 1)


def ray_box(ox, oy, dx, dy, cx, cy, yaw, sx, sy):
    """Isin ile donuk kutu kesisimi; mesafe ya da None."""
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * (ox - cx) - s * (oy - cy), s * (ox - cx) + c * (oy - cy)
    ldx, ldy = c * dx - s * dy, s * dx + c * dy
    tmin, tmax = -1e9, 1e9
    for o, d, h in ((lx, ldx, sx / 2), (ly, ldy, sy / 2)):
        if abs(d) < 1e-12:
            if abs(o) > h:
                return None
            continue
        t1, t2 = (-h - o) / d, (h - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0):
        return None
    return tmin if tmin > 0 else None


def make_scan(px, py, pyaw, noise=0.005, wall_x=None, extra_boxes=()):
    """LiDAR (0,0,0)'da. Palet merkezi (px,py), giris yonu pyaw."""
    boxes = []
    for side in (-1, 1):
        cx = px + math.cos(pyaw + math.pi / 2) * 0.34 * side
        cy = py + math.sin(pyaw + math.pi / 2) * 0.34 * side
        boxes.append((cx, cy, pyaw, 1.10, 0.08))
    boxes += list(extra_boxes)
    rng = random.Random(1)
    ranges = []
    for i in range(N):
        a = ANGLE_MIN + i * INC
        dx, dy = math.cos(a), math.sin(a)
        best = float("inf")
        for b in boxes:
            t = ray_box(0, 0, dx, dy, *b)
            if t is not None and t < best:
                best = t
        if wall_x is not None and dx > 1e-6:
            best = min(best, wall_x / dx)
        if math.isfinite(best):
            best += rng.gauss(0, noise)
        ranges.append(best)
    return ranges


def check(px, py, pyaw, **kw):
    det = detect_pallet(make_scan(px, py, pyaw, **kw), ANGLE_MIN, INC)
    assert det is not None, "palet bulunamadi"
    assert abs(det.x - px) < 0.03, (det.x, px)
    assert abs(det.y - py) < 0.03, (det.y, py)
    assert abs(math.atan2(math.sin(det.yaw - pyaw), math.cos(det.yaw - pyaw))) < math.radians(3)
    return det


def test_straight_ahead():
    check(1.35, 0.0, 0.0)


def test_lateral_offset():
    check(1.40, 0.15, 0.0)
    check(1.40, -0.20, 0.0)


def test_rotated_pallet():
    check(1.45, 0.05, math.radians(10))
    check(1.45, -0.05, math.radians(-15))


def test_with_back_wall_and_far():
    check(2.0, 0.1, math.radians(5), wall_x=6.0)


def test_no_pallet():
    ranges = make_scan(50, 50, 0.0, wall_x=4.0)  # palet menzil disinda
    assert detect_pallet(ranges, ANGLE_MIN, INC) is None


def test_ignores_two_boxes_with_wrong_spacing():
    far = [(1.5, 0.6, 0.0, 0.3, 0.3), (1.5, -0.6, 0.0, 0.3, 0.3)]
    ranges = make_scan(50, 50, 0.0, extra_boxes=far)
    assert detect_pallet(ranges, ANGLE_MIN, INC) is None
