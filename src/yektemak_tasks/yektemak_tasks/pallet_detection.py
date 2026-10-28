"""LiDAR ile palet tespiti (ROS bagimsiz, birim testli).

Fikir
-----
Robotun on LiDAR'i yerden ~11 cm yukseklikte. Bu yukseklikte paletin
sadece iki tasiyici rayi gorunur (tabla 23 cm'den basliyor). Paletin
"agzina" bakan bir robot icin scan'de:

    ray ucu (8 cm)   bosluk (60 cm, arkasi uzakta/bos)   ray ucu (8 cm)

gorulur. Algoritma:
  1. Gecerli olcumleri x-y noktalarina cevir (LiDAR cercevesi).
  2. Ardisik noktalari mesafe atlamasina gore kumelere ayir.
  3. Her kumenin LiDAR'a en yakin kismini "ray ucu" kabul et (yandan
     bakildiginda rayin uzun yan yuzu de gorunur; uc yuzu en yakin kisimdir).
  4. Merkezleri arasi ~0.68 m olan iki kume ciftini ara.
  5. Ciftin arasinda, paletin acikligi icinde nokta olmadigini dogrula.
  6. Palet on kenarinin ortasi + normal yon -> palet merkezi ve yonu.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence


@dataclass
class PalletGeometry:
    runner_spacing: float = 0.68     # iki rayin merkezleri arasi (m)
    spacing_tolerance: float = 0.08
    runner_end_to_center: float = 0.55  # ray ucu ile palet merkezi arasi (m)
    max_range: float = 3.0
    cluster_gap: float = 0.06
    min_cluster_points: int = 3
    end_face_depth: float = 0.06     # kumenin en yakin kismindan bu kadar derinlik


@dataclass
class PalletDetection:
    x: float          # palet merkezi, LiDAR cercevesinde (m)
    y: float
    yaw: float        # paletin "giris" yonu (robotun palete girerken bakacagi yon)
    front_x: float    # palet agzinin ortasi
    front_y: float
    spacing: float    # olculen ray araligi
    score: float      # kucuk = iyi


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, max_range: float):
    """(aci, x, y) listesi; gecersiz olcumler icin None."""
    pts = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        if r is None or not math.isfinite(r) or r < range_min or r > max_range:
            pts.append(None)
        else:
            pts.append((a, r * math.cos(a), r * math.sin(a)))
    return pts


def cluster(points, gap: float, min_points: int):
    """Ardisik ve birbirine yakin noktalari indeks listeleri olarak kumeler."""
    clusters, cur, last = [], [], None
    for i, p in enumerate(points):
        if p is None:
            if len(cur) >= min_points:
                clusters.append(cur)
            cur, last = [], None
            continue
        if last is not None and math.hypot(p[1] - last[1], p[2] - last[2]) > gap:
            if len(cur) >= min_points:
                clusters.append(cur)
            cur = []
        cur.append(i)
        last = p
    if len(cur) >= min_points:
        clusters.append(cur)
    return clusters


def end_face_centroid(points, idx, depth):
    """Kumenin LiDAR'a en yakin kismi (ray ucu yuzu) icin agirlik merkezi."""
    ds = [math.hypot(points[i][1], points[i][2]) for i in idx]
    dmin = min(ds)
    sel = [i for i, d in zip(idx, ds) if d <= dmin + depth]
    x = sum(points[i][1] for i in sel) / len(sel)
    y = sum(points[i][2] for i in sel) / len(sel)
    return x, y, min(sel), max(sel)


def detect_pallet(ranges: Sequence[float], angle_min: float, angle_increment: float,
                  range_min: float = 0.1, geometry: PalletGeometry = PalletGeometry(),
                  expected_yaw: Optional[float] = None) -> Optional[PalletDetection]:
    """En olasi paleti dondurur, yoksa None."""
    g = geometry
    pts = scan_to_points(ranges, angle_min, angle_increment, range_min, g.max_range)
    clusters = cluster(pts, g.cluster_gap, g.min_cluster_points)
    # (uc_x, uc_y, kume_ilk_indeks, kume_son_indeks)
    ends = [end_face_centroid(pts, c, g.end_face_depth)[:2] + (c[0], c[-1]) for c in clusters]

    best: Optional[PalletDetection] = None
    for a in range(len(ends)):
        for b in range(a + 1, len(ends)):
            ax, ay, a_lo, a_hi = ends[a]
            bx, by, b_lo, b_hi = ends[b]
            spacing = math.hypot(bx - ax, by - ay)
            if abs(spacing - g.runner_spacing) > g.spacing_tolerance:
                continue
            mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
            # Ciftin normali (LiDAR'dan uzaga bakan)
            nx, ny = -(by - ay) / spacing, (bx - ax) / spacing
            if nx * mx + ny * my < 0:
                nx, ny = -nx, -ny
            # Palete cok egik bakiliyorsa (>60 derece) guvenilmez
            view = math.acos(max(-1.0, min(1.0, (nx * mx + ny * my) / max(1e-6, math.hypot(mx, my)))))
            if view > math.radians(60):
                continue
            # Agiz bos mu? Iki kume arasindaki noktalar palet cercevesine
            # tasinir; acikligin icinde (ray ic yuzlerinden uzakta) ve palet
            # derinligi icinde nokta varsa bu bir palet agzi degildir.
            ux, uy = (bx - ax) / spacing, (by - ay) / spacing
            lo, hi = (a_hi, b_lo) if a_hi < b_lo else (b_hi, a_lo)
            half_open = spacing / 2.0 - 0.07
            gap_ok, n_gap = True, 0
            for i in range(lo + 1, hi):
                n_gap += 1
                p = pts[i]
                if p is None:
                    continue
                rx, ry = p[1] - mx, p[2] - my
                along, lat = rx * nx + ry * ny, rx * ux + ry * uy
                if abs(lat) < half_open and -0.10 < along < 2.0 * g.runner_end_to_center:
                    gap_ok = False
                    break
            if not gap_ok or n_gap < 3:
                continue
            front_dist = math.hypot(mx, my)
            yaw = math.atan2(ny, nx)
            score = abs(spacing - g.runner_spacing) * 5.0 + view * 0.5 + front_dist * 0.05
            if expected_yaw is not None:
                score += abs(math.atan2(math.sin(yaw - expected_yaw), math.cos(yaw - expected_yaw)))
            cx = mx + nx * g.runner_end_to_center
            cy = my + ny * g.runner_end_to_center
            det = PalletDetection(cx, cy, yaw, mx, my, spacing, score)
            if best is None or det.score < best.score:
                best = det
    return best


def average_detections(dets: List[PalletDetection]) -> Optional[PalletDetection]:
    """Birden fazla scan sonucunun ortalamasi (aci icin dairesel ortalama)."""
    if not dets:
        return None
    n = len(dets)
    s = sum(math.sin(d.yaw) for d in dets) / n
    c = sum(math.cos(d.yaw) for d in dets) / n
    return PalletDetection(
        sum(d.x for d in dets) / n, sum(d.y for d in dets) / n, math.atan2(s, c),
        sum(d.front_x for d in dets) / n, sum(d.front_y for d in dets) / n,
        sum(d.spacing for d in dets) / n, sum(d.score for d in dets) / n)
