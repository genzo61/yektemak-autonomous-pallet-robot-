"""Yektemak fabrika dunyasi (filo yonetimi senaryosu) uretici.

36 x 24 m fabrika, tam malzeme akisi:
    MAL KABUL (IN)  ->  DEPO (ST)  ->  URETIM HATTI (PL)  ->  SEVKIYAT (OUT)
Ayrica 2 sarj istasyonu (CH), dekor raflari, ofis, kolonlar.

Calistir:  python3 tools/generate_factory.py
Uretilenler:
    src/yektemak_gazebo/worlds/factory.world
    src/yektemak_fleet/config/stations.yaml   (istasyon konumlari, filo yoneticisi okur)

Istasyon pozu = palet merkezi + robotun palete girerken baktigi yon (yaw).
Robot paletin "arkasindan" (yaw yonunun tersinden) yaklasir, birakinca ileri cikar.
"""

import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")
PI2 = math.pi / 2

# --------------------------------------------------------------------------- yerlesim
X0, X1, Y0, Y1 = -2.0, 34.0, -12.0, 12.0

STATIONS = {
    # ad: (x, y, yaw, tur)
    "CH_1": (0.5, 1.5, 0.0, "charge"),
    "CH_2": (0.5, -1.5, 0.0, "charge"),
    "IN_1": (4.0, -8.0, -PI2, "inbound"),
    "IN_2": (7.5, -8.0, -PI2, "inbound"),
    "IN_3": (11.0, -8.0, -PI2, "inbound"),
    "ST_1": (17.0, -7.0, -PI2, "storage"),
    "ST_2": (20.0, -7.0, -PI2, "storage"),
    "ST_3": (23.0, -7.0, -PI2, "storage"),
    "ST_4": (26.0, -7.0, -PI2, "storage"),
    "PL_1": (17.5, 5.5, PI2, "production"),
    "PL_2": (23.5, 5.5, PI2, "production"),
    "OUT_1": (30.5, -3.0, 0.0, "outbound"),
    "OUT_2": (30.5, 0.0, 0.0, "outbound"),
    "OUT_3": (30.5, 3.0, 0.0, "outbound"),
}
# Baslangicta palet olan istasyonlar
INITIAL_PALLETS = ["IN_1", "IN_2", "IN_3", "ST_1", "ST_2"]
COLUMNS = [(9.25, 4.5), (9.25, -4.5), (20.5, 4.5), (14.0, -4.5)]

# --------------------------------------------------------------------------- yardimcilar
def mat(rgb, emissive=None, alpha=None):
    r, g, b = rgb
    e = f"<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>" if emissive else ""
    return (f"<material><ambient>{r*0.8:.2f} {g*0.8:.2f} {b*0.8:.2f} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse><specular>0.1 0.1 0.1 1</specular>{e}</material>"
            + (f"<transparency>{alpha}</transparency>" if alpha else ""))


_n = [0]


def box(pose, size, rgb, collide=True, yaw=0.0, emissive=None, alpha=None, name=None):
    _n[0] += 1
    name = name or f"b{_n[0]}"
    p = " ".join(f"{v:.4g}" for v in pose) + f" 0 0 {yaw:.4g}"
    s = " ".join(f"{v:.4g}" for v in size)
    out = ""
    if collide:
        out += (f'<collision name="{name}_c"><pose>{p}</pose>'
                f'<geometry><box><size>{s}</size></box></geometry></collision>\n')
    out += (f'<visual name="{name}_v"><pose>{p}</pose><cast_shadows>{"true" if collide else "false"}'
            f'</cast_shadows><geometry><box><size>{s}</size></box></geometry>'
            f'{mat(rgb, emissive)}' + (f"<transparency>{alpha}</transparency>" if alpha else "")
            + '</visual>\n')
    return out


def cyl(pose, r, h, rgb, collide=True, emissive=None):
    _n[0] += 1
    name = f"c{_n[0]}"
    p = " ".join(f"{v:.4g}" for v in pose) + " 0 0 0"
    out = ""
    if collide:
        out += (f'<collision name="{name}_c"><pose>{p}</pose><geometry><cylinder><radius>{r}</radius>'
                f'<length>{h}</length></cylinder></geometry></collision>\n')
    out += (f'<visual name="{name}_v"><pose>{p}</pose><geometry><cylinder><radius>{r}</radius>'
            f'<length>{h}</length></cylinder></geometry>{mat(rgb, emissive)}</visual>\n')
    return out


def floor(cx, cy, sx, sy, rgb, z=0.001):
    return box((cx, cy, z), (sx, sy, 0.002), rgb, collide=False)


def frame(cx, cy, sx, sy, rgb, w=0.07):
    return (floor(cx, cy + sy / 2, sx + w, w, rgb, 0.003) + floor(cx, cy - sy / 2, sx + w, w, rgb, 0.003)
            + floor(cx + sx / 2, cy, w, sy, rgb, 0.003) + floor(cx - sx / 2, cy, w, sy, rgb, 0.003))


def static_model(name, body):
    return (f'    <model name="{name}">\n      <static>true</static>\n      <link name="link">\n'
            f'{body}      </link>\n    </model>\n')


def include(uri, name, x, y, yaw=0.0):
    return (f"    <include>\n      <uri>model://{uri}</uri>\n      <name>{name}</name>\n"
            f"      <pose>{x} {y} 0 0 0 {yaw:.5f}</pose>\n    </include>\n")


# --------------------------------------------------------------------------- renkler
WALL = (0.82, 0.82, 0.78)
YEL = (0.98, 0.82, 0.05)
BLACK = (0.08, 0.08, 0.08)
ZONE = {
    "charge": (0.95, 0.85, 0.25), "inbound": (0.20, 0.70, 0.35), "storage": (0.20, 0.45, 0.90),
    "production": (0.95, 0.50, 0.10), "outbound": (0.60, 0.30, 0.80),
}
TINT = {k: tuple(0.55 + 0.12 * (c - 0.55) for c in v) for k, v in ZONE.items()}


# --------------------------------------------------------------------------- bina
def building():
    t, h = 0.25, 4.0
    b = ""
    b += box(((X0 + X1) / 2, Y1 + t / 2, h / 2), (X1 - X0 + 2 * t, t, h), WALL)
    b += box((X0 - t / 2, 0, h / 2), (t, Y1 - Y0, h), WALL)
    # Guney duvari: mal kabul rampa kapilari icin bosluklu
    doors_s = [STATIONS[n][0] for n in ("IN_1", "IN_2", "IN_3")]
    b += wall_with_doors("x", Y0 - t / 2, X0 - t, X1 + t, doors_s, t, h)
    # Dogu duvari: sevkiyat kapilari
    doors_e = [STATIONS[n][1] for n in ("OUT_1", "OUT_2", "OUT_3")]
    b += wall_with_doors("y", X1 + t / 2, Y0, Y1, doors_e, t, h)
    for x, y in COLUMNS:
        b += box((x, y, h / 2), (0.40, 0.40, h), (0.68, 0.68, 0.68))
        b += box((x, y, 0.15), (0.52, 0.52, 0.30), YEL)
    # Ofis (kuzeydogu kose): cam duvarlar, kapi
    ox0, ox1, oy0 = 29.0, X1, 7.5
    b += box(((ox0 + ox1) / 2 + 0.6, oy0, 1.25), (ox1 - ox0 - 1.2, 0.12, 2.5), (0.55, 0.75, 0.85), alpha=0.35)
    b += box((ox0, (oy0 + Y1) / 2, 1.25), (0.12, Y1 - oy0, 2.5), (0.55, 0.75, 0.85), alpha=0.35)
    b += box(((ox0 + ox1) / 2 + 0.6, oy0, 0.12), (ox1 - ox0 - 1.2, 0.14, 0.24), (0.3, 0.3, 0.3))
    b += box((ox0, (oy0 + Y1) / 2, 0.12), (0.14, Y1 - oy0, 0.24), (0.3, 0.3, 0.3))
    for i, (x, y) in enumerate([(31.0, 10.0), (32.8, 10.0)]):
        b += box((x, y, 0.38), (1.4, 0.7, 0.04), (0.55, 0.38, 0.22))          # masa
        b += box((x, y + 0.25, 0.62), (0.5, 0.04, 0.32), BLACK, collide=False)  # monitor
    return b


def wall_with_doors(axis, fixed, a0, a1, doors, t, h, door_w=2.8):
    """Kapi bosluklu duvar. Kapi yerinde zeminden 3 m acik, ustu duvar."""
    b, cur = "", a0
    segs = []
    for d in sorted(doors):
        segs.append((cur, d - door_w / 2))
        cur = d + door_w / 2
    segs.append((cur, a1))
    for s0, s1 in segs:
        c, L = (s0 + s1) / 2, s1 - s0
        b += box((c, fixed, h / 2) if axis == "x" else (fixed, c, h / 2),
                 (L, t, h) if axis == "x" else (t, L, h), WALL)
    for d in doors:
        # Kapi ustu, yan tamponlar, rampa (lidar icin kapi ag i zeminde kapali bir esik)
        b += box((d, fixed, 3.5) if axis == "x" else (fixed, d, 3.5),
                 (door_w, t, 1.0) if axis == "x" else (t, door_w, 1.0), WALL)
        b += box((d, fixed, 0.5) if axis == "x" else (fixed, d, 0.5),
                 (door_w, 0.05, 1.0) if axis == "x" else (0.05, door_w, 1.0), (0.35, 0.37, 0.40))
        for s in (-1, 1):
            q = d + s * (door_w / 2 - 0.1)
            b += box((q, fixed - (0.2 if axis == "x" else 0) * 0, 0.6) if axis == "x" else (fixed, q, 0.6),
                     (0.18, 0.35, 1.2) if axis == "x" else (0.35, 0.18, 1.2), BLACK)
    return b


def trucks():
    """Kapilarin disinda dorseler (sadece gorsel)."""
    b = ""
    for n in ("IN_1", "IN_2", "IN_3"):
        x = STATIONS[n][0]
        b += box((x, Y0 - 0.25 - 6.5, 2.0), (2.5, 13.0, 2.8), (0.85, 0.85, 0.88), collide=False)
        b += box((x, Y0 - 0.25 - 13.3, 1.6), (2.5, 0.6, 2.2), (0.15, 0.35, 0.65), collide=False)
    for n in ("OUT_1", "OUT_2", "OUT_3"):
        y = STATIONS[n][1]
        b += box((X1 + 0.25 + 6.5, y, 2.0), (13.0, 2.5, 2.8), (0.90, 0.90, 0.92), collide=False)
    return b


def production_line():
    """Konveyor + 3 makine + guvenlik citi (on yuzu zeminde dolu, LiDAR gorur)."""
    b = ""
    cx0, cx1, cy = 14.0, 27.5, 10.0
    b += box(((cx0 + cx1) / 2, cy, 0.42), (cx1 - cx0, 0.9, 0.08), (0.18, 0.18, 0.20))   # bant
    b += box(((cx0 + cx1) / 2, cy, 0.20), (cx1 - cx0, 0.7, 0.36), (0.45, 0.47, 0.50))  # govde
    for i in range(10):
        b += box((cx0 + 0.7 + i * 1.35, cy, 0.52), (0.45, 0.35, 0.12),
                 (0.78, 0.6, 0.38) if i % 2 else (0.66, 0.49, 0.29), collide=False)       # bantta koliler
    for i, x in enumerate([16.0, 20.5, 25.0]):
        b += box((x, 11.2, 1.1), (2.6, 1.3, 2.2), (0.10, 0.55, 0.60))
        b += box((x, 10.52, 1.3), (1.2, 0.06, 0.6), (0.05, 0.08, 0.10), collide=False)   # ekran
        b += cyl((x + 1.0, 11.2, 2.45), 0.08, 0.3, (0.1, 0.9, 0.2), collide=False, emissive=(0.1, 0.9, 0.2))
    # Guvenlik citi: y=8.0, x 13..28
    fy, fx0, fx1 = 8.0, 13.0, 28.0
    b += box(((fx0 + fx1) / 2, fy, 0.15), (fx1 - fx0, 0.06, 0.24), YEL)                 # zemin kick plate
    b += box(((fx0 + fx1) / 2, fy, 1.0), (fx1 - fx0, 0.02, 1.5), (0.3, 0.3, 0.3), alpha=0.6)
    b += box(((fx0 + fx1) / 2, fy, 1.75), (fx1 - fx0, 0.06, 0.06), YEL)
    x = fx0
    while x <= fx1 + 1e-6:
        b += box((x, fy, 0.9), (0.06, 0.06, 1.8), YEL)
        x += 1.5
    for s in (fx0, fx1):
        b += box((s, (fy + Y1) / 2, 0.9), (0.06, Y1 - fy, 1.8), YEL)
        b += box((s, (fy + Y1) / 2, 0.15), (0.06, Y1 - fy, 0.24), YEL)
    return b


def chargers():
    b = ""
    for n in ("CH_1", "CH_2"):
        y = STATIONS[n][1]
        b += box((X0 + 0.2, y, 0.45), (0.3, 0.7, 0.9), (0.20, 0.22, 0.25))
        b += box((X0 + 0.37, y, 0.75), (0.04, 0.4, 0.12), (0.1, 0.9, 0.3), collide=False, emissive=(0.1, 0.9, 0.3))
        b += box((X0 + 0.5, y, 0.10), (0.25, 0.3, 0.08), (0.75, 0.55, 0.1))
    return b


def markings():
    b = ""
    # Bolge zeminleri
    b += floor(7.5, -8.6, 11.0, 6.4, TINT["inbound"])
    b += floor(21.5, -8.6, 13.0, 6.4, TINT["storage"])
    b += floor(20.5, 6.0, 15.0, 3.8, TINT["production"])
    b += floor(31.3, 0.0, 5.0, 9.0, TINT["outbound"])
    b += floor(0.6, 0.0, 2.6, 5.4, TINT["charge"])
    # Istasyon cerceveleri (palet 1.2 x 0.8, cerceve biraz buyuk)
    for n, (x, y, yaw, kind) in STATIONS.items():
        along = abs(math.cos(yaw)) > 0.5
        sx, sy = (1.45, 1.05) if along else (1.05, 1.45)
        if kind == "charge":
            sx, sy = 1.15, 0.8
        b += frame(x, y, sx, sy, ZONE[kind])
    # Ana koridor seritleri
    for y in (3.5, -3.5):
        b += floor(15.0, y, 26.0, 0.09, YEL, 0.003)
    # Yaya yolu (beyaz kesikli) bati duvari boyunca
    for i in range(9):
        b += floor(-1.2, -10.5 + i * 2.6, 0.18, 1.3, (0.95, 0.95, 0.95), 0.003)
    return b


def write_world():
    pallets = ""
    for i, n in enumerate(INITIAL_PALLETS, 1):
        x, y, yaw, _ = STATIONS[n]
        pallets += include("yektemak_pallet", f"pallet_{i}", x, y, yaw)
    racks = ""
    rack_poses = [(4.5, 6.0), (9.0, 6.0), (4.5, 10.0), (9.0, 10.0),
                  (17.5, -10.4), (22.0, -10.4), (26.5, -10.4)]
    for i, (x, y) in enumerate(rack_poses, 1):
        racks += include("yektemak_warehouse_rack", f"rack_{i}", x, y)

    world = f"""<?xml version="1.0"?>
<!-- Bu dosya tools/generate_factory.py ile uretildi. Elle duzenlemek yerine uretici dosyayi degistirin. -->
<sdf version="1.6">
  <world name="yektemak_factory">
    <gravity>0 0 -9.81</gravity>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20.0</update_rate>
    </plugin>
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.70 0.76 0.82 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.35 0.25 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          {mat((0.55, 0.57, 0.58))}
        </visual>
      </link>
    </model>
{static_model("factory_building", building())}{static_model("trucks", trucks())}{static_model("production_line", production_line())}{static_model("chargers", chargers())}{static_model("floor_markings", markings())}{racks}{pallets}
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-6 -22 22 0 0.75 0.78</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
"""
    path = os.path.join(SRC, "yektemak_gazebo", "worlds", "factory.world")
    open(path, "w").write(world)
    print("yazildi:", path)


def write_stations():
    d = {"stations": {n: {"x": x, "y": y, "yaw": round(yaw, 5), "type": k}
                      for n, (x, y, yaw, k) in STATIONS.items()},
         "initial_pallets": {n: f"pallet_{i}" for i, n in enumerate(INITIAL_PALLETS, 1)}}
    path = os.path.join(SRC, "yektemak_fleet", "config", "stations.yaml")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("# tools/generate_factory.py ile uretildi. Dunya ile ayni olmali.\n")
        yaml.safe_dump(d, f, sort_keys=False)
    print("yazildi:", path)


if __name__ == "__main__":
    write_world()
    write_stations()
