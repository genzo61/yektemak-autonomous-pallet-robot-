"""Yektemak LIMAN sahasi uretici (acik alan, RTK-GNSS senaryosu).

64 x 40 m acik liman sahasi:
    GIRIS KAPISI (GATE, tirdan gelen paletler)
        -> KONTEYNER SAHASI (YARD, konteyner bloklari arasindaki "sokak")
        -> KONTEYNER DOLDURMA (CNT, acik kapili konteynerlere palet yukleme)
        -> RIHTIM (QUAY, kren altinda gemiye yukleme)
Ayrica 3 sarj istasyonu (CH), ofis, isik direkleri, cevre citi, gemi ve kren.

Calistir:  python3 tools/generate_port.py
Uretilenler:
    src/yektemak_gazebo/worlds/port.world
    src/yektemak_navigation/maps/port.pgm + port.yaml   (dunya geometrisinden)
    src/yektemak_fleet/config/port_stations.yaml        (istasyonlar, bolgeler, GNSS bolgeleri)
    dashboard/public/port.png + port.json                (panel haritasi)

Harita SLAM ile degil, dunyanin kendi geometrisinden (LiDAR yuksekligindeki
engeller) uretilir. Gercek bir limanda bu, saha olcum (survey) verisine karsilik gelir.
"""

import json
import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
SRC = os.path.join(ROOT, "src")
PI2 = math.pi / 2

X0, X1, Y0, Y1 = -4.0, 60.0, -20.0, 20.0
QUAY_Y = 19.7                 # rihtim kenari (kuzey)
LIDAR_Z = 0.11
RES = 0.05

# Dunya orijininin (0, 0) cografi konumu: GNSS datumu
DATUM = {"lat": 40.9650, "lon": 27.0000}

STATIONS = {
    "CH_1": (0.0, 10.0, 0.0, "charge"),
    "CH_2": (0.0, 13.0, 0.0, "charge"),
    "CH_3": (0.0, 16.0, 0.0, "charge"),
    "GATE_1": (8.0, -15.0, -PI2, "inbound"),
    "GATE_2": (12.0, -15.0, -PI2, "inbound"),
    "GATE_3": (16.0, -15.0, -PI2, "inbound"),
    "YARD_1": (17.0, -1.0, -PI2, "storage"),
    "YARD_2": (21.0, -1.0, -PI2, "storage"),
    "YARD_3": (25.0, -1.0, -PI2, "storage"),
    "YARD_4": (29.0, -1.0, -PI2, "storage"),
    "CNT_1": (42.0, 5.0, 0.0, "production"),
    "CNT_2": (42.0, -7.0, 0.0, "production"),
    "QUAY_1": (21.0, 15.0, PI2, "outbound"),
    "QUAY_2": (26.0, 15.0, PI2, "outbound"),
    "QUAY_3": (31.0, 15.0, PI2, "outbound"),
}
INITIAL_PALLETS = ["GATE_1", "GATE_2", "GATE_3", "YARD_1", "YARD_2"]

# Bolgeler (panel + kilitler): ad, gorunen ad, (cx, cy, sx, sy)
ZONES = {
    "inbound": ("Giris kapisi", (12.0, -15.6, 13.0, 6.0)),
    "storage": ("Konteyner sahasi", (23.0, -1.0, 16.0, 5.6)),
    "production": ("Konteyner doldurma", (43.5, -1.0, 6.0, 16.0)),
    "outbound": ("Rihtim / gemi", (26.0, 15.6, 14.0, 5.6)),
    "charge": ("Sarj", (0.0, 13.0, 2.6, 9.0)),
}

# GNSS bolgeleri: bu dikdortgenlerde RTK en fazla bu kaliteye cikabilir
#   FLOAT: uydu gorusu kismen kapali (konteyner sokagi, kren alti, bina yani)
GNSS_ZONES = [
    {"name": "konteyner sokagi", "rect": [12.0, -3.6, 34.0, 1.6], "cap": "FLOAT"},
    {"name": "kren alti", "rect": [17.0, 13.0, 35.0, 19.7], "cap": "FLOAT"},
    {"name": "ofis yani", "rect": [-4.0, 7.0, 2.0, 19.0], "cap": "FLOAT"},
    {"name": "konteyner ici", "rect": [43.0, -8.5, 49.5, 6.5], "cap": "SINGLE"},
]

# --------------------------------------------------------------------------- SDF yardimcilari
OBST = []   # haritaya girecek engeller: (cx, cy, sx, sy, yaw)


def mat(rgb, emissive=None):
    r, g, b = rgb
    e = f"<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>" if emissive else ""
    return (f"<material><ambient>{r*0.85:.2f} {g*0.85:.2f} {b*0.85:.2f} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse><specular>0.15 0.15 0.15 1</specular>{e}</material>")


_n = [0]


def box(pose, size, rgb, collide=True, yaw=0.0, emissive=None, shadow=True):
    _n[0] += 1
    name = f"b{_n[0]}"
    x, y, z = pose
    sx, sy, sz = size
    p = f"{x:.4g} {y:.4g} {z:.4g} 0 0 {yaw:.4g}"
    s = f"{sx:.4g} {sy:.4g} {sz:.4g}"
    out = ""
    if collide:
        out += (f'<collision name="{name}_c"><pose>{p}</pose>'
                f'<geometry><box><size>{s}</size></box></geometry></collision>\n')
        if z - sz / 2 <= LIDAR_Z <= z + sz / 2:
            OBST.append((x, y, sx, sy, yaw))
    out += (f'<visual name="{name}_v"><pose>{p}</pose><cast_shadows>{"true" if shadow else "false"}'
            f'</cast_shadows><geometry><box><size>{s}</size></box></geometry>{mat(rgb, emissive)}</visual>\n')
    return out


def cyl(pose, r, h, rgb, collide=True, emissive=None):
    _n[0] += 1
    name = f"c{_n[0]}"
    x, y, z = pose
    out = ""
    if collide:
        out += (f'<collision name="{name}_c"><pose>{x} {y} {z} 0 0 0</pose><geometry><cylinder>'
                f'<radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>\n')
        if z - h / 2 <= LIDAR_Z <= z + h / 2:
            OBST.append((x, y, 2 * r, 2 * r, 0.0))
    out += (f'<visual name="{name}_v"><pose>{x} {y} {z} 0 0 0</pose><geometry><cylinder><radius>{r}</radius>'
            f'<length>{h}</length></cylinder></geometry>{mat(rgb, emissive)}</visual>\n')
    return out


def floor(cx, cy, sx, sy, rgb, z=0.002, yaw=0.0):
    return box((cx, cy, z), (sx, sy, 0.002), rgb, collide=False, yaw=yaw, shadow=False)


def frame(cx, cy, sx, sy, rgb, w=0.08):
    return (floor(cx, cy + sy / 2, sx + w, w, rgb, 0.004) + floor(cx, cy - sy / 2, sx + w, w, rgb, 0.004)
            + floor(cx + sx / 2, cy, w, sy, rgb, 0.004) + floor(cx - sx / 2, cy, w, sy, rgb, 0.004))


def static_model(name, body):
    return (f'    <model name="{name}">\n      <static>true</static>\n      <link name="link">\n'
            f'{body}      </link>\n    </model>\n')


def include(uri, name, x, y, yaw=0.0):
    return (f"    <include>\n      <uri>model://{uri}</uri>\n      <name>{name}</name>\n"
            f"      <pose>{x} {y} 0 0 0 {yaw:.5f}</pose>\n    </include>\n")


YEL = (0.98, 0.82, 0.05)
ZONE_COLOR = {"charge": (0.95, 0.85, 0.25), "inbound": (0.20, 0.70, 0.35), "storage": (0.20, 0.45, 0.90),
              "production": (0.95, 0.50, 0.10), "outbound": (0.60, 0.30, 0.80)}
CONTAINER_COLORS = [(0.70, 0.12, 0.10), (0.10, 0.35, 0.65), (0.15, 0.50, 0.25), (0.85, 0.55, 0.10),
                    (0.55, 0.55, 0.58), (0.45, 0.20, 0.45), (0.80, 0.80, 0.78), (0.20, 0.55, 0.60)]


# --------------------------------------------------------------------------- saha
def container(x, y, level, color, yaw=0.0):
    L, W, H = 6.06, 2.44, 2.59
    z = H / 2 + level * H
    b = box((x, y, z), (L, W, H - 0.02), color, yaw=yaw)
    # Oluklu yan yuzey cizgileri (gorsel)
    for k in range(-4, 5):
        dx = k * 0.6
        b += box((x + dx, y + W / 2 + 0.005, z), (0.08, 0.01, H - 0.2), tuple(c * 0.8 for c in color),
                 collide=False, shadow=False)
        b += box((x + dx, y - W / 2 - 0.005, z), (0.08, 0.01, H - 0.2), tuple(c * 0.8 for c in color),
                 collide=False, shadow=False)
    return b


def container_blocks():
    b = ""
    i = 0
    for row_y in (4.5, 7.2, -6.5, -9.2):          # iki blok, her blok iki sira
        for k, x in enumerate([15.0, 21.3, 27.6, 33.9]):
            levels = 1 + ((k * 7 + int(row_y * 3)) % 3)
            for lv in range(levels):
                b += container(x, row_y, lv, CONTAINER_COLORS[(i + lv * 3) % len(CONTAINER_COLORS)])
            i += 1
    return b


def stuffing_containers():
    """Kapilari acik, icine palet yuklenen konteynerler (CNT istasyonlarinin dogusunda)."""
    b = ""
    L, W, H, t = 6.06, 2.44, 2.59, 0.06
    for cy in (5.0, -7.0):
        x0 = 43.2                                  # acik kapi (bati yuzu)
        col = (0.10, 0.35, 0.65) if cy > 0 else (0.70, 0.12, 0.10)
        b += box((x0 + L / 2, cy + W / 2 - t / 2, H / 2), (L, t, H), col)      # kuzey duvar
        b += box((x0 + L / 2, cy - W / 2 + t / 2, H / 2), (L, t, H), col)      # guney duvar
        b += box((x0 + L - t / 2, cy, H / 2), (t, W, H), col)                  # arka duvar
        b += box((x0 + L / 2, cy, H - t / 2), (L, W, t), col)                  # tavan
        b += box((x0 + L / 2, cy, 0.01), (L - 0.1, W - 0.1, 0.02), (0.45, 0.35, 0.25), collide=False)
        for s in (-1, 1):                                                       # acik kapi kanatlari
            b += box((x0 - 0.62, cy + s * (W / 2 + 0.62), H / 2), (1.2, t, H - 0.1),
                     tuple(c * 0.9 for c in col), yaw=s * 1.2)
    return b


def quay():
    b = ""
    # Rihtim kenari: sari tampon (LiDAR gorur, robot suya dusmez)
    b += box(((X0 + X1) / 2, QUAY_Y, 0.15), (X1 - X0, 0.4, 0.30), YEL)
    for x in range(-2, 60, 6):                                                  # baba (bollard)
        b += cyl((x, QUAY_Y - 0.5, 0.25), 0.18, 0.5, (0.15, 0.15, 0.15))
    # Su ve gemi (gorsel)
    b += box(((X0 + X1) / 2 + 10, 40.0, -0.4), (120, 40, 0.1), (0.12, 0.30, 0.42), collide=False, shadow=False)
    b += box((26.0, 27.5, 3.0), (52.0, 11.0, 7.0), (0.15, 0.18, 0.22), collide=False)        # gemi govdesi
    b += box((26.0, 27.5, 6.6), (52.2, 11.2, 0.3), (0.65, 0.12, 0.10), collide=False)
    for k, x in enumerate(range(6, 48, 7)):                                       # gemideki konteynerler
        for lv in range(2):
            b += box((x, 27.5, 8.0 + lv * 2.6), (6.0, 9.0, 2.5),
                     CONTAINER_COLORS[(k + lv) % len(CONTAINER_COLORS)], collide=False)
    # Rihtim kreni (STS): 4 bacak engel, ust kiris gorsel
    for x in (17.0, 35.0):
        for y in (13.0, 19.2):
            b += box((x, y, 9.0), (1.0, 1.0, 18.0), (0.92, 0.55, 0.10))
            b += box((x, y, 0.25), (1.3, 1.3, 0.5), YEL)
    b += box((26.0, 13.0, 18.3), (19.0, 1.0, 1.2), (0.92, 0.55, 0.10), collide=False)
    b += box((26.0, 19.2, 18.3), (19.0, 1.0, 1.2), (0.92, 0.55, 0.10), collide=False)
    b += box((26.0, 22.0, 19.2), (3.0, 30.0, 1.4), (0.92, 0.55, 0.10), collide=False)         # bom
    b += box((26.0, 16.0, 16.5), (3.5, 2.5, 2.5), (0.85, 0.85, 0.82), collide=False)          # kabin
    return b


def perimeter():
    """Cevre citi (bati, guney, dogu), guneybatida giris kapisi bosluklu."""
    b = ""
    gray = (0.55, 0.57, 0.6)

    def fence(x0, y0, x1, y1):
        out = ""
        L = math.hypot(x1 - x0, y1 - y0)
        cx, cy, yaw = (x0 + x1) / 2, (y0 + y1) / 2, math.atan2(y1 - y0, x1 - x0)
        out += box((cx, cy, 0.15), (L, 0.06, 0.25), YEL, yaw=yaw)                  # zemin kick plate
        out += box((cx, cy, 1.1), (L, 0.02, 1.6), gray, yaw=yaw, collide=False)    # tel
        n = max(1, int(L / 3))
        for i in range(n + 1):
            t = i / n
            out += cyl((x0 + t * (x1 - x0), y0 + t * (y1 - y0), 1.0), 0.05, 2.0, gray)
        return out
    b += fence(X0, Y0, X0, QUAY_Y - 0.2)
    b += fence(X0, Y0, 3.0, Y0)                     # kapi: x 3..21 arasi acik (tirlar)
    b += fence(21.0, Y0, X1, Y0)
    b += fence(X1, Y0, X1, QUAY_Y - 0.2)
    b += box((2.8, Y0, 3.5), (0.4, 0.4, 7.0), (0.3, 0.3, 0.32))                   # kapi direkleri
    b += box((21.2, Y0, 3.5), (0.4, 0.4, 7.0), (0.3, 0.3, 0.32))
    b += box((12.0, Y0, 6.8), (18.8, 0.6, 0.8), (0.95, 0.80, 0.05), collide=False)  # kapi tabelasi
    # Giris kapisinda bekleyen tirlar (gorsel, cit disinda)
    for x in (8.0, 12.0, 16.0):
        b += box((x, Y0 - 7.5, 2.0), (2.5, 13.0, 2.8), (0.86, 0.86, 0.9), collide=False)
        b += box((x, Y0 - 14.4, 1.7), (2.5, 1.0, 2.4), (0.15, 0.35, 0.65), collide=False)
    return b


def office_and_chargers():
    b = ""
    b += box((-2.9, 13.0, 1.6), (2.2, 12.0, 3.2), (0.82, 0.82, 0.78))
    b += box((-1.78, 13.0, 2.2), (0.02, 10.0, 1.0), (0.35, 0.55, 0.75), collide=False)        # pencere
    for n in ("CH_1", "CH_2", "CH_3"):
        y = STATIONS[n][1]
        b += box((-1.55, y, 0.45), (0.3, 0.7, 0.9), (0.20, 0.22, 0.25))
        b += box((-1.38, y, 0.75), (0.04, 0.4, 0.12), (0.1, 0.9, 0.3), collide=False, emissive=(0.1, 0.9, 0.3))
    # RTK baz istasyonu (ofis catisinda anten) + isik direkleri
    b += cyl((-2.9, 17.5, 3.9), 0.04, 1.4, (0.85, 0.85, 0.85), collide=False)
    b += box((-2.9, 17.5, 4.65), (0.35, 0.35, 0.12), (0.95, 0.95, 0.95), collide=False)
    for (x, y) in [(6, 2), (6, -8), (38, 11), (38, -13), (50, 2), (10, 9), (52, 14), (52, -14)]:
        b += cyl((x, y, 6.0), 0.12, 12.0, (0.6, 0.6, 0.62))
        b += box((x, y, 12.1), (0.6, 0.3, 0.2), (0.95, 0.95, 0.8), collide=False, emissive=(0.4, 0.4, 0.3))
        b += box((x, y, 0.2), (0.5, 0.5, 0.4), YEL)
    return b


def markings():
    b = ""
    for key, (_, (cx, cy, sx, sy)) in ZONES.items():
        c = ZONE_COLOR[key]
        b += floor(cx, cy, sx, sy, tuple(0.30 + 0.25 * v for v in c), 0.0015)
    for n, (x, y, yaw, kind) in STATIONS.items():
        along = abs(math.cos(yaw)) > 0.5
        sx, sy = (1.45, 1.05) if along else (1.05, 1.45)
        if kind == "charge":
            sx, sy = 1.15, 0.8
        b += frame(x, y, sx, sy, ZONE_COLOR[kind])
    # Ana yollar: beyaz kesikli seritler
    for y in (11.0, -12.0):
        for x in range(0, 56, 4):
            b += floor(x + 1, y, 2.0, 0.15, (0.92, 0.92, 0.92), 0.003)
    for x in (8.0, 38.5):
        for y in range(-11, 11, 4):
            b += floor(x, y + 1, 0.15, 2.0, (0.92, 0.92, 0.92), 0.003)
    return b


# --------------------------------------------------------------------------- harita
def rasterize():
    W, H = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
    img = bytearray([254]) * (W * H)
    for (cx, cy, sx, sy, yaw) in OBST:
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = sx / 2, sy / 2
        ext = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        i0, i1 = int((cx - ext[0] - X0) / RES), int((cx + ext[0] - X0) / RES) + 1
        j0, j1 = int((cy - ext[1] - Y0) / RES), int((cy + ext[1] - Y0) / RES) + 1
        for i in range(max(0, i0), min(W, i1)):
            wx = X0 + (i + 0.5) * RES - cx
            for j in range(max(0, j0), min(H, j1)):
                wy = Y0 + (j + 0.5) * RES - cy
                lx, ly = c * wx + s * wy, -s * wx + c * wy
                if abs(lx) <= hx + RES / 2 and abs(ly) <= hy + RES / 2:
                    img[(H - 1 - j) * W + i] = 0
    pgm = os.path.join(SRC, "yektemak_navigation", "maps", "port.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(bytes(img))
    with open(os.path.join(SRC, "yektemak_navigation", "maps", "port.yaml"), "w") as f:
        f.write(f"image: port.pgm\nmode: trinary\nresolution: {RES}\norigin: [{X0}, {Y0}, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n\n"
                "# tools/generate_port.py ile dunya geometrisinden uretildi (LiDAR yuksekligindeki engeller).\n")
    # Panel icin saydam PNG
    try:
        from PIL import Image
        im = Image.frombytes("L", (W, H), bytes(img))
        rgba = Image.new("RGBA", (W, H))
        rgba.putdata([(38, 48, 58, 255) if v < 50 else (255, 255, 255, 0) for v in im.getdata()])
        pub = os.path.join(ROOT, "dashboard", "public")
        rgba.save(os.path.join(pub, "port.png"))
        json.dump({"origin": [X0, Y0], "resolution": RES, "width": W, "height": H},
                  open(os.path.join(pub, "port.json"), "w"))
    except ImportError:
        print("PIL yok: panel haritasi uretilmedi")
    print(f"harita: {W}x{H}, {len(OBST)} engel")


def write_world():
    pallets = ""
    for i, n in enumerate(INITIAL_PALLETS, 1):
        x, y, yaw, _ = STATIONS[n]
        pallets += include("yektemak_pallet", f"pallet_{i}", x, y, yaw)
    world = f"""<?xml version="1.0"?>
<!-- tools/generate_port.py ile uretildi. Elle duzenlemek yerine uretici dosyayi degistirin. -->
<sdf version="1.6">
  <world name="yektemak_port">
    <gravity>0 0 -9.81</gravity>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{DATUM["lat"]}</latitude_deg>
      <longitude_deg>{DATUM["lon"]}</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
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
      <ambient>0.65 0.65 0.68 1</ambient>
      <background>0.62 0.75 0.90 1</background>
      <shadows>true</shadows>
      <sky><clouds><speed>4</speed></clouds></sky>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.3 -0.85</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          {mat((0.33, 0.34, 0.36))}
        </visual>
      </link>
    </model>
{static_model("port_yard", container_blocks() + stuffing_containers() + quay() + perimeter() + office_and_chargers())}{static_model("port_markings", markings())}{pallets}
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-14 -38 32 0 0.62 0.9</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
"""
    open(os.path.join(SRC, "yektemak_gazebo", "worlds", "port.world"), "w").write(world)


def write_stations():
    d = {
        "scenario": "port",
        "map": "port",
        "datum": DATUM,
        "stations": {n: {"x": x, "y": y, "yaw": round(yaw, 5), "type": k}
                     for n, (x, y, yaw, k) in STATIONS.items()},
        "initial_pallets": {n: f"pallet_{i}" for i, n in enumerate(INITIAL_PALLETS, 1)},
        "zones": {k: {"label": lbl, "rect": list(r)} for k, (lbl, r) in ZONES.items()},
        "gnss_zones": GNSS_ZONES,
    }
    path = os.path.join(SRC, "yektemak_fleet", "config", "port_stations.yaml")
    with open(path, "w") as f:
        f.write("# tools/generate_port.py ile uretildi. Dunya ile ayni olmali.\n")
        yaml.safe_dump(d, f, sort_keys=False, allow_unicode=True)


if __name__ == "__main__":
    write_world()
    rasterize()
    write_stations()
    print("tamam")
