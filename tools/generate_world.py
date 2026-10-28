"""Depo dunyasi, palet ve raf modellerinin SDF dosyalarini uretir.

Dunyayi degistirmek icin bu dosyadaki olculeri/konumlari duzenleyip calistirin:
    python3 tools/generate_world.py
Istasyon konumlarini degistirirseniz yektemak_tasks/config/mission.yaml'i de
guncelleyin ve haritayi yeniden cikarin.
"""
import os
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "yektemak_gazebo")

def box_inertia(m, x, y, z):
    return m*(y*y+z*z)/12, m*(x*x+z*z)/12, m*(x*x+y*y)/12

def mat(rgb, emissive=None):
    r, g, b = rgb
    e = f"<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>" if emissive else ""
    return (f"<material><ambient>{r*0.8:.2f} {g*0.8:.2f} {b*0.8:.2f} 1</ambient>"
            f"<diffuse>{r} {g} {b} 1</diffuse><specular>0.1 0.1 0.1 1</specular>{e}</material>")

def boxpart(name, pose, size, rgb, collide=True, surface=""):
    p = " ".join(f"{v:.4g}" for v in pose)
    s = " ".join(f"{v:.4g}" for v in size)
    out = ""
    if collide:
        out += (f'<collision name="{name}_col"><pose>{p} 0 0 0</pose>'
                f'<geometry><box><size>{s}</size></box></geometry>{surface}</collision>\n')
    out += (f'<visual name="{name}_vis"><pose>{p} 0 0 0</pose>'
            f'<geometry><box><size>{s}</size></box></geometry>{mat(rgb)}</visual>\n')
    return out

def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w").write(text)

def model_config(name, desc):
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>Yektemak</name></author>
  <description>{desc}</description>
</model>
"""

# ---------------------------------------------------------------- PALET
WOOD = (0.72, 0.47, 0.22); WOOD_D = (0.55, 0.33, 0.14)
CARD = (0.78, 0.60, 0.38); CARD_D = (0.66, 0.49, 0.29); WRAP = (0.85, 0.88, 0.92)
fr = ('<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>')
deck_surf = ('<surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>')
parts = ""
# İki taşıyıcı ray: robot aralarındaki 60 cm boşluktan paletin altına girer
parts += boxpart("left_runner", (0, 0.34, 0.115), (1.10, 0.08, 0.23), WOOD_D, surface=fr)
parts += boxpart("right_runner", (0, -0.34, 0.115), (1.10, 0.08, 0.23), WOOD_D, surface=fr)
parts += boxpart("top_deck", (0, 0, 0.25), (1.20, 0.80, 0.04), WOOD, surface=deck_surf)
# Tabla üzerindeki tahta çizgileri (sadece görsel)
for i, y in enumerate([-0.33, -0.165, 0.0, 0.165, 0.33]):
    parts += boxpart(f"plank_{i}", (0, y, 0.2705), (1.20, 0.13, 0.001), WOOD, collide=False)
# Yük: koli yığını (streç film sarılı)
parts += boxpart("cargo", (0, 0, 0.27 + 0.30), (1.10, 0.74, 0.60), WRAP)
for ix, x in enumerate([-0.275, 0.275]):
    for iy, y in enumerate([-0.185, 0.185]):
        for iz, z in enumerate([0.42, 0.72]):
            parts += boxpart(f"box_{ix}{iy}{iz}", (x, y, z), (0.53, 0.355, 0.28),
                             CARD if (ix + iy + iz) % 2 == 0 else CARD_D, collide=False)
m = 55.0
ixx, iyy, izz = box_inertia(m, 1.2, 0.8, 0.85)
pallet = f"""<?xml version="1.0"?>
<!-- Yektemak palet modeli: 1.20 x 0.80 m, iki raylı, üstünde yük.
     Rayların iç aralığı 0.60 m; robot (tekerlerle 0.48 m) bu boşluğa girer.
     Tablanın alt yüzü yerden 0.23 m yukarıda. -->
<sdf version="1.6">
  <model name="yektemak_pallet">
    <static>false</static>
    <link name="pallet_link">
      <inertial>
        <pose>0 0 0.40 0 0 0</pose>
        <mass>{m}</mass>
        <inertia><ixx>{ixx:.3f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.3f}</iyy><iyz>0</iyz><izz>{izz:.3f}</izz></inertia>
      </inertial>
{parts}    </link>
  </model>
</sdf>
"""
write(f"{ROOT}/models/yektemak_pallet/model.sdf", pallet)
write(f"{ROOT}/models/yektemak_pallet/model.config",
      model_config("yektemak_pallet", "Yuklu, iki rayli palet"))

# ---------------------------------------------------------------- RAF
STEEL = (0.20, 0.32, 0.62); BEAM = (0.95, 0.45, 0.08); SHELF = (0.62, 0.64, 0.66)
GUARD = (0.95, 0.80, 0.05)
parts = ""
L, D = 4.0, 1.10
for sx in (-1, 1):
    for sy in (-1, 1):
        parts += boxpart(f"post_{sx}{sy}", (sx*1.95, sy*0.55, 1.50), (0.10, 0.10, 3.00), STEEL)
for lvl, z in enumerate([0.75, 1.60, 2.45]):
    for sy in (-1, 1):
        parts += boxpart(f"beam_{lvl}_{sy}", (0, sy*0.55, z - 0.07), (3.80, 0.08, 0.12), BEAM)
    parts += boxpart(f"shelf_{lvl}", (0, 0, z), (3.80, 1.00, 0.03), SHELF)
    # Raf üstü koliler
    for k, x in enumerate([-1.30, 0.0, 1.30]):
        if (lvl + k) % 3 == 2:
            continue
        h = 0.45 if (lvl + k) % 2 else 0.60
        parts += boxpart(f"stock_{lvl}_{k}", (x, 0, z + 0.015 + h/2), (1.05, 0.75, h),
                         CARD if k % 2 else CARD_D)
# Zemin seviyesinde koruma bariyeri: LiDAR (11 cm) rafın sınırını görsün
for sy in (-1, 1):
    parts += boxpart(f"guard_long_{sy}", (0, sy*0.60, 0.15), (4.0, 0.06, 0.20), GUARD)
for sx in (-1, 1):
    parts += boxpart(f"guard_end_{sx}", (sx*2.00, 0, 0.15), (0.06, 1.26, 0.20), GUARD)
rack = f"""<?xml version="1.0"?>
<!-- Depo rafı: 4.0 x 1.1 m, 3 kat. Zemindeki sarı bariyerler LiDAR yüksekliğinde. -->
<sdf version="1.6">
  <model name="yektemak_warehouse_rack">
    <static>true</static>
    <link name="rack_link">
{parts}    </link>
  </model>
</sdf>
"""
write(f"{ROOT}/models/yektemak_warehouse_rack/model.sdf", rack)
write(f"{ROOT}/models/yektemak_warehouse_rack/model.config",
      model_config("yektemak_warehouse_rack", "Uc katli depo rafi"))

# ---------------------------------------------------------------- DÜNYA
WALL = (0.80, 0.80, 0.76)
X0, X1, Y0, Y1 = -2.0, 18.0, -6.0, 6.0
walls = ""
t, h = 0.20, 2.5
walls += boxpart("wall_south", ((X0+X1)/2, Y0 - t/2, h/2), (X1-X0+2*t, t, h), WALL)
walls += boxpart("wall_north", ((X0+X1)/2, Y1 + t/2, h/2), (X1-X0+2*t, t, h), WALL)
walls += boxpart("wall_west", (X0 - t/2, 0, h/2), (t, Y1-Y0, h), WALL)
walls += boxpart("wall_east", (X1 + t/2, 0, h/2), (t, Y1-Y0, h), WALL)
# Kolonlar
cols = ""
for i, (x, y) in enumerate([(8.0, 0.0), (8.0, 4.9), (8.0, -4.9)]):
    cols += boxpart(f"column_{i}", (x, y, h/2), (0.40, 0.40, h), (0.70, 0.70, 0.70))
    cols += boxpart(f"column_guard_{i}", (x, y, 0.15), (0.50, 0.50, 0.30), (0.95, 0.80, 0.05))

# Zemin işaretleri (çarpışmasız)
YEL = (0.98, 0.82, 0.05); GRN = (0.15, 0.70, 0.30); BLU = (0.15, 0.45, 0.90); WHT = (0.95, 0.95, 0.95)
marks = ""
def frame(name, cx, cy, sx, sy, rgb, w=0.06):
    s = ""
    s += boxpart(f"{name}_a", (cx, cy + sy/2, 0.002), (sx + w, w, 0.002), rgb, collide=False)
    s += boxpart(f"{name}_b", (cx, cy - sy/2, 0.002), (sx + w, w, 0.002), rgb, collide=False)
    s += boxpart(f"{name}_c", (cx + sx/2, cy, 0.002), (w, sy, 0.002), rgb, collide=False)
    s += boxpart(f"{name}_d", (cx - sx/2, cy, 0.002), (w, sy, 0.002), rgb, collide=False)
    return s
# Alma (yeşil) ve bırakma (mavi) istasyonları
stations = {"pick_1": (3.0, 1.5, GRN), "pick_2": (3.0, -1.5, GRN),
            "drop_1": (14.5, 1.5, BLU), "drop_2": (14.5, -1.5, BLU)}
for n, (x, y, c) in stations.items():
    marks += frame(n, x, y, 1.40, 1.00, c)
# Robot park/şarj alanı
marks += frame("home", 0.0, 0.0, 1.10, 0.80, YEL)
# Koridor kenar şeritleri
for i, y in enumerate([2.35, -2.35]):
    marks += boxpart(f"lane_{i}", (8.0, y, 0.002), (17.0, 0.08, 0.002), YEL, collide=False)

def include(uri, name, x, y, yaw=0.0):
    return (f"    <include>\n      <uri>model://{uri}</uri>\n      <name>{name}</name>\n"
            f"      <pose>{x} {y} 0 0 0 {yaw}</pose>\n    </include>\n")
racks = ""
for i, (x, y) in enumerate([(5.5, 3.0), (10.5, 3.0), (5.5, -3.0), (10.5, -3.0)]):
    racks += include("yektemak_warehouse_rack", f"rack_{i+1}", x, y)
pallets = include("yektemak_pallet", "pallet_1", 3.0, 1.5) + include("yektemak_pallet", "pallet_2", 3.0, -1.5)

world = f"""<?xml version="1.0"?>
<!--
  Yektemak depo dünyası (20 x 12 m)
    - Robot park alanı : (0, 0)
    - Alma istasyonları (yeşil)  : pick_1 (3.0, 1.5), pick_2 (3.0, -1.5)
    - Bırakma istasyonları (mavi): drop_1 (14.5, 1.5), drop_2 (14.5, -1.5)
    - 4 raf, 3 kolon, ana koridor genişliği ~4.7 m
  Konumlar yektemak_tasks/config/mission.yaml ile aynı olmalı.
-->
<sdf version="1.6">
  <world name="yektemak_warehouse">

    <gravity>0 0 -9.81</gravity>

    <!-- /gazebo/get_entity_state ve /gazebo/set_entity_state servisleri -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>20.0</update_rate>
    </plugin>

    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.70 0.75 0.80 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          {mat((0.55, 0.57, 0.58))}
        </visual>
      </link>
    </model>

    <model name="warehouse_building">
      <static>true</static>
      <link name="link">
{walls}{cols}      </link>
    </model>

    <model name="floor_markings">
      <static>true</static>
      <link name="link">
{marks}      </link>
    </model>

{racks}
{pallets}
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-4.5 -10.5 9.0 0 0.62 0.85</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
"""
write(f"{ROOT}/worlds/warehouse.world", world)
print("ok")
