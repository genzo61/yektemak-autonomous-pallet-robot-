# YEKTEMAK — Otonom Palet Taşıma Robotu (ROS 2 Humble + Gazebo)

Depoda kendi konumunu bulan, paleti LiDAR ile tespit edip altına giren, kaldırıp hedefe taşıyan ve bırakan otonom bir AMR'nin simülasyonu.

![Depo genel görünüm](docs/images/depo_genel.jpg)

| Robot | Yüklü taşıma | SLAM haritası |
|---|---|---|
| ![Robot](docs/images/robot.jpg) | ![Palet taşıma](docs/images/palet_tasima.jpg) | ![Harita](docs/images/slam_harita.png) |

## Ne yapıyor?

Tek komutla robot şu döngüyü insan müdahalesi olmadan tamamlar: park alanından çıkar, Nav2 ile paletin önüne gider, ön LiDAR ile paletin iki rayını bulup konumunu ölçer, rayların arasından paletin altına girer, lift'i kaldırır, yük moduna geçer (büyük footprint, %60 hız), paleti bırakma istasyonuna taşır, indirir, altından çıkar ve sıradaki göreve geçer. Bütün görevler bitince parka döner.

Başsız (headless) Gazebo testinde iki palet görevinin ölçülen sonuçları şöyle: paletin altına girme hatası 0.5 cm'nin altında, bırakma hatası 1–7 cm, parka dönüş hatası yaklaşık 6 cm.

## Kurulum (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-laser-filters \
  ros-humble-teleop-twist-keyboard python3-numpy

# Repo kökü doğrudan workspace'tir (içinde src/ var)
git clone https://github.com/genzo61/yektemak-autonomous-pallet-robot-.git ~/yektemak_ws
cd ~/yektemak_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Kullanım

**Tam otonom demo (önerilen):**

```bash
ros2 launch yektemak_bringup sim_mission.launch.py
# başka terminalde durumu izle:
ros2 topic echo /mission/status
```

Çalışırken yeni görev eklemek için:

```bash
ros2 topic pub --once /mission/request std_msgs/msg/String "{data: 'pick_1:drop_2'}"
```

**Sadece navigasyon** (RViz'de "Nav2 Goal" ile hedef verebilirsin):

```bash
ros2 launch yektemak_bringup sim_navigation.launch.py
```

**Haritayı yeniden çıkarma:**

```bash
ros2 launch yektemak_bringup sim_mapping.launch.py
ros2 run yektemak_tasks map_explorer          # robotu rotada otomatik gezdirir
# (ya da: ros2 run teleop_twist_keyboard teleop_twist_keyboard)
ros2 run nav2_map_server map_saver_cli -f src/yektemak_navigation/maps/warehouse
```

Haritayı yeniden çıkarırsan palet istasyonlarını haritadan temizlemen gerekir (aşağıdaki "Tasarım kararları" bölümüne bak).

**Elle test komutları:**

```bash
ros2 launch yektemak_gazebo sim.launch.py                    # sadece Gazebo + robot
ros2 launch yektemak_description display.launch.py           # sadece RViz'de model
ros2 topic pub --once /lift_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.10]}"
ros2 service call /pallet_attacher/attach std_srvs/srv/SetBool "{data: true}"
```

`gui:=false` ile Gazebo arayüzü, `rviz:=false` ile RViz kapatılabilir.

## Paketler

| Paket | İçerik |
|---|---|
| `yektemak_description` | URDF/Xacro: gövde, 4 teker, lift (prismatic), ön LiDAR, IMU, ros2_control |
| `yektemak_gazebo` | Depo dünyası, palet ve raf modelleri, simülasyon launch'ı |
| `yektemak_gazebo_plugins` | Palet bağlayıcı Gazebo plugin'i (C++) |
| `yektemak_navigation` | LiDAR filtresi, slam_toolbox, Nav2 parametreleri, harita, RViz |
| `yektemak_tasks` | LiDAR palet tespiti, docking, görev yöneticisi, haritalama gezgini |
| `yektemak_bringup` | Üç ana launch dosyası (mapping / navigation / mission) |

Ek olarak `tools/generate_world.py` depo dünyasını, palet ve raf modellerini üretir. Dünyayı değiştirmek istersen ölçüleri bu dosyada düzenleyip çalıştırman yeterli.

## Robot

| Özellik | Değer |
|---|---|
| Gövde | 0.80 × 0.38 × 0.10 m, 80 kg, yerden 4 cm |
| Teker | 4 adet, Ø 0.20 m (dış genişlik 0.48 m) |
| Lift | 0.68 × 0.30 m platform, 0–10 cm strok, PID pozisyon kontrol |
| LiDAR | Ön tampon, yerden 11 cm, ±100°, 20 m |
| IMU | Gövde merkezi, 100 Hz |
| Palet | 1.20 × 0.80 m, iki raylı (iç aralık 0.60 m), yükle 55 kg |

## Tasarım kararları

**LiDAR neden alçakta?** Robot paleti kaldırınca palet tablası robotun üstünü kapatır. LiDAR yukarıda olsaydı yüklüyken kör kalırdı. Yerden 11 cm yükseklikte palet tablasının altında kalıyor ve yüklüyken de görmeye devam ediyor. Bu yükseklikte rafların sadece direkleri görüneceği için raflara zemin seviyesinde sarı koruma bariyerleri eklendi (gerçek depolarda da bulunur).

**İki scan konusu.** `/scan_raw` ham veridir ve docking bunu kullanır, çünkü palet raylarını görmesi gerekir. `/scan` ise robotun etrafındaki palet izdüşümü kutusu çıkarılmış halidir ve SLAM, AMCL ile costmap bunu kullanır. Böylece taşınan paletin rayları "engel" sanılmaz.

**Palet istasyonları haritada boş.** Paletler hareket ettiği için haritaya statik engel olarak girmemeli. Aksi halde robot palet altındayken "başlangıç engel içinde" hatası alınır. Paletleri navigasyon sırasında LiDAR anlık olarak görür.

**Neden planar_move + palet bağlayıcı?** Gazebo'da 4 tekerli skid-steer teması kararsızdır, bu yüzden robotu `gazebo_ros_planar_move` sürer (navigasyon tarafında `vy = 0` tutulur, yani robot diferansiyel sürüşlü gibi davranır). Bu plugin linklerin hızını dışarıdan zorladığı için sürtünme paleti taşıyamıyor. Bu yüzden lift kalkınca `yektemak_gazebo_plugins` paleti eklemle kilitler. Gerçek robotta bu işi yük ağırlığı ve sürtünme yapar, servis yoksa görev yöneticisi bu adımı atlar.

**Docking.** LiDAR iki ray ucunu (aralarında yaklaşık 68 cm) bulur ve aralarındaki açıklığın boş olduğunu doğrular. Bundan palet merkezini ve yönünü hesaplar. Robot palet ekseni boyunca Stanley benzeri bir kontrolcüyle ilerler. Raylara girmeden önce bir hizalama kapısı vardır: yanal hata 3.5 cm'den ya da açı hatası 3°'den büyükse robot geri çekilir, paleti yeniden ölçer ve tekrar dener. Algoritmanın birim testleri `yektemak_tasks/test` altında, `python3 -m pytest src/yektemak_tasks/test` ile çalışır.

## Bilinen tuzaklar (bunlarla karşılaştık)

1. **URDF yorumlarında `: ` ve `'` kullanma.** gazebo_ros2_control robot tanımını YAML olarak parse eder. `<!-- IMU: gövde -->` gibi masum bir yorum bile "Couldn't parse parameter override rule" hatasıyla her şeyi çökertir.
2. **Launch argüman adları çakışabilir.** `gzserver.launch.py` da `params_file` adlı bir argüman tanımlıyor. ROS 2'de launch argümanları include'lar arasında paylaşıldığı için senin argümanın sessizce ezilir. Bu yüzden argümanlar `slam_params_file` ve `nav2_params_file` olarak adlandırıldı.
3. **Lift PID'i yükü kaldıracak kadar güçlü olmalı.** Düşük kazançla lift 55 kg yük altında sadece 4 cm kalkıyordu. Kazançlar `yektemak_ros2_control.xacro` içinde.
4. **planar_move ile teker sürtünmesi düşük olmalı**, aksi halde tekerler fren yapar ve yüklü robot devrilmeye çalışır.

## Gerçek robota geçiş için notlar

Simülasyondaki arayüzler gerçek robotta da aynı kalacak şekilde seçildi: `/cmd_vel`, `/odom`, `/scan_raw`, `/imu/data` ve `/lift_controller/commands`. Gerçek tarafta yapılacaklar şunlar:
- Motor ve lift sürücüleri için bir `ros2_control` hardware interface'i yazmak (CANopen veya RS485 sürücüler ya da bir STM32 köprüsü).
- Gerçek LiDAR'ın sürücüsünü `/scan_raw` olarak yayınlamak.
- Odometri ile IMU'yu `robot_localization` EKF ile birleştirmek.
- Güvenlik katmanını (acil durdurma, güvenlik lazer alanları, watchdog) eklemek.
