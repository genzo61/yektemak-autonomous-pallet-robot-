# EVRENOS real-vehicle operations

These commands are for ROS 2 Humble. Run them from a terminal on the vehicle
computer. Keep the physical emergency stop reachable whenever power can reach
the motors.

## Terminal setup

Run this in every new terminal:

```bash
cd ~/yektemak_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Create a persistent map directory once:

```bash
mkdir -p "$HOME/evrenos_maps"
```

## 1. Safe system start

Start the complete mapping stack with every physical data driver disabled and
motor motion latched off:

```bash
ros2 launch yektemak_bringup evrenos_real_mapping.launch.py \
  start_lidar_driver:=false \
  start_bno055_driver:=false \
  motion_enabled:=false
```

To inspect connected sensors while keeping all motor output disabled, select
each driver explicitly. For example, with both sensors connected:

```bash
ros2 launch yektemak_bringup evrenos_real_mapping.launch.py \
  start_lidar_driver:=true \
  start_bno055_driver:=true \
  motion_enabled:=false
```

Never change the launch defaults to enable motion.

## 2. Real mapping

First start mapping with motion disabled and complete the preflight checklist:

```bash
ros2 launch yektemak_bringup evrenos_real_mapping.launch.py \
  start_lidar_driver:=true \
  start_bno055_driver:=true \
  motion_enabled:=false
```

After preflight, stop that launch with Ctrl+C and deliberately restart it with
the low-speed limits and motor authority enabled:

```bash
ros2 launch yektemak_bringup evrenos_real_mapping.launch.py \
  start_lidar_driver:=true \
  start_bno055_driver:=true \
  motion_enabled:=true \
  max_linear_speed:=0.10 \
  max_angular_speed:=0.20
```

If the BNO055 has not yet passed its checks, keep
`start_bno055_driver:=false`. Mapping still requires valid LiDAR and `/scan`
data.

## 3. Save the map

While the mapping launch is running, use a second configured terminal:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/evrenos_maps/evrenos_site"
```

Confirm that both `evrenos_site.yaml` and its referenced image file exist
before stopping the mapping launch.

## 3b. Watch the map from a laptop (RViz), keep Jetson headless

`evrenos_real_mapping.launch.py` and `evrenos_real_nav2.launch.py` never
start RViz on the vehicle computer on purpose - running RViz locally on the
Jetson competes with SLAM/Nav2 for CPU/GPU and visibly lags. Run RViz on a
separate laptop on the same network instead:

```bash
source /opt/ros/humble/setup.bash
# If the laptop has this workspace built too, also:
# source ~/yektemak_ws/install/setup.bash

rviz2 -d ~/yektemak_ws/install/yektemak_bringup/share/yektemak_bringup/rviz/evrenos_real_mapping.rviz
```

This config opens with Fixed Frame `map` and the `Map`, `LaserScan`,
`RobotModel`, and `TF` displays already added (a disabled `PointCloud2`
display pointed at `/unilidar/cloud` is included too, toggle it on only
when you specifically need to debug the raw cloud - keeping it off saves
bandwidth over the link).

For RViz on the laptop to actually see topics published on the Jetson:

- Both machines must be on the **same `ROS_DOMAIN_ID`**. If neither side
  sets the environment variable, both default to `0` and this is a
  non-issue; if either machine exports `ROS_DOMAIN_ID` in its `.bashrc`,
  make sure the other one matches.
- Both machines need to be reachable for ROS 2's DDS discovery traffic
  (multicast on the local network by default). If they only ever see each
  other's `SSH`/`ping` traffic but no topics show up in
  `ros2 topic list` from the laptop, check that neither machine's firewall
  (e.g. `ufw`) is blocking UDP, and that you are on the vehicle's local
  Wi-Fi/router and not routed through Tailscale for this step (Tailscale
  is fine for SSH, but DDS multicast discovery generally does not traverse
  it without extra configuration).
- Quick check before opening RViz:
  `ros2 topic list` from the laptop should already show `/map`, `/scan`,
  `/tf` while the mapping launch is running on the Jetson. If it shows
  nothing, fix discovery before troubleshooting RViz itself.

## 4. Nav2 localization with the saved map

Start localization with motion disabled first:

```bash
ros2 launch yektemak_bringup evrenos_real_nav2.launch.py \
  map:="$HOME/evrenos_maps/evrenos_site.yaml" \
  start_lidar_driver:=true \
  start_bno055_driver:=true \
  motion_enabled:=false
```

After localization and the full preflight have passed, stop with Ctrl+C and
restart explicitly with low-speed motor authority:

```bash
ros2 launch yektemak_bringup evrenos_real_nav2.launch.py \
  map:="$HOME/evrenos_maps/evrenos_site.yaml" \
  start_lidar_driver:=true \
  start_bno055_driver:=true \
  motion_enabled:=true \
  max_linear_speed:=0.10 \
  max_angular_speed:=0.20
```

## 5. Low-speed manual motion test

Clear the test area, keep a spotter at the physical emergency stop, and start
only the real base at the enforced low-speed limits:

```bash
ros2 launch yektemak_bringup evrenos_real_base.launch.py \
  motion_enabled:=true \
  max_linear_speed:=0.10 \
  max_angular_speed:=0.20 \
  start_bno055_driver:=false
```

In a second configured terminal, a short 0.05 m/s forward pulse can be issued
as follows. The watchdog stops the vehicle if commands cease; the explicit zero
is still required as normal procedure:

```bash
ros2 topic pub --rate 10 --times 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

For supervised driving, `teleop_twist_keyboard` may be used; the bridge still
clamps commands to the launch limits:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 6. Explicit motor enable

Motor motion is disabled by default in every EVRENOS real launch. Enabling it
requires restarting the selected launch with this explicit argument:

```text
motion_enabled:=true
```

Runtime parameter changes are intentionally not allowed to enable motion.

## 7. Emergency stop

Use the physical emergency stop first whenever available. It is the primary
stop and does not depend on Linux, ROS, USB, or the network.

From any configured ROS terminal, latch software motion off immediately:

```bash
ros2 param set /evrenos_stm32_bridge motion_enabled false
```

The bridge immediately sends redundant zero commands and continues enforcing
zero output. Runtime re-enabling is rejected. As an additional zero command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Then stop teleoperation or Nav2 command sources and press Ctrl+C in the launch
terminal. Do not release the physical emergency stop until the cause is known
and the complete preflight has been repeated.

## Real-vehicle preflight order

0. Confirm the RC transmitter's mode switch is set to **manual**, not
   autonomous, before any mapping run where a person will drive by RC.
   Mode arbitration happens in firmware on an SBUS channel; if the switch
   is left in the autonomous position, RC stick inputs are ignored even
   though the RC link itself looks fine on the transmitter. Also confirm
   the RC link stays powered and connected for the whole session -
   firmware fail-stops the vehicle on SBUS signal loss even in autonomous
   mode, independent of the Jetson/ROS watchdog.
1. Secure the test area, use wheel stands for the first powered test, appoint
   a spotter, and verify the physical emergency stop actually removes motor
   power.
2. Start the selected stack with `motion_enabled:=false`. Confirm it reports
   `MOTION DISABLED - zero velocity only`.
3. Check the STM32 path and bridge:
   `ls -l /dev/ttyACM0`, `ros2 node info /evrenos_stm32_bridge`, and
   `ros2 param get /evrenos_stm32_bridge motion_enabled`.
4. Check LiDAR input and its frame:
   `ros2 topic hz /unilidar/cloud` and
   `ros2 topic echo /unilidar/cloud --field header --once`.
5. Check BNO055 data, `imu_link`, calibration, signs, and covariance fields:
   `ros2 topic hz /imu/data` and `ros2 topic echo --once /imu/data`.
6. With the wheels safely raised and motion still disabled, rotate each wheel
   by hand and verify all encoder signs in `ros2 topic echo /wheel/odom`.
7. Check the TF branches:
   `ros2 run tf2_ros tf2_echo base_footprint lidar_link` and
   `ros2 run tf2_ros tf2_echo base_link imu_link`.
8. Check EKF output and frame IDs:
   `ros2 topic hz /odometry/filtered` and
   `ros2 topic echo --once /odometry/filtered`.
9. Check scan conversion and visualization:
   `ros2 topic hz /scan` and confirm obstacles appear correctly in RViz.
10. Confirm `motion_enabled` is still `false`, lower the vehicle only when the
    area is clear, and enable motion solely by an intentional launch restart.
