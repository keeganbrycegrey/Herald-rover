# HERALD codebase

## AN ENTRY TO THE DIVISION SCIENCE AND TECHNOLOGY FAIR - DIVISION OF QUEZON - REGION IV-A CALABARZON - PHILIPPINES

HERALD is a ROS 2 Humble rover stack with an ESP32 motor controller, a Raspberry Pi 4 compute node, a 2D LiDAR, and an MPU6050 IMU.

## Layout

- `esp32_firmware/skid_steering_firmware.ino`: ESP32 motor PWM, encoder interrupts, UART protocol, and command watchdog.
- `herald_ws/src/herald_bringup`: ROS 2 package containing the rover nodes, launch files, configuration, and dashboard.
- `field_testing/record_trial.sh`: records the three field-test topic sets.
- `field_testing/plot_raw_vs_corrected.py`: plots Battery 1 raw and corrected scan data.

## ROS 2 nodes

| Node | Inputs | Outputs |
|---|---|---|
| `motor_bridge_node` | `/cmd_vel_mux_out`, ESP32 UART encoder packets | `/wheel_odom`, `odom -> base_link` TF |
| `tilt_compensation_node` | `/scan`, MPU6050 over I2C | `/scan_corrected`, `/scan_level`, `/imu/data_raw`, `/imu/filtered_pitch_roll` |
| `floor_drop_detector_node` | `/scan_corrected` | `/floor_drop_alert`, `/floor_drop_measured_range` |
| `hazard_classification_node` | `/scan_corrected`, floor-drop topics | `/hazards`, `~/herald_hazard_log.csv` |
| `autonomy_manager_node` | `/map`, `/autonomy/command`, `/autonomy/goal_pose` | `/autonomy/state`, `/autonomy/mode`, `/manual_override_lock`, `/autonomy/current_frontier_target` |

`tilt_compensation_node` rotates each valid LiDAR return using `R = Ry(pitch) * Rx(roll)`, including the fixed 15-degree mount angle. It publishes the full corrected point cloud and a filtered `LaserScan` for SLAM/Nav2. The filtered scan keeps points in `-0.05 m <= z <= 0.5 m`.

`floor_drop_detector_node` scans the full corrected cloud and raises an alert when `z < -(mount_height_m + delta_margin_m)`. `exclude_radius_m` is disabled by default.

`hazard_classification_node` uses geometric break-point clustering. It labels clusters as `wall`, `obstacle`, or `unclassified_structure`, and adds a `void` entry while the floor-drop alert is active. It does not perform semantic recognition.

`autonomy_manager_node` supports `explore_only`, `navigate_only`, and `explore_then_navigate`. It finds frontier clusters from `/map`, sends Nav2 `NavigateToPose` goals, blacklists failed frontier locations temporarily, and supports manual override through `twist_mux`.

## Shared configuration

`config/sensor_extrinsics.yaml` is loaded by both sensor nodes:

- `phi_mount_deg: 15.0`
- `mount_height_m: 0.21`

The current launch configuration also sets `delta_margin_m: 0.10` and `exclude_radius_m: 0.0`.

The motor bridge launch defaults are:

- serial port `/dev/ttyAMA0` at `115200` baud
- command input `/cmd_vel_mux_out`
- wheel separation `0.60 m`
- wheel radius `0.04 m`
- encoder PPR `11`
- gearbox ratio `56.0`
- maximum linear speed `0.5 m/s`
- maximum PWM `255`
- TF publishing enabled

Verify the physical wheel dimensions, encoder count, gearbox ratio, sensor height, and serial device before field testing.

## ESP32 firmware

The firmware groups three left motors and three right motors through one BTS7960 driver per side. It uses ESP32 LEDC PWM at 20 kHz with 8-bit duty values.

The Raspberry Pi connects to ESP32 `Serial2` at 115200 baud:

- ESP32 TX2 GPIO17 -> Raspberry Pi RX GPIO15
- ESP32 RX2 GPIO16 <- Raspberry Pi TX GPIO14
- shared ground

Commands from the Pi use `M<left_pwm>,<right_pwm>\n`. Encoder reports use `E<left_ticks>,<right_ticks>,<dt_ms>\n` and are sent every 50 ms. The command watchdog stops both sides after 500 ms without a valid command.

## Build and launch

```bash
cd ~/herald_ws
colcon build --symlink-install --packages-select herald_bringup
source install/setup.bash
```

Install the external ROS packages used by the full stack:

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-rosbridge-suite ros-humble-twist-mux
```

Base bringup starts the LiDAR driver when `ydlidar_ros2_driver` is installed, tilt compensation, floor-drop detection, hazard classification, and the motor bridge:

```bash
ros2 launch herald_bringup herald_bringup.launch.py
```

For standalone motor testing without `twist_mux`, override the command topic:

```bash
ros2 launch herald_bringup herald_bringup.launch.py cmd_vel_topic:=/cmd_vel
```

Full autonomy starts base bringup, `slam_toolbox`, Nav2, the autonomy manager, `twist_mux`, and rosbridge on port 9090:

```bash
ros2 launch herald_bringup herald_autonomy.launch.py
```

Open `herald_ws/src/herald_bringup/dashboard/index.html` on the laptop and connect it to `ws://<pi-hostname-or-ip>:9090`.

## Autonomy modes

- `explore_only`: explore frontiers, then return to `IDLE`.
- `navigate_only`: wait for a goal on `/autonomy/goal_pose`.
- `explore_then_navigate`: explore first, then wait for a goal.

Commands on `/autonomy/command` are:

- `set_mode:<mode>`
- `start_explore`
- `start_navigate`
- `cancel`
- `manual_override_on`
- `manual_override_off`

The default mode is `explore_then_navigate`. Manual teleoperation uses `/cmd_vel_teleop`; Nav2 uses `/cmd_vel`; `twist_mux` publishes the final motor command on `/cmd_vel_mux_out`.

## Field testing

```bash
./field_testing/record_trial.sh battery1 <trial_id>
./field_testing/record_trial.sh battery2 <trial_id>
./field_testing/record_trial.sh battery3 <trial_id>
```

- Battery 1: `/scan`, `/scan_corrected`, `/imu/data_raw`, `/imu/filtered_pitch_roll`
- Battery 2: Battery 1 topics plus `/cmd_vel_mux_out`
- Battery 3: `/scan`, `/scan_corrected`, `/floor_drop_alert`, `/floor_drop_measured_range`

The script refuses to overwrite an existing bag directory. Plot Battery 1 data with:

```bash
python3 field_testing/plot_raw_vs_corrected.py <bag_path> <output_png>
```

## Current limitations

- Motor control is open loop; encoder feedback is used for odometry, not PID speed control.
- The LiDAR has a fixed downward mount, so `/scan_level` has uneven obstacle-height coverage across azimuth.
- The corrected cloud and filtered scan preserve the raw `frame_id`; adding a TF-based mount correction later would require removing the manual rotation to avoid double correction.
- The hazard node is geometric only and cannot identify object semantics without a camera and vision model.
- There is no INA219 current telemetry, so motor stalls require video or other external review.
- The fixed-frontier selector chooses the largest frontier cluster, not necessarily the closest one.
