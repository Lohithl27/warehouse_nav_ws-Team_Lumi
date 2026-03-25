# warehouse_nav_ws (Lumi Vs Dyam)
Autonomous Self-Driving Indoor Navigation for Warehouse Inventory Management
# WarehouseNav AMR — Complete Working Prototype
## MAHE Mobility Challenge 2026 | Robotics Track | ARTPARK, IISc
### Built from: bhavikmk/warehousebot + kuralme/rb1_ros2_description + AAISHAA1585/Autonomous_Indoor_Robot-ROS2

---

## Workspace Structure

```
warenav_ws/
└── src/
    ├── warenav_description/          ← Package 1: Robot URDF + Gazebo World
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── urdf/
    │   │   └── warenav_amr.urdf.xacro   RB1-style circular robot
    │   ├── worlds/
    │   │   └── warehouse.world          20×15m, 18 shelves, pedestrian
    │   ├── launch/
    │   │   └── display.launch.py        URDF visualiser only
    │   └── rviz/
    │       └── warenav.rviz             Pre-configured dashboard
    │
    ├── warenav_nav/                  ← Package 2: Navigation Nodes + Configs
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── warenav_nav/
    │   │   ├── frontier_explorer.py     Autonomous open-world exploration
    │   │   ├── mission_controller.py    Warehouse patrol state machine
    │   │   ├── obstacle_detector.py     LiDAR proximity detection
    │   │   └── inventory_logger.py      JSON log per shelf
    │   └── config/
    │       ├── ekf.yaml                 IMU + odometry fusion
    │       ├── slam_toolbox.yaml        Lifelong async SLAM
    │       └── nav2_params.yaml         MPPI + SmacPlanner + costmaps
    │
    └── warenav_bringup/              ← Package 3: All Launch Files
        ├── package.xml
        ├── CMakeLists.txt
        └── launch/
            ├── simulation.launch.py     ★ MAIN — one command = everything
            ├── slam_mapping.launch.py   Manual mapping with teleop
            └── nav2_localization.launch.py  Navigate on saved map
```

---

##  Step 1 — Install Prerequisites (Ubuntu 22.04)

```bash
# ROS2 Humble (if not installed)
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip

# Python dependencies
pip3 install numpy scipy

# Source ROS2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Init rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
```

---

##  Step 2 — Build the Workspace

```bash
# Place the warenav_ws folder in your home directory
cd ~/warenav_ws

# Install all ROS2 deps
rosdep install --from-paths src --ignore-src -r -y

# Build (symlink-install = Python changes take effect immediately)
colcon build --symlink-install

# Source workspace
source install/setup.bash
echo "source ~/warenav_ws/install/setup.bash" >> ~/.bashrc
```

### Verify the build

```bash
ros2 pkg list | grep warenav
# Expected output:
# warenav_bringup
# warenav_description
# warenav_nav
```

---

##  Step 3 — Run the Simulation

### Option A — Full autonomous simulation (recommended)

```bash
ros2 launch warenav_bringup simulation.launch.py
```

**Startup sequence (watch the terminal):**
- `t=0s` — Gazebo opens, robot spawns at charging dock
- `t=5s` — EKF sensor fusion + SLAM toolbox start
- `t=10s` — Nav2 (MPPI planner + costmaps) starts
- `t=18s` — Frontier explorer + Mission controller + Obstacle detector start
- `t=20s` — RViz2 opens with full dashboard

**Wait for this message in terminal:**
```
[slam_toolbox]: Message Filter dropping message: frame 'laser_link'...
```
That means SLAM is receiving scans. Wait ~5 more seconds then start the mission.

### Option B — Auto-start (robot begins exploring automatically)

```bash
ros2 launch warenav_bringup simulation.launch.py auto_start:=true
```

### Option C — Skip exploration (go straight to shelf patrol)

```bash
ros2 launch warenav_bringup simulation.launch.py \
  auto_start:=true skip_exploration:=true
```

---

##  Step 4 — Control the Mission

Open a **new terminal** and source:

```bash
source ~/warenav_ws/install/setup.bash
```

### Start the full warehouse patrol

```bash
ros2 service call /start_mission std_srvs/srv/Trigger
```

The robot will:
1. **Phase 1 — Explore:** Autonomously map the entire warehouse using frontier exploration
2. **Phase 2 — Patrol:** Visit all 18 shelf locations (Rows A–F)
3. **Phase 3 — Return:** Drive back to charging dock
4. **Done:** Mission log saved to `/tmp/mission_*.json`

### Navigate to a specific shelf directly

```bash
# Row B, Shelf 2
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"B2\"}"}'

# Far corner (Row F, Shelf 3) — hardest navigation challenge
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"F3\"}"}'

# Return to dock
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"DOCK\"}"}'

# Go to packing station
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"PACKING\"}"}'
```

### Control services

```bash
# Stop mission immediately
ros2 service call /stop_mission std_srvs/srv/Trigger

# Pause mid-mission
ros2 service call /pause_mission std_srvs/srv/Trigger

# Resume after pause
ros2 service call /resume_mission std_srvs/srv/Trigger

# Get full mission report
ros2 service call /mission_report std_srvs/srv/Trigger
```

---

##  Step 5 — Monitor Everything

```bash
# Mission status (updates every 3s)
ros2 topic echo /mission_status

# Exploration status
ros2 topic echo /explore_status

# Obstacle detection
ros2 topic echo /obstacle_detected

# Inventory being logged at each shelf
ros2 topic echo /inventory_log

# LiDAR scan rate (should be ~10 Hz)
ros2 topic hz /scan

# EKF output rate (should be ~30 Hz)
ros2 topic hz /odometry/filtered

# Map coverage (watch it grow in RViz2)
# Grey = unknown, White = mapped free space, Black = obstacles
```

---

##  Step 6 — Two-Phase Workflow (Map then Navigate)

### Phase 1 — Build the map manually

```bash
# Terminal 1: Launch SLAM + Gazebo + Teleop
ros2 launch warenav_bringup slam_mapping.launch.py

# Terminal 2: Drive the robot around the warehouse
# An xterm window opens with teleop — use arrow keys:
# i = forward  ,  = backward  j = left  l = right  k = stop
```

**In RViz2:** watch the grey map fill in as you drive.

```bash
# Terminal 3: Save the map when >90% coverage
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/tmp/warenav_map'}}"
# Creates: /tmp/warenav_map.yaml + /tmp/warenav_map.pgm
```

### Phase 2 — Navigate on saved map

```bash
ros2 launch warenav_bringup nav2_localization.launch.py \
  map:=/tmp/warenav_map.yaml

# Start patrol (skip exploration — map already exists)
ros2 service call /start_mission std_srvs/srv/Trigger
```

---

##  What Each RViz2 Display Shows

| Display | Color | Meaning |
|---|---|---|
| Map (SLAM) | Grey/White/Black | Grey=unknown, White=free, Black=obstacle |
| Global Path | Bright green | Nav2 SmacPlanner route to next waypoint |
| Local Path (MPPI) | Cyan | MPPI best trajectory (updates 20Hz) |
| MPPI Trajectories | Orange lines | All 1000 sampled candidate paths |
| Frontier Markers | Cyan spheres | Unexplored boundaries (exploration phase) |
| Frontier Markers | Orange sphere | Currently targeted frontier |
| Waypoint Markers | Blue = pending | Shelf locations yet to visit |
| Waypoint Markers | Green = done | Successfully visited shelves |
| Waypoint Markers | Red = failed | Shelves that could not be reached |
| Waypoint Markers | Orange = current | Shelf being navigated to right now |
| Obstacle Markers | Red/Orange sphere | Detected obstacle from LiDAR |
| Camera Feed | Bottom panel | Live robot camera view |

---

##  Key Commands Quick Reference

```bash
# ── BUILD ─────────────────────────────────────────────
colcon build --symlink-install && source install/setup.bash

# ── FULL LAUNCH ───────────────────────────────────────
ros2 launch warenav_bringup simulation.launch.py

# ── MISSION CONTROL ───────────────────────────────────
ros2 service call /start_mission  std_srvs/srv/Trigger
ros2 service call /stop_mission   std_srvs/srv/Trigger
ros2 service call /pause_mission  std_srvs/srv/Trigger
ros2 service call /resume_mission std_srvs/srv/Trigger
ros2 service call /mission_report std_srvs/srv/Trigger

# ── GO TO SHELF ───────────────────────────────────────
ros2 topic pub --once /nav_cmd std_msgs/msg/String '{"data": "{\"shelf\": \"D2\"}"}'

# ── SAVE MAP ──────────────────────────────────────────
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/tmp/warenav_map'}}"

# ── MANUAL DRIVE ──────────────────────────────────────
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel

# ── MONITOR ───────────────────────────────────────────
ros2 topic echo /mission_status
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 node list
ros2 run tf2_tools view_frames

# ── EMERGENCY RESET ───────────────────────────────────
pkill -f ros2 && pkill -f gzserver && pkill -f gzclient
```

---

##  Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Map stays grey | SLAM not receiving /scan | `ros2 topic hz /scan` — should be 10Hz |
| Robot not moving | Nav2 not active | `ros2 node list \| grep nav2` — wait 15s |
| EKF not publishing | Missing /odom or /imu | `ros2 topic hz /odom /imu/data` |
| "Goal rejected" | Nav2 costmap not ready | Wait 20s after launch, check Nav2 nodes |
| Robot spinning | MPPI tuning | Reduce `vx_max: 0.3` in nav2_params.yaml |
| Gazebo crash | Memory/GPU | Add `gz_verbose:=false`, remove pedestrian actor |
| colcon build fails | Missing deps | `rosdep install --from-paths src --ignore-src -r -y` |
| RViz "Fixed frame error" | Frame not published yet | Wait for SLAM to start, set Fixed Frame to `map` |
| Frontier not found | Map too small | Drive manually to open more space first |

---

##  Hardware Migration Guide

When you have the physical Jetson + RPLiDAR + MPU-9250:

```bash
# Install hardware drivers
sudo apt install ros-humble-rplidar-ros ros-humble-usb-cam

# RPLiDAR node (replaces Gazebo ray sensor)
ros2 run rplidar_ros rplidarNode \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200

# USB Camera
ros2 run usb_cam usb_cam_node_exe \
  --ros-args -p video_device:=/dev/video0

# Everything else (EKF, SLAM, Nav2, mission nodes) is IDENTICAL
# Just set use_sim_time: false in all configs
```

---

##  Reference Repositories Used

| Repository | What was taken |
|---|---|
| bhavikmk/warehousebot | 3-package workspace structure, ROS2 Humble pattern |
| kuralme/rb1_ros2_description | Circular chassis URDF design, sensor placement |
| AAISHAA1585/Autonomous_Indoor_Robot-ROS2 | Nav2 action client pattern for waypoint navigation |
| RobotecAI/agentic-mobile-manipulator | Agentic task sequencing for mission controller |

---

*WarehouseNav AMR | MAHE Mobility Challenge 2026 | Built with ROS2 Humble*
