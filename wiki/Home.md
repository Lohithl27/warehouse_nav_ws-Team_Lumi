# WarehouseNav AMR — Project Wiki

> **MAHE Mobility Challenge 2026 | Robotics Track | ARTPARK, IISc**
> Team Lumi — Autonomous Self-Driving Indoor Navigation for Warehouse Inventory Management

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [ROS2 Packages](#3-ros2-packages)
4. [Navigation Stack](#4-navigation-stack)
5. [Key Nodes](#5-key-nodes)
6. [ROS2 Topics & Services](#6-ros2-topics--services)
7. [Installation & Setup](#7-installation--setup)
8. [Running the Simulation](#8-running-the-simulation)
9. [Mission Control](#9-mission-control)
10. [Monitoring & Visualization](#10-monitoring--visualization)
11. [Two-Phase Workflow (Manual Map + Navigate)](#11-two-phase-workflow-manual-map--navigate)
12. [Hardware Migration Guide](#12-hardware-migration-guide)
13. [Configuration Reference](#13-configuration-reference)
14. [Troubleshooting](#14-troubleshooting)
15. [Quick Command Reference](#15-quick-command-reference)
16. [Reference Repositories](#16-reference-repositories)

---

## 1. Project Overview

**WarehouseNav AMR** is a complete autonomous mobile robot (AMR) system built on **ROS2 Humble** that performs end-to-end warehouse inventory management in simulation and is designed to migrate easily to physical hardware.

### What the Robot Does

| Phase | Description |
|-------|-------------|
| **Explore** | Autonomously maps the entire warehouse using frontier-based exploration |
| **Patrol** | Visits all 18 shelf locations (Rows A–F, 3 shelves each) |
| **Scan** | Logs inventory JSON records at each shelf |
| **Return** | Drives back to the charging dock |

### Key Features

- 🗺️ **Autonomous SLAM mapping** via SLAM Toolbox (lifelong async mode)
- 🧭 **Frontier-based exploration** — discovers unknown space without pre-built maps
- 🤖 **MPPI motion planning** — Model Predictive Path Integral controller (1000 trajectory samples/cycle)
- 🚧 **Real-time obstacle detection** via 360° LiDAR
- 📦 **Per-shelf inventory logging** to JSON files
- 🔄 **Full state-machine mission control** with start/stop/pause/resume
- 🎯 **Direct shelf navigation** via ROS2 topics
- 🖥️ **Pre-configured RViz2 dashboard** with live path, frontier, and waypoint visualizations

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                         │
│  warehouse.world (20×15m, 18 shelves, pedestrian actor)     │
│  warenav_amr.urdf.xacro (RB1-style circular chassis)        │
└──────────────────────┬──────────────────────────────────────┘
                       │ /scan (LiDAR)  /odom  /imu/data
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                 Sensor Fusion Layer                          │
│  robot_localization EKF  (/odom + /imu → /odometry/filtered)│
└──────────────────────┬──────────────────────────────────────┘
                       │ /odometry/filtered
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    SLAM Layer                                │
│  slam_toolbox (lifelong async mode) → /map (OccupancyGrid)  │
└──────────────────────┬──────────────────────────────────────┘
                       │ /map
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                Navigation Stack (Nav2)                       │
│  SmacPlannerHybrid (global) + MPPI (local) + costmaps       │
│  Action server: /navigate_to_pose                           │
└──────────────────────┬──────────────────────────────────────┘
                       │ NavigateToPose action
          ┌────────────┼────────────────┐
          ▼            ▼                ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────────┐
│  Frontier    │ │   Mission    │ │    Obstacle      │
│  Explorer   │ │  Controller  │ │    Detector      │
│ (mapping)   │ │ (state mach) │ │  (LiDAR safety)  │
└──────────────┘ └──────────────┘ └──────────────────┘
                       │
                       ▼
              ┌──────────────────┐
              │ Inventory Logger │
              │ (JSON per shelf) │
              └──────────────────┘
```

---

## 3. ROS2 Packages

The workspace (`warenav_ws/`) contains three packages under `src/`:

### `warenav_description` — Robot & World

| File | Purpose |
|------|---------|
| `urdf/warenav_amr.urdf.xacro` | RB1-style circular robot URDF with LiDAR, camera, IMU |
| `worlds/warehouse.world` | 20×15m Gazebo world with 18 shelves and pedestrian actor |
| `launch/display.launch.py` | URDF-only visualizer (no navigation) |
| `rviz/warenav.rviz` | Pre-configured RViz2 dashboard |

### `warenav_nav` — Navigation Nodes & Configs

| File | Purpose |
|------|---------|
| `warenav_nav/frontier_explorer.py` | Autonomous frontier-based exploration |
| `warenav_nav/mission_controller.py` | 4-phase warehouse mission state machine |
| `warenav_nav/obstacle_detector.py` | LiDAR proximity detection with emergency stop |
| `warenav_nav/inventory_logger.py` | Per-shelf JSON inventory logger |
| `config/ekf.yaml` | IMU + odometry EKF fusion configuration |
| `config/slam_toolbox.yaml` | Lifelong async SLAM configuration |
| `config/nav2_params.yaml` | MPPI + SmacPlanner + costmap configuration |

### `warenav_bringup` — Launch Files

| File | Purpose |
|------|---------|
| `launch/simulation.launch.py` | ⭐ **MAIN** — one command launches everything |
| `launch/slam_mapping.launch.py` | Manual mapping with teleop keyboard |
| `launch/nav2_localization.launch.py` | Navigate on a previously saved map |

---

## 4. Navigation Stack

### Sensor Fusion — Extended Kalman Filter

The EKF node (`robot_localization`) fuses wheel odometry and IMU data at **30 Hz**:

- **Input 1:** `/odom` — wheel encoder odometry (x, y, yaw, velocities)
- **Input 2:** `/imu/data` — angular velocity + linear acceleration
- **Output:** `/odometry/filtered` — smooth, drift-corrected pose estimate

### SLAM — SLAM Toolbox (Lifelong Async Mode)

- Builds an `OccupancyGrid` map on the fly from LiDAR scans
- Supports map saving and reload for the two-phase workflow
- Grey = unknown | White = free space | Black = obstacles

### Path Planning — SmacPlanner (Global)

- Uses **Hybrid-A\*** search (`nav2_smac_planner/SmacPlannerHybrid`)
- Respects differential-drive kinematic constraints (Dubin curves)
- Plans through the full global costmap with 72-bin angle quantization

### Motion Control — MPPI (Local)

- **Model Predictive Path Integral** controller
- Evaluates **1000 trajectory samples** per 50ms cycle
- Maximizes a reward function across 8 critic terms:
  - `ConstraintCritic`, `GoalCritic`, `GoalAngleCritic`, `PreferForwardCritic`
  - `CostCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`
- Max forward speed: **0.5 m/s** | Max rotation: **1.9 rad/s**

### Costmaps

| Costmap | Purpose | Key Layers |
|---------|---------|-----------|
| Global | Long-range planning | StaticLayer + ObstacleLayer + InflationLayer |
| Local | Real-time obstacle avoidance | VoxelLayer + InflationLayer (rolling 4×4m window) |

Robot radius: **0.25 m** | Inflation radius: **0.55 m**

---

## 5. Key Nodes

### `frontier_explorer` — Autonomous Exploration

**Algorithm:**
1. Subscribe to `/map` (OccupancyGrid)
2. Find all **frontier cells**: FREE cells (0) adjacent to UNKNOWN cells (−1)
3. Cluster adjacent frontier cells into groups
4. Score each cluster: `score = size / (dist + 0.5)`
5. Send best cluster centroid as Nav2 goal
6. On goal completion → repeat
7. Stop when `map_coverage ≥ coverage_threshold` (default 75%) or no frontiers remain

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `coverage_threshold` | `0.75` | Stop exploring when 75% of map is known |
| `min_frontier_size` | `10` | Ignore tiny frontier clusters |
| `goal_timeout_sec` | `30.0` | Abandon a stuck goal after 30s |
| `update_rate_hz` | `2.0` | Frontier re-evaluation frequency |
| `min_frontier_dist` | `0.5` | Don't target frontiers closer than 0.5m |
| `max_frontier_dist` | `8.0` | Don't target frontiers farther than 8m |
| `auto_start` | `true` | Auto-begin after Nav2 is ready |

**Published topics:**
- `/frontier_markers` — Cyan spheres (candidates) and orange sphere (chosen) in RViz2
- `/explore_status` — JSON status including coverage %, goals sent/succeeded
- `/exploration_complete` — `Bool` signal sent to Mission Controller when done

---

### `mission_controller` — Mission State Machine

**Mission phases:**

```
IDLE ──► EXPLORING ──► PATROLLING ──► RETURNING ──► DOCKED
              │               │              │
              └───────────────┴── PAUSED ◄───┘
                                    │
                              RECOVERING ◄── (retry logic)
                                    │
                               ABORTED
```

**Shelf waypoints (18 total):**

| Row | Shelf 1 | Shelf 2 | Shelf 3 |
|-----|---------|---------|---------|
| A | (−7.4, −4.5) | (−7.4, 0.0) | (−7.4, 4.5) |
| B | (−5.0, −4.5) | (−5.0, 0.0) | (−5.0, 4.5) |
| C | (−2.6, −4.5) | (−2.6, 0.0) | (−2.6, 4.5) |
| D | (0.2, −4.5) | (0.2, 0.0) | (0.2, 4.5) |
| E | (4.2, −4.5) | (4.2, 0.0) | (4.2, 4.5) |
| F | (6.6, −4.5) | (6.6, 0.0) | (6.6, 4.5) |

Special locations: **DOCK** (−9.5, −7.2) | **PACKING** (−9.0, 6.5)

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `auto_start` | `false` | Auto-begin mission after 20s |
| `waypoint_pause_sec` | `2.0` | Dwell time at each shelf |
| `max_retries` | `3` | Retry failed waypoints this many times |
| `skip_exploration` | `false` | Jump straight to patrol (requires existing map) |

---

### `obstacle_detector` — LiDAR Safety

Monitors the forward **120° arc** of the LiDAR scan for proximity threats:

| State | Condition | Action |
|-------|-----------|--------|
| CLEAR | No obstacle nearby | Publishes `False` on `/obstacle_detected` |
| WARNING | Any obstacle < 0.8m | Publishes warning status |
| EMERGENCY | Front obstacle < 0.4m | Publishes `True` on `/obstacle_detected` |

Obstacle position published as a `Marker` in RViz2 every 10 scans.

---

### `inventory_logger` — JSON Data Recorder

Subscribes to `/inventory_log` and writes all shelf visits to:
```
/tmp/warenav_logs/inv_YYYY-MM-DD.json
```

Example log entry:
```json
{
  "ts": "2026-04-03T14:32:11.456",
  "shelf": "Row-B Shelf-2",
  "x": -5.0,
  "y": 0.0,
  "status": "scanned"
}
```

---

## 6. ROS2 Topics & Services

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → nodes | 360° LiDAR data at ~10 Hz |
| `/odom` | `nav_msgs/Odometry` | Gazebo → EKF | Wheel encoder odometry |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo → EKF | IMU acceleration + gyro |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF → Nav2 | Fused pose at ~30 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM → Nav2/Frontier | Live occupancy grid |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → Gazebo | Motor velocity commands |
| `/mission_status` | `std_msgs/String` (JSON) | MissionCtrl → user | Phase, waypoint, elapsed time |
| `/explore_status` | `std_msgs/String` (JSON) | FrontierExpl → user | Coverage %, frontiers, goals |
| `/obstacle_detected` | `std_msgs/Bool` | ObstacleDet → user | Emergency stop signal |
| `/obstacle_status` | `std_msgs/String` (JSON) | ObstacleDet → user | Level + nearest distance |
| `/inventory_log` | `std_msgs/String` (JSON) | MissionCtrl → Logger | Per-shelf scan records |
| `/exploration_complete` | `std_msgs/Bool` | FrontierExpl → MissionCtrl | Triggers patrol phase |
| `/nav_cmd` | `std_msgs/String` (JSON) | user → MissionCtrl | Direct shelf navigation |
| `/waypoint_markers` | `visualization_msgs/MarkerArray` | MissionCtrl → RViz2 | Shelf status cylinders |
| `/frontier_markers` | `visualization_msgs/MarkerArray` | FrontierExpl → RViz2 | Frontier spheres |
| `/obstacle_markers` | `visualization_msgs/MarkerArray` | ObstacleDet → RViz2 | Obstacle spheres |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/start_mission` | `std_srvs/Trigger` | Begin the warehouse patrol mission |
| `/stop_mission` | `std_srvs/Trigger` | Abort mission immediately |
| `/pause_mission` | `std_srvs/Trigger` | Pause mid-patrol |
| `/resume_mission` | `std_srvs/Trigger` | Resume from pause |
| `/mission_report` | `std_srvs/Trigger` | Get full JSON status report |
| `/slam_toolbox/save_map` | `slam_toolbox/SaveMap` | Save current SLAM map to disk |

---

## 7. Installation & Setup

### Prerequisites — Ubuntu 22.04 + ROS2 Humble

```bash
# Install ROS2 Humble packages
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

# Source ROS2 and init rosdep
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo rosdep init 2>/dev/null || true
rosdep update
```

### Build the Workspace

```bash
cd ~/warenav_ws

# Resolve all ROS2 package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with symlink install (Python changes take effect immediately)
colcon build --symlink-install

# Source the workspace
source install/setup.bash
echo "source ~/warenav_ws/install/setup.bash" >> ~/.bashrc
```

### Verify Build

```bash
ros2 pkg list | grep warenav
# Expected:
# warenav_bringup
# warenav_description
# warenav_nav
```

---

## 8. Running the Simulation

### Option A — Full Autonomous Simulation *(recommended)*

```bash
ros2 launch warenav_bringup simulation.launch.py
```

**Startup sequence:**

| Time | Event |
|------|-------|
| `t = 0s` | Gazebo opens; robot spawns at charging dock (−9.5, −7.2) |
| `t = 5s` | EKF sensor fusion starts; SLAM Toolbox begins mapping |
| `t = 10s` | Nav2 (MPPI + costmaps) starts |
| `t = 18s` | Frontier Explorer + Mission Controller + Obstacle Detector start |
| `t = 20s` | RViz2 opens with full dashboard |

Wait for this message before starting the mission:
```
[slam_toolbox]: Message Filter dropping message: frame 'laser_link'...
```
This confirms SLAM is receiving LiDAR scans. Wait ~5 more seconds.

### Option B — Auto-start Mission

```bash
ros2 launch warenav_bringup simulation.launch.py auto_start:=true
```
Robot begins exploring automatically after Nav2 is ready (~20s).

### Option C — Skip Exploration (patrol only)

```bash
ros2 launch warenav_bringup simulation.launch.py \
  auto_start:=true skip_exploration:=true
```
Jumps directly to shelf patrol. Best used with a pre-built map.

---

## 9. Mission Control

Source the workspace in every new terminal:
```bash
source ~/warenav_ws/install/setup.bash
```

### Start / Control the Mission

```bash
# Start full warehouse patrol
ros2 service call /start_mission std_srvs/srv/Trigger

# Pause mid-mission
ros2 service call /pause_mission std_srvs/srv/Trigger

# Resume after pause
ros2 service call /resume_mission std_srvs/srv/Trigger

# Stop (abort) immediately
ros2 service call /stop_mission std_srvs/srv/Trigger

# Get full mission report (JSON)
ros2 service call /mission_report std_srvs/srv/Trigger
```

### Navigate to a Specific Location

```bash
# Row B, Shelf 2
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"B2\"}"}'

# Far corner — Row F, Shelf 3
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"F3\"}"}'

# Return to charging dock
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"DOCK\"}"}'

# Go to packing station
ros2 topic pub --once /nav_cmd std_msgs/msg/String \
  '{"data": "{\"shelf\": \"PACKING\"}"}'
```

Valid shelf IDs: `A1`–`A3`, `B1`–`B3`, `C1`–`C3`, `D1`–`D3`, `E1`–`E3`, `F1`–`F3`, `DOCK`, `PACKING`

---

## 10. Monitoring & Visualization

### ROS2 Topic Monitoring

```bash
# Mission state (updates every 3s)
ros2 topic echo /mission_status

# Exploration progress
ros2 topic echo /explore_status

# Obstacle alerts
ros2 topic echo /obstacle_detected

# Inventory scan records
ros2 topic echo /inventory_log

# Check LiDAR is running (should be ~10 Hz)
ros2 topic hz /scan

# Check EKF output (should be ~30 Hz)
ros2 topic hz /odometry/filtered

# List all active nodes
ros2 node list

# Check TF transform tree
ros2 run tf2_tools view_frames
```

### RViz2 Dashboard Reference

| Display | Color/Style | Meaning |
|---------|-------------|---------|
| Map (SLAM) | Grey/White/Black | Grey=unknown, White=free, Black=obstacle |
| Global Path | Bright green line | SmacPlanner route to next waypoint |
| Local Path (MPPI) | Cyan line | MPPI best trajectory (updates 20 Hz) |
| MPPI Trajectories | Orange lines | All 1000 sampled candidate paths |
| Frontier Markers | Cyan spheres | Unexplored boundaries (exploration phase) |
| Frontier Markers | Orange sphere | Currently targeted frontier |
| Waypoint Markers | Blue cylinder | Shelf pending visit |
| Waypoint Markers | Green cylinder | Shelf successfully visited |
| Waypoint Markers | Red cylinder | Shelf navigation failed |
| Waypoint Markers | Orange cylinder | Shelf being navigated to now |
| Obstacle Markers | Red sphere | Emergency obstacle (< 0.4m) |
| Obstacle Markers | Orange sphere | Warning obstacle (< 0.8m) |
| Camera Feed | Bottom panel | Live robot camera view |

---

## 11. Two-Phase Workflow (Manual Map + Navigate)

Use this approach when you want precise control over the map before running patrol.

### Phase 1 — Build the Map Manually

```bash
# Terminal 1: Launch Gazebo + SLAM + Teleop
ros2 launch warenav_bringup slam_mapping.launch.py

# Terminal 2 (auto-opened xterm): Drive the robot
# i = forward  ,  = backward  j = left  l = right  k = stop
```

Watch the RViz2 map fill in as you drive. Target **> 90% grey area mapped**.

```bash
# Terminal 3: Save the map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/tmp/warenav_map'}}"
# Creates: /tmp/warenav_map.yaml + /tmp/warenav_map.pgm
```

### Phase 2 — Navigate on Saved Map

```bash
ros2 launch warenav_bringup nav2_localization.launch.py \
  map:=/tmp/warenav_map.yaml

# Start patrol (skipping exploration since map exists)
ros2 service call /start_mission std_srvs/srv/Trigger
```

---

## 12. Hardware Migration Guide

When migrating from simulation to a physical robot (Jetson + RPLiDAR + MPU-9250):

### Install Hardware Drivers

```bash
sudo apt install ros-humble-rplidar-ros ros-humble-usb-cam
```

### Run Hardware Nodes

```bash
# RPLiDAR (replaces Gazebo ray sensor → same /scan topic)
ros2 run rplidar_ros rplidarNode \
  --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200

# USB Camera
ros2 run usb_cam usb_cam_node_exe \
  --ros-args -p video_device:=/dev/video0
```

### Configuration Change

In all `config/*.yaml` files, change:
```yaml
use_sim_time: true
```
to:
```yaml
use_sim_time: false
```

Everything else — EKF, SLAM, Nav2, and all mission nodes — is **identical** between simulation and hardware.

---

## 13. Configuration Reference

### `ekf.yaml` — Sensor Fusion

| Parameter | Value | Description |
|-----------|-------|-------------|
| `frequency` | 30 Hz | EKF update rate |
| `two_d_mode` | true | Planar-only (no Z/roll/pitch) |
| `odom0` | `/odom` | Wheel encoder source |
| `imu0` | `/imu/data` | IMU source |

Fused state: `[x, y, yaw, vx, vy, vyaw, ax, ay]`

### `slam_toolbox.yaml` — Mapping

Lifelong async SLAM mode — suitable for both online mapping and map updating.

### `nav2_params.yaml` — Navigation

**MPPI controller key params:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `batch_size` | 1000 | Trajectory samples per cycle |
| `time_steps` | 56 | Lookahead steps |
| `model_dt` | 0.05s | Time between steps |
| `vx_max` | 0.5 m/s | Max forward speed |
| `vx_min` | −0.35 m/s | Max reverse speed |
| `wz_max` | 1.9 rad/s | Max rotation speed |

> **Tip:** If the robot spins excessively, reduce `vx_max: 0.3` in `nav2_params.yaml`.

**SmacPlanner key params:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `motion_model_for_search` | DUBIN | Differential-drive kinematic model |
| `angle_quantization_bins` | 72 | 5° angular resolution |
| `minimum_turning_radius` | 0.45 m | Minimum feasible turn |
| `max_planning_time` | 5.0s | Planning timeout |

---

## 14. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Map stays grey | SLAM not receiving `/scan` | Run `ros2 topic hz /scan` — should be ~10 Hz |
| Robot not moving | Nav2 not yet active | Run `ros2 node list \| grep nav2` — wait 15s after launch |
| EKF not publishing | Missing `/odom` or `/imu` | Run `ros2 topic hz /odom /imu/data` |
| "Goal rejected" | Nav2 costmap not ready | Wait 20s after launch; check Nav2 lifecycle nodes |
| Robot spinning in place | MPPI tuning | Reduce `vx_max: 0.3` in `nav2_params.yaml` |
| Gazebo crash | GPU/memory overload | Add `gz_verbose:=false`, remove pedestrian actor from `warehouse.world` |
| `colcon build` fails | Missing ROS2 deps | Run `rosdep install --from-paths src --ignore-src -r -y` |
| RViz "Fixed frame error" | `map` frame not yet published | Wait for SLAM to start; set Fixed Frame to `map` in RViz2 |
| Frontier not found | Map too small | Drive manually with teleop to open more free space |
| Mission stuck in EXPLORING | Frontier explorer timeout | Check `/explore_status` topic; try `skip_exploration:=true` |

### Emergency Reset

```bash
pkill -f ros2 && pkill -f gzserver && pkill -f gzclient
```

---

## 15. Quick Command Reference

```bash
# ── BUILD ──────────────────────────────────────────────────
colcon build --symlink-install && source install/setup.bash

# ── LAUNCH ─────────────────────────────────────────────────
ros2 launch warenav_bringup simulation.launch.py              # Full auto
ros2 launch warenav_bringup simulation.launch.py auto_start:=true
ros2 launch warenav_bringup slam_mapping.launch.py            # Manual mapping
ros2 launch warenav_bringup nav2_localization.launch.py map:=/tmp/warenav_map.yaml

# ── MISSION CONTROL ────────────────────────────────────────
ros2 service call /start_mission  std_srvs/srv/Trigger
ros2 service call /stop_mission   std_srvs/srv/Trigger
ros2 service call /pause_mission  std_srvs/srv/Trigger
ros2 service call /resume_mission std_srvs/srv/Trigger
ros2 service call /mission_report std_srvs/srv/Trigger

# ── DIRECT SHELF NAVIGATION ────────────────────────────────
ros2 topic pub --once /nav_cmd std_msgs/msg/String '{"data": "{\"shelf\": \"D2\"}"}'

# ── SAVE MAP ───────────────────────────────────────────────
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/tmp/warenav_map'}}"

# ── MANUAL DRIVE ───────────────────────────────────────────
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel

# ── MONITORING ─────────────────────────────────────────────
ros2 topic echo /mission_status
ros2 topic echo /explore_status
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 node list
ros2 run tf2_tools view_frames

# ── EMERGENCY RESET ────────────────────────────────────────
pkill -f ros2 && pkill -f gzserver && pkill -f gzclient
```

---

## 16. Reference Repositories

| Repository | Contribution to This Project |
|------------|------------------------------|
| [bhavikmk/warehousebot](https://github.com/bhavikmk/warehousebot) | 3-package ROS2 workspace structure and Humble patterns |
| [kuralme/rb1_ros2_description](https://github.com/kuralme/rb1_ros2_description) | Circular chassis URDF design and sensor placement |
| [AAISHAA1585/Autonomous_Indoor_Robot-ROS2](https://github.com/AAISHAA1585/Autonomous_Indoor_Robot-ROS2) | Nav2 action client pattern for waypoint navigation |
| [RobotecAI/agentic-mobile-manipulator](https://github.com/RobotecAI/agentic-mobile-manipulator) | Agentic task sequencing for the mission controller |

---

*WarehouseNav AMR | Team Lumi | MAHE Mobility Challenge 2026 | Built with ROS2 Humble*
