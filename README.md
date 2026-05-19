# 🛰️ GPS-Denied Visual SLAM for Autonomous Drone Swarm Navigation

> **Graduation Thesis Project** — Autonomous Master-Slave drone swarm navigation using Visual SLAM in GPS-denied environments.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](http://gazebosim.org/)
[![RTAB-Map](https://img.shields.io/badge/RTAB--Map-SLAM-green)](http://introlab.github.io/rtabmap/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow)](https://python.org/)

---

## 📋 Overview

This project demonstrates a **GPS-denied drone swarm navigation system** that seamlessly transitions between GPS-based and Vision-based localization. When a GPS signal is lost (e.g., due to jamming or indoor operation), the system automatically switches to **RTAB-Map Visual Odometry** to maintain autonomous flight and formation.

### Key Features

- **Hybrid Localization** — Automatic GPS ↔ Visual Odometry switching with smooth recovery
- **V-Formation Swarm** — Master-Slave architecture with 1 master + 2 slave UAVs
- **GPS Cyber-Attack Simulation** — Real-time GPS jamming/spoofing simulator
- **Automated Test Scenarios** — Scripted fault-injection tests with metrics recording
- **Publication-Quality Visualization** — Auto-generated trajectory plots and error analysis

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        SYSTEM OVERVIEW                          │
│                                                                 │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────┐  │
│  │   Teleop /   │───▶│   Formation      │───▶│  Kinematic   │  │
│  │  Auto Test   │    │   Controller     │    │   Physics    │  │
│  │  (cmd_vel)   │    │  (V-formation)   │    │  (Gazebo)    │  │
│  └──────────────┘    └──────┬───────────┘    └──────────────┘  │
│                             │                                   │
│                      ┌──────▼───────────┐                      │
│                      │  GPS Broadcaster  │                      │
│                      │  (Odom source)    │                      │
│                      └──────┬───────────┘                      │
│                             │                                   │
│  ┌──────────────┐    ┌──────▼───────────┐    ┌──────────────┐  │
│  │  GPS Jammer  │───▶│   HYBRID         │◀───│  RTAB-Map    │  │
│  │  (Cyber Atk) │    │   LOCALIZER      │    │  Visual Odom │  │
│  └──────────────┘    │  GPS↔VO Fusion   │    │  (rgbd_odom) │  │
│                      └──────┬───────────┘    └──────────────┘  │
│                             │                                   │
│                      ┌──────▼───────────┐    ┌──────────────┐  │
│                      │   RTAB-Map SLAM  │───▶│  Metrics     │  │
│                      │   (3D Mapping)   │    │  Recorder    │  │
│                      └──────────────────┘    └──────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
realsense_vslam/
├── scripts/
│   ├── hybrid_localizer.py      # Core: GPS/VO fusion with smooth recovery
│   ├── formation_controller.py  # V-formation swarm control (state machine)
│   ├── kinematic_physics.py     # Gazebo entity positioning via SetEntityState
│   ├── odom_broadcaster.py      # Simulated GPS odometry publisher
│   ├── gps_jammer.py            # Interactive GPS cyber-attack simulator
│   ├── automated_flight_test.py # 7-phase autonomous flight test
│   ├── fault_scenario.py        # 9-phase GPS-denial fault injection test
│   ├── metrics_recorder.py      # CSV logger for position error & mode transitions
│   └── plot_results.py          # Thesis-quality graph generator
├── launch/
│   ├── system_bringup.launch.py # Single entry point: Gazebo + Engine + SLAM
│   ├── single_gazebo.launch.py  # Master-only Gazebo (for solo SLAM testing)
│   ├── master_gazebo.launch.py  # Full swarm: Master + 2 Slaves
│   ├── flight_engine.launch.py  # Core nodes: control, physics, odom, localizer, metrics
│   └── slam_rtabmap.launch.py   # RTAB-Map VO + SLAM + Visualization
├── models/
│   ├── master_uav/model.sdf     # Master drone with RGB-D depth camera
│   └── slave_uav/model.sdf      # Slave drone (no camera, formation only)
├── worlds/
│   └── vslam.world              # Custom 16x12m textured indoor warehouse
├── CMakeLists.txt
└── package.xml
```

---

## 🔧 Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Gazebo Classic 11**
- **RTAB-Map ROS2** (`ros-humble-rtabmap-ros`)
- **Teleop Twist Keyboard** (`ros-humble-teleop-twist-keyboard`)

### Installation

```bash
# Install dependencies
sudo apt install ros-humble-rtabmap-ros ros-humble-gazebo-ros-pkgs \
                 ros-humble-teleop-twist-keyboard python3-matplotlib

# Clone and build
mkdir -p ~/graduation_thesis/src && cd ~/graduation_thesis/src
git clone https://github.com/burakdgdvrn/realsense-visual-slam-gps-denied.git
cd ~/graduation_thesis
colcon build
source install/setup.bash
```

---

## 🚀 Usage

### Quick Start (All-in-one)

**Terminal 1** — Launch everything:
```bash
source ~/graduation_thesis/install/setup.bash
ros2 launch realsense_vslam system_bringup.launch.py
```

**Terminal 2** — Fly with keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/master/cmd_vel
```

**Terminal 3** — GPS jammer (press ENTER to toggle):
```bash
ros2 run realsense_vslam gps_jammer.py
```

### Automated Fault Scenario Test

Runs a full GPS → VO → Recovery cycle automatically (no manual input needed):

```bash
# Terminal 1: Launch system
ros2 launch realsense_vslam system_bringup.launch.py

# Terminal 2: Run automated fault scenario
ros2 run realsense_vslam fault_scenario.py
```

### Generate Result Graphs

```bash
python3 src/realsense-visual-slam-gps-denied/realsense_vslam/scripts/plot_results.py
```

Output saved to `~/graduation_thesis/test_results/`:
- `test_results_*.png` — Combined 4-panel result figure
- `trajectory_*.png` — XY trajectory map
- `position_error_*.png` — Position error time series
- `mode_timeline_*.png` — Localization mode transitions

---

## 🧠 Hybrid Localization Algorithm

The `HybridLocalizer` node implements a three-mode sensor fusion strategy:

| Mode | Trigger | Method |
|------|---------|--------|
| **GPS** | GPS signal active | Direct GPS odometry pass-through |
| **VO** | GPS signal lost | Rotation matrix-based VO→GPS frame alignment |
| **RECOVERY** | GPS signal restored | Linear interpolation (lerp) from VO to GPS |

### Frame Alignment Math (GPS → VO Transition)

When GPS is lost, the offset between GPS and VO frames is computed:

```
offset_yaw = gps_yaw - vo_yaw
offset_x = gps_x - (vo_x·cos(θ) - vo_y·sin(θ))
offset_y = gps_y - (vo_x·sin(θ) + vo_y·cos(θ))
```

The VO position is then transformed into the GPS frame using a 2D rigid body transformation.

### Smooth Recovery (VO → GPS Transition)

When GPS returns, the system blends VO and GPS positions over ~2 seconds:

```
blended = lerp(vo_position, gps_position, α)
α += 0.05 per step  →  100% GPS in ~20 steps
```

---

## 🌍 Test Environment

Custom **16m × 12m enclosed indoor warehouse** designed for GPS-denied SLAM testing:

- **Textured walls** — Brick and tile patterns for visual feature extraction
- **Partition walls** — Create corridors and rooms for complex navigation
- **Pillars** — Rocky-textured SLAM landmarks at key positions
- **Shelves & Crates** — Wood-pallet textured obstacles at various angles
- **Barrels** — Cylindrical geometry for feature diversity
- **Wall panels** — At drone flight height (1m) for direct camera visibility

All surfaces use Gazebo built-in textures to ensure RTAB-Map Visual Odometry can extract and track features during GPS-denied operation.

---

## 📊 Metrics & Results

The system records the following metrics to timestamped CSV files:

| Metric | Description |
|--------|-------------|
| **Position Error** | Euclidean distance between fused and GPS positions |
| **Mode Transitions** | GPS → VO → RECOVERY → GPS event timestamps |
| **Formation Error** | Slave deviation from expected formation position |
| **Trajectory** | Full XY position trace with mode color coding |

---

## 🛠️ ROS 2 Topic Map

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/master/cmd_vel` | `Twist` | Teleop / AutoTest | FormationController |
| `/swarm/master_target` | `Pose` | FormationController | KinematicPhysics, OdomBroadcaster |
| `/swarm/slave1_target` | `Pose` | FormationController | KinematicPhysics |
| `/swarm/slave2_target` | `Pose` | FormationController | KinematicPhysics |
| `/master/gps_odom` | `Odometry` | OdomBroadcaster | HybridLocalizer |
| `/rtabmap/vo` | `Odometry` | rgbd_odometry | HybridLocalizer |
| `/system/gps_status` | `Bool` | GpsJammer | HybridLocalizer |
| `/master/odom` | `Odometry` | HybridLocalizer | RTAB-Map SLAM |
| `/system/localization_mode` | `String` | HybridLocalizer | MetricsRecorder |

---

## 📝 Notes

- This system uses a **kinematic simulation approach** for upper-level coordination validation rather than low-level flight dynamics modeling
- The drone model uses `<kinematic>1</kinematic>` with gravity disabled for deterministic positioning
- Aerodynamic tilt animations are disabled to prevent SLAM map distortion
- The depth camera simulates an **Intel RealSense** sensor (640×480, 30 FPS, 86° FOV, 0.1-10m range)

---

## 👤 Author

**Burak Dağdeviren**  
Graduation Thesis — 2026

---

## 📄 License

This project is developed for academic purposes as part of a graduation thesis.
