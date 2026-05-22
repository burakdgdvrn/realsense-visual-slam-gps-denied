# GPS/Visual Odometry Switching with Smooth Recovery for UAV Localization in Simulated GPS-Denied Environment

> **Graduation Thesis Project** — GPS/Visual Odometry Switching with Smooth Recovery for Leader-Follower UAV Formations under Simulated GPS Outage Conditions.

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic%2011-orange)](http://gazebosim.org/)
[![RTAB-Map](https://img.shields.io/badge/RTAB--Map-Visual%20Odometry-green)](http://introlab.github.io/rtabmap/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow)](https://python.org/)

---

## 📋 Overview

This project implements a **priority-based localization switching node** for a leader-follower UAV formation. The system selects between two position estimation sources — GPS odometry and Visual Odometry — depending on GPS signal availability. When GPS is lost (simulated as a binary outage event), the system transitions to **RTAB-Map Frame-to-Map Visual Odometry** with coordinate frame alignment. When GPS returns, a time-based smooth recovery (bumpless transfer) is performed to avoid position discontinuities.

> **Architectural Note:** This system performs **sensor arbitration** (priority-based source selection), not statistical sensor fusion. Unlike Kalman filter or complementary filter approaches, the two sources are never combined simultaneously. Instead, one source is selected as the active localization input at any given time, based on GPS signal availability.

### What This Project Does

- **Priority-Based Switching** — GPS ↔ Visual Odometry source selection with coordinate frame alignment
- **Smooth Recovery** — Time-based linear interpolation (T=2.0s) from VO back to GPS when signal returns (bumpless transfer)
- **Leader-Follower Formation** — Geometric V-formation with 1 leader + 2 followers (rigid offset, no inter-agent communication)
- **Simulated GPS Outage** — Binary signal loss toggler for GPS-denied scenario testing
- **Academic Metrics** — ATE, RPE, RMSE, and yaw error recording with per-mode breakdown

### What This Project Does NOT Do

- ❌ Autonomous navigation (no waypoint planner, no obstacle avoidance)
- ❌ Distributed multi-agent SLAM or cooperative localization
- ❌ SLAM-corrected global pose feedback into the localization loop
- ❌ Statistical sensor fusion (no Kalman filter, no complementary filter)
- ❌ Realistic GPS error modeling (no multipath, HDOP, or satellite geometry)
- ❌ Dynamic flight physics (kinematic simulation, no IMU/wind/vibration)

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         SYSTEM OVERVIEW                             │
│                                                                     │
│  ┌──────────────┐    ┌───────────────────┐    ┌──────────────────┐  │
│  │   Teleop     │───▶│   Formation       │───▶│   Kinematic      │  │
│  │  (cmd_vel)   │    │   Controller      │    │   Physics        │  │
│  │              │    │  (V-formation)    │    │  (Gazebo teleport)│  │
│  └──────────────┘    └───────┬───────────┘    └──────────────────┘  │
│                              │                                      │
│                       ┌──────▼───────────┐                          │
│                       │  GPS Simulator   │                          │
│                       │  (Noisy sensor:  │                          │
│                       │   σ=1.5m + bias) │                          │
│                       └──────┬───────────┘                          │
│                              │                                      │
│  ┌──────────────┐     ┌──────▼───────────┐    ┌──────────────────┐  │
│  │  GPS Outage  │────▶│   GPS/VO         │◀───│  RTAB-Map        │  │
│  │  Simulator   │     │   SWITCHER       │    │  Visual Odometry │  │
│  │  (Bool toggle)│    │  (source select) │    │  (Frame-to-Map)  │  │
│  └──────────────┘     └──────┬───────────┘    └──────────────────┘  │
│                              │                                      │
│                       ┌──────▼───────────┐                          │
│                       │   Metrics        │                          │
│                       │   Recorder       │                          │
│                       │  (ATE,RPE,RMSE)  │                          │
│                       └──────────────────┘                          │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  RTAB-Map SLAM (runs independently, map→odom TF only)      │   │
│  │  Not connected to switching pipeline                        │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
realsense_vslam/
├── scripts/
│   ├── gps_vo_switcher.py       # Core: GPS/VO priority-based switching with smooth recovery
│   ├── formation_controller.py  # V-formation geometry (leader-follower state machine)
│   ├── kinematic_physics.py     # Gazebo entity positioning via SetEntityState
│   ├── odom_broadcaster.py      # Simulated noisy GPS odometry (σ=1.5m + random walk)
│   ├── gps_outage_sim.py        # Interactive GPS outage toggle (binary on/off)
│   ├── metrics_recorder.py      # CSV logger: ATE, RPE, RMSE, mode transitions
│   └── plot_results.py          # Result graph generator
├── launch/
│   ├── system_bringup.launch.py # Single entry point: Gazebo + Engine + SLAM
│   ├── single_gazebo.launch.py  # Leader-only Gazebo (for solo testing)
│   ├── master_gazebo.launch.py  # Full formation: Leader + 2 Followers
│   ├── flight_engine.launch.py  # Core nodes: control, physics, odom, switcher, metrics
│   └── slam_rtabmap.launch.py   # RTAB-Map VO + SLAM + Visualization
├── models/
│   ├── master_uav/model.sdf     # Leader drone with RGB-D depth camera
│   └── slave_uav/model.sdf      # Follower drone (no camera, formation only)
├── worlds/
│   └── vslam.world              # ISCAS Museum environment for VO feature tracking
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

### Quick Start

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

**Terminal 3** — Toggle GPS outage (press ENTER):
```bash
ros2 run realsense_vslam gps_outage_sim.py
```

### Generate Result Graphs

```bash
python3 src/realsense-visual-slam-gps-denied/realsense_vslam/scripts/plot_results.py
```

Output saved to `~/graduation_thesis/test_results/`.

---

## 🧠 GPS/VO Switching Logic

The `GpsVoSwitcher` node implements a three-mode **priority-based source selection** strategy:

| Mode | Trigger | Source Used |
|------|---------|-------------|
| **GPS** | GPS signal active | GPS odometry (noisy) passed through directly |
| **VO** | GPS signal lost | RTAB-Map Frame-to-Map VO, aligned to GPS frame via rigid body transform |
| **RECOVERY** | GPS signal restored | Time-based interpolation from VO position to GPS position (T=2.0s) |

> This is **sensor arbitration**, not sensor fusion. Only one source is active at a time.

### GPS → VO Transition (Coordinate Frame Alignment)

When GPS is lost, the offset between the last known GPS position and the current VO position is computed using a 2D rigid body transformation:

```
offset_yaw = atan2(sin(gps_yaw - vo_yaw), cos(gps_yaw - vo_yaw))
offset_x = gps_x - (vo_x·cos(θ) - vo_y·sin(θ))
offset_y = gps_y - (vo_x·sin(θ) + vo_y·cos(θ))
```

Subsequent VO frames are transformed into the GPS coordinate frame using this offset, ensuring position continuity (graceful degradation).

### VO → GPS Recovery (Bumpless Transfer)

When GPS returns, the system blends VO and GPS positions over 2 seconds using time-based interpolation:

```
α += dt / T    (T = 2.0 seconds, time-based increment)
position = lerp(vo_aligned, gps_position, α)
yaw = slerp(vo_aligned_yaw, gps_yaw, α)    # shortest-path angular interpolation
```

This prevents position discontinuities that would occur with an instant switch.

---

## 📊 Metrics

The `MetricsRecorder` node records the following to timestamped CSV files:

| Metric | Description | Reference |
|--------|-------------|-----------|
| **ATE** | Euclidean distance: estimated position vs ground truth | `/formation/master_target` (ground truth) |
| **RPE** | Translation and rotation error between consecutive frames | Frame-to-frame delta comparison |
| **Yaw Error** | Angular deviation from ground truth orientation | Wrapped ±π difference |
| **GPS ATE** | Euclidean distance: noisy GPS vs ground truth | Quantifies GPS noise level |
| **Mode Transitions** | GPS → VO → RECOVERY → GPS timestamps | Event log CSV |

Per-mode statistics (GPS / VO / RECOVERY) are computed at shutdown including min, max, mean, median, RMSE, and standard deviation.

---

## 🛠️ ROS 2 Topic Map

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/master/cmd_vel` | `Twist` | Teleop keyboard | FormationController |
| `/formation/master_target` | `Pose` | FormationController | KinematicPhysics, OdomBroadcaster, MetricsRecorder |
| `/formation/slave1_target` | `Pose` | FormationController | KinematicPhysics |
| `/formation/slave2_target` | `Pose` | FormationController | KinematicPhysics |
| `/master/gps_odom` | `Odometry` | OdomBroadcaster | GpsVoSwitcher, MetricsRecorder |
| `/rtabmap/vo` | `Odometry` | rgbd_odometry | GpsVoSwitcher, RTAB-Map SLAM |
| `/system/gps_status` | `Bool` | GpsOutageSim | GpsVoSwitcher, MetricsRecorder |
| `/master/odom` | `Odometry` | GpsVoSwitcher | MetricsRecorder |
| `/system/localization_mode` | `String` | GpsVoSwitcher | MetricsRecorder |

> **Note:** `/master/odom` is consumed only by MetricsRecorder for logging. It does not feed back into any control loop or RTAB-Map.

---

## 📝 Limitations & Scope

This section documents known limitations for academic transparency:

1. **Kinematic Simulation:** Drones are positioned via `SetEntityState` (teleportation). There is no flight dynamics, IMU, wind, or vibration modeling. Consequently, VO drift results are more optimistic than a physical system would produce, since camera frames are inherently stable.

2. **Simplified GPS Model:** GPS noise is modeled as additive Gaussian (σ=1.5m) with random walk bias (σ_step=0.02m per update). Real GPS errors (multipath, HDOP variation, satellite geometry) are not simulated.

3. **Binary GPS Outage:** GPS loss is modeled as an instantaneous binary event (on/off). Gradual signal degradation, partial outage, or spoofing scenarios are not implemented.

4. **VO Only, Not SLAM-Corrected:** The switching node uses raw Frame-to-Map Visual Odometry (`/rtabmap/vo`), not the globally optimized SLAM pose. RTAB-Map SLAM runs concurrently for mapping, but its loop closure corrections do not feed back into the localization output.

5. **No Autonomous Navigation:** There is no waypoint planner, mission controller, or obstacle avoidance. Flight is controlled manually via teleop.

6. **Geometric Formation Only:** Follower drones have no cameras or independent localization. They maintain position through rigid geometric offsets from the leader — this is leader-follower geometric coupling, not distributed cooperative localization.

7. **No Closed-Loop Control:** The `/master/odom` output is used only for metrics recording. It does not feed back into drone flight control. In a real system, this output would be provided to the autopilot.

8. **Priority-Based Switching, Not Fusion:** The system does not combine GPS and VO signals simultaneously. It selects one source at a time based on GPS availability. This is distinct from Kalman filter or complementary filter-based sensor fusion approaches.

---

## 👤 Author

**Burak Dağdeviren**  
Graduation Thesis — 2026

---

## 📄 License

MIT License — See [LICENSE](LICENSE) for details.
