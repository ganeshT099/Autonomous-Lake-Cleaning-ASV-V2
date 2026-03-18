 Department of Mechatronics Engineering -SRMIST

 # Autonomous Lake Cleaning ASV

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-GPLv3-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%2022.04-orange)

Autonomous Surface Vehicle (ASV) designed for systematic lake cleaning using structured coverage planning, real-time navigation, and safety monitoring.

---

## 🚀 Project Overview

This project implements a modular ROS 2 based navigation stack for an Autonomous Lake Cleaning Surface Vehicle (ALCSV).

The system includes:

- Coverage Planning (Zig-Zag within Geofence)
- Geofencing Safety System
- Waypoint-based Navigation
- Real-time Visualization in RViz
- Modular ROS 2 Architecture

---

## 🏗 System Architecture

### Mission Layer
Handles system startup, monitoring, and mission control.

### Coverage Planning Layer
Generates structured zig-zag coverage path within a defined geofence boundary.

### Navigation & Control Layer
Executes waypoint tracking and heading control.

### Safety Layer
Monitors geofence violations and emergency conditions.

---
---

## Research Contributions

This project includes a comparative study of object detection models for floating debris detection on edge hardware.

See:
docs/papers/debris_detection_analysis.docx

Key Findings:
- Fine-tuned YOLO11n selected for deployment
- Real-time capable on Jetson Nano
- Optimized for edge AI ASV systems

## 📦 ROS 2 Workspace Structure

alcsv_ws/src/
├── alcsv_sensors
├── alcsv_vision
├── alcsv_coverage_planner
├── alcsv_controller
├── alcsv_safety
├── alcsv_bringup


---

## 🧭 Current Implemented Features

- Circular 50m Geofence
- Zig-Zag Coverage Generation inside boundary
- Path Visualization in RViz
- Waypoint Publisher
- Geofence Violation Detection

---

## 🖥 Installation

```bash
sudo apt update
sudo apt install ros-humble-desktop


Clone repository:

git clone https://github.com/ganeshT099/Autonomous-Lake-Cleaning-ASV.git


Build workspace:

cd alcsv_ws
colcon build
source install/setup.bash

▶️ Running Coverage Planner
ros2 run alcsv_coverage_planner zigzag_coverage

🛡 License

This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

👨‍💻 Author

Ganesh T (TEAM ALCSV)
Mechatronics Engineering With Specialization in Robotics
SRM Institute of Science and Technology
