# 🤖 Real-Time Sensor Fusion System

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> Production-quality C++ robotics system demonstrating sensor fusion with Extended Kalman Filter for industrial automation applications.

---

## 📋 Project Overview

Multi-sensor fusion system combining IMU and Lidar data using an Extended Kalman Filter to provide accurate, real-time robot pose estimation. Built entirely in **C++** using ROS 2, demonstrating professional robotics development practices.

**Key Features:**
- ✅ 4 ROS 2 nodes in C++ (880+ lines)
- ✅ Extended Kalman Filter for sensor fusion
- ✅ Real-time performance (50 Hz prediction rate)
- ✅ Safety monitoring and validation
- ✅ Professional code structure

---

## 🏗️ System Architecture
```
┌─────────────┐
│ IMU Sensor  │  C++ Node (100 Hz)
│             │──┐
└─────────────┘  │
                 │
┌─────────────┐  │    ┌──────────────────┐    ┌─────────────────┐
│Lidar Sensor │  ├───→│   Fusion Node    │───→│ Safety Monitor  │
│             │  │    │ (Kalman Filter)  │    │                 │
└─────────────┘  │    │    50 Hz         │    └─────────────────┘
                 │    └──────────────────┘
```

**Data Flow:**
- IMU provides angular velocity and linear acceleration
- Lidar provides position measurements
- Fusion node combines both using Kalman Filter
- Safety monitor validates output

---

## 🚀 Quick Start

### Prerequisites
- ROS 2 Humble
- Docker (recommended for macOS/Windows)
- C++17 compiler
- Eigen3 library

### Building
```bash
# Clone repository
git clone https://github.com/YOUR-USERNAME/sensor-fusion-system.git

# Using Docker (recommended)
docker run -it -v $(pwd):/root/ros2_ws osrf/ros:humble-desktop /bin/bash

# Inside Docker
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sensor_fusion_system
source install/setup.bash
```

### Running

**Terminal 1 - IMU Sensor:**
```bash
ros2 run sensor_fusion_system imu_sensor_node
```

**Terminal 2 - Lidar Sensor:**
```bash
ros2 run sensor_fusion_system lidar_sensor_node
```

**Terminal 3 - Fusion Node:**
```bash
ros2 run sensor_fusion_system fusion_node
```

**Terminal 4 - Safety Monitor:**
```bash
ros2 run sensor_fusion_system safety_monitor_node
```

### Verify System
```bash
# Check active topics
ros2 topic list

# Monitor data rates
ros2 topic hz /imu/data        # ~100 Hz
ros2 topic hz /lidar/scan      # ~10 Hz
ros2 topic hz /fusion/pose     # ~50 Hz

# View live data
ros2 topic echo /fusion/pose
```

---

## 📊 Technical Details

### State Vector (6D)
```
x = [x, y, θ, vx, vy, ω]ᵀ
```
- Position: (x, y) in meters
- Orientation: θ in radians
- Linear velocity: (vx, vy) in m/s
- Angular velocity: ω in rad/s

### Kalman Filter Algorithm

**Prediction Step (50 Hz):**
```cpp
x_predicted = F * x_previous
P_predicted = F * P * F^T + Q
```

**Update Step (Asynchronous):**
```cpp
K = P * H^T * (H * P * H^T + R)^(-1)
x_updated = x_predicted + K * (z - H * x_predicted)
P_updated = (I - K * H) * P_predicted
```

### C++ Implementation Highlights
- **Modern C++17**: Smart pointers, RAII, lambdas
- **Eigen Library**: Efficient matrix operations
- **Multiple Subscribers**: Handles asynchronous sensor data
- **Real-time Performance**: Optimized for embedded systems

---

## 📁 Project Structure
```
sensor_fusion_system/
├── src/
│   ├── sensors/
│   │   ├── imu_sensor_node.cpp      # IMU publisher (100 Hz)
│   │   └── lidar_sensor_node.cpp    # Lidar publisher (10 Hz)
│   ├── fusion/
│   │   └── fusion_node.cpp          # Kalman Filter (50 Hz)
│   └── safety/
│       └── safety_monitor_node.cpp  # Safety validation
├── launch/
│   └── sensor_fusion_system.launch.py
├── config/
│   └── params.yaml
├── package.xml
├── CMakeLists.txt
└── README.md
```

---

## 🎯 Key Features Demonstrated

### C++ Skills
- Object-oriented design with inheritance
- Smart pointers (std::shared_ptr, std::unique_ptr)
- RAII pattern for resource management
- Lambda functions for callbacks
- Template programming (Eigen matrices)
- Const correctness

### Robotics Concepts
- Extended Kalman Filter implementation
- Multi-rate sensor fusion
- Covariance propagation
- Real-time system design
- Safety-critical validation

### Software Engineering
- Professional code structure
- CMake build system
- ROS 2 best practices
- Clear documentation

---

## 🔧 Development

### Building from Source
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_system --symlink-install
```

### Code Formatting
```bash
find src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
```

---

## 📚 Learning Resources

This project demonstrates concepts from:
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Eigen Library**: https://eigen.tuxfamily.org/
- **Kalman Filtering**: Probabilistic Robotics (Thrun, Burgard, Fox)

---

## 🎓 Author

**Your Name**  
M.Sc. Robotics Student @ TH Deggendorf  
Email: asadullahrahoo98@gmail.com  
GitHub: (https://github.com/asadrahu60)  


---

## 📝 License

MIT License - See LICENSE file for details

---

## 🙏 Acknowledgments

- ROS 2 Humble community
- Eigen library developers
- TH Deggendorf Robotics Program

---

## 🎯 Project Status

**Status:** ✅ Fully Functional  
**Build:** ✅ Passing  
**Tested:** ✅ All 4 nodes verified  

Built as portfolio project demonstrating C++ robotics development for embedded systems roles.

---

**⭐ If you find this project useful, please consider giving it a star!**