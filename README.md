# KLTN-SLAM-2D
# Autonomous Differential Drive Robot SLAM (LD14 LiDAR + ESP32 + MPU6050 + ROS 2)

An end-to-end ROS 2 Humble mapping and localization system for custom differential-drive mobile robots. This repository integrates high-speed C++ driver nodes for the **LD14 LiDAR** and **ESP32 microcontroller (MPU6050 IMU + Quadrature Encoders)**, combined with an **Extended Kalman Filter (EKF)** and **SLAM Toolbox** for robust 2D occupancy grid mapping.

---

## 🌟 Key Features

- **Hardware-Level Sensor Fusion**: Real-time encoder odometry combined with MPU6050 6-DOF IMU data published via serial protocol at 50Hz.
- **Dynamic Covariance & Anti-Slip / Collision System**: Custom fault-detection algorithm that monitors longitudinal acceleration ($A_x$) and angular mismatch between wheel kinematics and gyro data. Upon detecting a collision or wheel slip, it triggers an emergency brake and dynamically adjusts measurement covariance ($100.0$) on `/wheel/odom` so `robot_localization` temporarily ignores erratic wheel data.
- **Soft-Start Engine**: Velocity ramping built into the controller node to eliminate current spikes and prevent ESP32 brownouts during rapid acceleration.
- **High-Efficiency C++ Serial Drivers**: Fast byte-stream parsing for LD14 LiDAR packets (360° LaserScan generation) and dual-channel ESP32 communication.
- **SLAM Toolbox & EKF Integration**: Configured for online asynchronous mapping with custom TF transforms and optimized parameters for low-latency loop closing.

---

## 🏗 System Architecture & Hardware Stack

```text
                     +----------------------------+
                     |    Teleop / Nav2 Stack     |
                     +----------------------------+
                                   | /cmd_vel
                                   v
+------------------+     +------------------------+     +----------------------+
|  LiDAR (LD14)    |     |  base_controller_node  |     |   robot_localization |
|  CP2102 Serial   |     |      (C++ Driver)      |     |      (EKF Node)      |
+------------------+     +------------------------+     +----------------------+
         |                           |                             |
  /scan  |                /wheel/odom| /imu/data                   | /odometry/filtered
         v                           v                             v
+----------------------------------------------------------------------------------+
|                              SLAM Toolbox Node                                   |
|                        (async_slam_toolbox_node)                                 |
+----------------------------------------------------------------------------------+
                                     |
                                     v
                            +-----------------+
                            |  /map (Occupancy|
                            |   Grid Map)     |
                            +-----------------+
