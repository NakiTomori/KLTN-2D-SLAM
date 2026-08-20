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
```

---

## Note:

- In file code, some command is write in Vietnamese.
- Remember to Udev rule setup for device path before using this code (or you can change it in file code)
- I am using Ubuntu 22.04 LTS and ROS 2 Humble Hawksbill to run this project
- Below is ROS 2 dependencies:
  + robot localization
  + slam toolbox
  + tf2 ros
  + teleop twist keyboard (this library help to control robot via keyboard)
  + nav2 map server
- And in my code already have serial file which is Communication Library, you can clone this library by cd to /src and git clone [https://github.com/wjwwood/serial.git](https://github.com/wjwwood/serial.git)
- This project have 2 option:
  + Option 1: Full System SLAM with EKF Sensor Fusion. This mode fuses wheel odometry ($V_x$) and IMU yaw rate ($V_{yaw}$) using robot_localization for enhanced accuracy during sharp turns or terrain irregularities -> run_system_w_slamtoolbox.launch.py
  + Option 2: Encoder-Only SLAM (Without IMU). You may still plug-in your IMU but system will not collect any data of it to using on SLAM Toolbox so do not worry. -> run_case1_no_imu.launch.py
- And the last one is Advanced Features Explained: Dual-Threshold Slip & Impact Protection. The base_controller_node includes self-defense reflexes:
  + High Acceleration Impact ($|A_x| > 4.0\text{ m/s}^2$): Triggers immediate emergency braking to prevent motor drive strain.
  + Quenching / Continuous Slip ($|A_x| > 2.0\text{ m/s}^2$ or Yaw Mismatch $> 0.5\text{ rad/s}$): Temporarily halts motor execution and elevates wheel covariance to $100.0$. This prevents SLAM drift when wheels spin freely against obstacles or slippery surfaces.
