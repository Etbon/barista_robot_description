# barista_robot_description
# ☕ Barista Robot
![barista_robot_description_rviz](https://github.com/user-attachments/assets/8536db6c-ae78-47fc-9c38-1a258026b8b9)

This project implements a modular robot description in ROS 2 using both URDF and Xacro. The robot model simulates a barista robot designed to play a game of pursuit using laser data and odometry. This checkpoint includes:

- Initial Git setup
- Basic URDF creation
- Modular Xacro refactor
- Launch integration for Gazebo and RViz
  
---
## ✅ Part 1 - Base Robot Model (URDF)

### 📂Files
- `urdf/barista_robot_model.urdf`
- `launch/barista_urdf.launch.py`

### ⚖️ Features
- Chassis (cylinder geometry)
- Differential drive wheels
- Front/back caster wheels (yaw, roll, pitch)
- Standoff rods (x4)
- Cup holder tray
- Hokuyo laser scanner (3D mesh)
- Diff drive plugin (libgazebo_ros_diff_drive.so)

### 🚀 How to Launch
```bash
ros2 launch barista_robot_description barista_urdf.launch.py
```

## ✅ Part 2 - Modular Xacro

### 📂 Files
- `xacro/barista_robot_model.urdf.xacro`
- `xacro/wheel.xacro`
- `xacro/standoff.xacro`
- `xacro/cup_holder_tray.xacro`
- `xacro/laser_scanner.xacro`
- `launch/barista_xacro.launch.py`

### ⚖️ Features
- `xacro:macro` for each part (wheels, rods, tray, laser)
- Parametrized properties (length, radius, mass)
- `include_laser` argument for toggling laser visibility (default true)

### 🚀 How to Launch
```bash
ros2 launch barista_robot_description barista_xacro.launch.py
```

### ✅ Verification
- Gazebo loads modular robot model
- RViz shows sensor TF tree
- `/scan` and `/odom` data published

---
## 🧪 Topics to Test
```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odom
```
---
## ✍️ Author
- Name: **Martin Etchevery Boneo**
---
