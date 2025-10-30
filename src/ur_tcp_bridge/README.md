# ur_tcp_bridge

ROS 2 Python package for controlling a **UR (Universal Robots)** manipulator via **TCP port 30003**.

This package bridges a custom Python controller class (`URClass`) that communicates directly with the robot,
and exposes its functions to ROS 2 as **topics** and **services** for real-time control and monitoring.

> 🧱 Tested on: **ROS 2 Humble (Python 3.10)**

---

## 🧩 Overview

```
    ┌────────────────────────────┐
    │         UR Robot           │
    │   (TCP Port 30003 Server)  │
    └─────────────▲──────────────┘
                  │  TCP Socket
                  │
    ┌─────────────┴──────────────┐
    │        URClass.py          │
    │  - Low-level TCP control   │
    │  - Parses robot state data │
    │  - Sends motion commands   │
    └─────────────▲──────────────┘
                  │  Python API
                  │
    ┌─────────────┴────────────────────────────┐
    │            ur_bridge_node.py             │
    │          (ROS 2 Node in rclpy)           │
    │                                          │
    │ Publishes:                               │
    │  • /ur/actual_joint (JointState)         │
    │  • /ur/actual_pose  (Pose)               │
    │  • /ur/actual_T     (Float64MultiArray)  │
    │  • /ur/digital_input (BoolMultiArray)    │
    │  • /ur/state_text    (String)            │
    │                                          │
    │ Subscribes:                              │
    │  • /ur/desired_joint (Float64MultiArray) │
    │  • /ur/desired_pose  (Float64MultiArray) │
    │  • /ur/desired_T     (Float64MultiArray) │
    │  • /ur/digital_output (Int32MultiArray)  │
    │                                          │
    │ Service:                                 │
    │  • /get_robot_state (Trigger)            │
    └──────────────────────────────────────────┘

```

---

## ⚙️ Package Structure

```
ur_tcp_bridge/
├── package.xml
├── setup.py
├── README.md
└── ur_tcp_bridge/
  ├── logger.py
  ├── ur_class.py
  └── ur_bridge_node.py
```

---

## 🚀 How to Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur_tcp_bridge
source install/setup.bash
```

---

## ▶️ How to Run

```bash
ros2 run ur_tcp_bridge ur_bridge_node
```

You can also specify robot IP as a parameter:

```bash
ros2 run ur_tcp_bridge ur_bridge_node --ros-args -p ur_ip:=192.168.1.77
```

---

## 🧠 ROS 2 Interfaces

### 🔹 Topics (Published)

| Name                | Type                         | Description                                |
| ------------------- | ---------------------------- | ------------------------------------------ |
| `/ur/actual_joint`  | `sensor_msgs/JointState`     | Real-time robot joint angles [rad]         |
| `/ur/actual_pose`   | `geometry_msgs/Pose`         | TCP position [m] and rotation vector [rad] |
| `/ur/actual_T`      | `std_msgs/Float64MultiArray` | 4×4 transformation matrix (flattened)      |
| `/ur/digital_input` | `std_msgs/Int32MultiArray`   | Controller input bits (8)                  |
| `/ur/state_text`    | `std_msgs/String`            | System mode, safety state, etc.            |

### 🔹 Topics (Subscribed)

| Name                 | Type                         | Description                                  |
| -------------------- | ---------------------------- | -------------------------------------------- |
| `/ur/desired_joint`  | `std_msgs/Float64MultiArray` | Target joint motion command (6D [rad])       |
| `/ur/desired_pose`   | `std_msgs/Float64MultiArray` | Target Cartesian TCP pose [x,y,z,rx,ry,rz]   |
| `/ur/desired_T`      | `std_msgs/Float64MultiArray` | Target 4×4 homogeneous transform (flattened) |
| `/ur/digital_output` | `std_msgs/Int32MultiArray`   | Digital output [port, value]                 |

### 🔹 Services

| Name               | Type               | Description                                               |
| ------------------ | ------------------ | --------------------------------------------------------- |
| `/get_robot_state` | `std_srvs/Trigger` | Returns full robot state (mode, safety, joint, pose, I/O) |

---

## 🦾 Example Commands

### Move robot joints

```bash
ros2 topic pub --once /ur/desired_joint std_msgs/Float64MultiArray "data: [1.57, -1.57, 1.57, 0, 1.57, 0]"
```

### Move TCP pose

```bash
ros2 topic pub --once /ur/desired_pose std_msgs/Float64MultiArray "data: [0.4, 0.0, 0.2, 0.0, 3.14, 0.0]"
```

### Move by 4×4 matrix

```bash
ros2 topic pub --once /ur/desired_T std_msgs/Float64MultiArray "data: [1,0,0,0.4, 0,1,0,0, 0,0,1,0.3, 0,0,0,1]"
```

### Set Digital Output

```bash
ros2 topic pub --once /ur/digital_output std_msgs/Int32MultiArray "data: [0, 1]"
```

### Read current state

```bash
ros2 service call /get_robot_state std_srvs/srv/Trigger
```

---

## 🧩 Internals (URClass)

`URClass` handles low-level socket communication with the UR controller (TCP port **30003**):

- Real-time state update via binary packet parsing (5120 bytes)
- Supported Commands:

  - `connect()`, `disconect()`, `is_connected()`
  - `movej()`, `movel_pose()`, `movel_T()`
  - `wait_move()`, `set_velocity()`, `stopj()`, `stopl()`
  - `controlbox_digital_out()`

- Maintains:

  - `act_q`, `act_X`, `act_T` — Actual joint, pose, transform
  - `des_q`, `des_X`, `des_T` — Target states
  - `digital_input`, `robot_state`, `safety_mode`

---

© 2025 — Maintained by **hanoo**
