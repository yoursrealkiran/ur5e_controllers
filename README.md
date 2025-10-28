# 🎮 UR5e Xbox Controller Teleoperation (ROS 2 Humble)

This project allows you to **control a UR5e robot in RViz** using an **Xbox controller** under **ROS 2 Humble**.  
It replaces the `joint_state_publisher_gui` sliders with a C++ node that publishes joint states based on real-time joystick input.

---

## 🧩 Overview

The package `ur5e_xbox_joint_publisher` provides a ROS 2 node that:

- Subscribes to `/joy` messages from the `joy_node`
- Converts Xbox stick and trigger movements into joint increments
- Publishes `sensor_msgs/msg/JointState` messages to `/joint_states`
- Animates the UR5e model in RViz through `robot_state_publisher`

A launch file (`ur5e_xbox_rviz.launch.py`) brings up everything at once:
- UR5e robot model from [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
- `joy_node` joystick driver
- Your C++ Xbox joint publisher node  
The launch also disables the `joint_state_publisher_gui` to avoid topic conflicts.

---

## ⚙️ Requirements

- **ROS 2 Humble**
- **Xbox controller**
- Packages:
  ```bash
  sudo apt install ros-humble-joy 

