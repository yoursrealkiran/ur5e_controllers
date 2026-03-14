# 🎮 UR5e Xbox Controller Teleoperation (ROS 2 Jazzy)

This project allows you to **control a UR5e robot in RViz** using an **Xbox controller** under **ROS 2 Jazzy**.  
It replaces the `joint_state_publisher_gui` sliders with a C++ node that publishes joint states based on real-time joystick input from Xbox controller.

---

## 📺 Demo

![UR5e Xbox Control Demo](videos/demo_rviz_xbox_2.gif)
#
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

- **ROS 2 Jazzy**
- **Xbox controller**

---

## Build and Launch

Install ROS 2 Jazzy from [`ROS 2 Documentation: jazzy`](https://docs.ros.org/en/jazzy/Installation.html)

After installation of ROS 2 Jazzy, In the terminal

`source /opt/ros/jazzy/setup.bash`

`git clone https://github.com/yoursrealkiran/ur5e_controllers.git`

`cd ur5e_controllers`

`colcon build --packages-select ur5e_xbox_joint_publisher`

`source install/setup.bash`

`ros2 launch ur5e_xbox_joint_publisher ur5e_xbox_rviz.launch.py`

Note: Before launching, make sure the Xbox controller is connected to your PC/system.

