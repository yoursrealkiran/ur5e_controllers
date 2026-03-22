# 🎮 UR5e Xbox Controller Teleoperation (ROS 2 Jazzy)

This project allows you to **control a UR5e robot (with Robotiq gripper)** in both **RViz** (visualization) and **Gazebo** (physics simulation) using an **Xbox controller** under **ROS 2 Jazzy**.  

---

## 📺 RViz Demo

![UR5e Xbox Control RViz Demo](videos/demo_rviz_xbox_2.gif)
#
---

## 📺 Gazebo Demo

![UR5e Xbox Control Gazebo Demo](videos/demo_gazebo_xbox.gif)
#
---

## 🧩 Overview

The ur5e_xbox_joint_publisher package now supports two modes of operation:

**1. RViz Mode (Kinematic Visualization)**

- Node: ur5e_xbox_joint_publisher
- Function: Directly publishes sensor_msgs/msg/JointState to /joint_states.
- Use Case: Light-weight testing of joint mappings and URDF visualization.

**2. Gazebo Mode (Dynamic Simulation)**

- Node: ur5e_xbox_gazebo
- Function: Publishes trajectory_msgs/msg/JointTrajectory to the ur5e_arm_controller and the gripper_controller.
- Features: Uses a custom Xacro wrapper (gazebo_ur5e.xacro) to inject ros2_control hardware interfaces.
    1. Uses Gazebo Sim with the gz_ros2_control plugin.
    2. Full physics interaction and gravity compensation.
    3. End-Effector: Integrated Robotiq 2F-85 Gripper with parallel linkage mimicry.

**The Robot model**

UR5e robot model from [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) is used here.

---

## ⚙️ Requirements

- **ROS 2 Jazzy**
- **Gazebo Sim (Included with ROS 2 desktop-full)**
- **Xbox controller**

---

## Build and Launch

Install ROS 2 Jazzy from [`ROS 2 Documentation: jazzy`](https://docs.ros.org/en/jazzy/Installation.html)

After installation of ROS 2 Jazzy, In the terminal

`source /opt/ros/jazzy/setup.bash`

The Gripper: Install Robotiq description package using below command in the terminal,

`sudo apt install ros-jazzy-robotiq-description`

Clone this repository

`git clone https://github.com/yoursrealkiran/ur5e_controllers.git`

Navigate to src/ folder 

`cd ur5e_controllers/src`

Clone Universal_Robots_ROS2_Description repository

`git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git`

Use below command to navigate back to ur5e_controllers folder (i.e, come out of src/ folder) 

`cd ..`

`colcon build --packages-select ur5e_xbox_joint_publisher Universal_Robots_ROS2_Description`

`source install/setup.bash`


**Launching in RViz (Visualization Only)**

`ros2 launch ur5e_xbox_joint_publisher ur5e_xbox_rviz.launch.py`

**Launching in Gazebo (Physics Simulation)**

`ros2 launch ur5e_xbox_joint_publisher ur5e_xbox_gazebo.launch.py`

Note: Before launching, make sure the Xbox controller is connected to your PC/system.

