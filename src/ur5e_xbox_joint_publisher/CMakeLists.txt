cmake_minimum_required(VERSION 3.8)
project(ur5e_xbox_joint_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ur5e_xbox_joint_publisher src/ur5e_xbox_joint_publisher.cpp)
ament_target_dependencies(ur5e_xbox_joint_publisher rclcpp sensor_msgs)

install(TARGETS ur5e_xbox_joint_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
