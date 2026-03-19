cmake_minimum_required(VERSION 3.8)
project(ur5e_xbox_joint_publisher)

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED) # Added for Gazebo support

# === Executable: RViz Version ===
add_executable(ur5e_xbox_joint_publisher src/ur5e_xbox_joint_publisher.cpp)
ament_target_dependencies(ur5e_xbox_joint_publisher rclcpp sensor_msgs)

# === Executable: Gazebo Version ===
add_executable(ur5e_xbox_gazebo src/ur5e_xbox_gazebo.cpp)
ament_target_dependencies(ur5e_xbox_gazebo rclcpp sensor_msgs trajectory_msgs)

# === Install targets ===
install(TARGETS 
  ur5e_xbox_joint_publisher
  ur5e_xbox_gazebo
  DESTINATION lib/${PROJECT_NAME})

# === Install launch files and config ===
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()