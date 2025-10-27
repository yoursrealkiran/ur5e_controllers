cmake_minimum_required(VERSION 3.10)
project(cartesian_impedance_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/CartesianImpedanceController.cpp
  src/plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle pluginlib controller_interface hardware_interface
  urdf Eigen3 pinocchio geometry_msgs std_msgs
)

# Export plugin
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(FILES controller_plugins.xml
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp rclcpp_lifecycle pluginlib controller_interface hardware_interface
  urdf Eigen3 pinocchio geometry_msgs std_msgs
)

ament_package()
