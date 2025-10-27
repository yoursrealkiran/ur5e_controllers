#include <pluginlib/class_list_macros.hpp>
#include "cartesian_impedance_controller/CartesianImpedanceController.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_impedance_controller::CartesianImpedanceController,
  controller_interface::ControllerInterface)
