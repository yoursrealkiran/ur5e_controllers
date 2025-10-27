#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>

namespace cartesian_impedance_controller
{

class CartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  CartesianImpedanceController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Params
  std::vector<std::string> joints_;
  std::string ee_frame_;                      // e.g., "tool0"
  Eigen::Matrix<double,6,1> K_;               // stiffness [Nx,Ny,Nz, Nr, Np, Nyaw]
  Eigen::Matrix<double,6,1> Dfac_;            // damping factors (unitless)
  bool use_nullspace_{false};
  double kns_{0.0}, dns_{0.0};                // nullspace gains
  Eigen::VectorXd qd_ns_;                     // nullspace joint target

  // References (thread-safe via RT-safe copy in update)
  std::atomic<bool> ref_pose_set_{false};
  Eigen::Matrix<double,6,1> x_ref_{Eigen::Matrix<double,6,1>::Zero()};   // [pos(3), rotvec(3)]
  Eigen::Matrix<double,6,1> xd_ref_{Eigen::Matrix<double,6,1>::Zero()};  // desired twist (usually 0)
  Eigen::Matrix<double,6,1> K_cmd_{Eigen::Matrix<double,6,1>::Zero()};
  Eigen::Matrix<double,6,1> Dfac_cmd_{Eigen::Matrix<double,6,1>::Zero()};

  // Filtering for smooth updates
  double alpha_{0.1}; // 0..1 (higher = faster change)

  // Interfaces (in the order of joints_)
  std::vector<size_t> cmd_idx_;
  std::vector<size_t> pos_idx_;
  std::vector<size_t> vel_idx_;

  // Pinocchio
  pinocchio::Model model_;
  pinocchio::Data data_;
  int ee_frame_id_{-1};

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ref_pose_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_set_stiffness_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_set_damping_;

  // Helpers
  bool load_model_from_robot_description_();
  void compute_fk_jacobian_(
      const Eigen::VectorXd & q, const Eigen::VectorXd & dq,
      Eigen::Matrix<double,6,1> & x_now, Eigen::Matrix<double,6,1> & twist,
      Eigen::Matrix<double,6,Eigen::RowMajor> & J6xN);

  // Callbacks
  void cbReferencePose_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cbSetStiffness_(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void cbSetDamping_(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // Utility
  static Eigen::Vector3d rotmatToRotvec_(const Eigen::Matrix3d & R);
};

} // namespace cartesian_impedance_controller
