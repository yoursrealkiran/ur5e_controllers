#include "cartesian_impedance_controller/CartesianImpedanceController.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/SVD>

using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using controller_interface::CallbackReturn;

namespace cartesian_impedance_controller
{

CallbackReturn CartesianImpedanceController::on_init()
{
  auto node = get_node();

  node->declare_parameter<std::vector<std::string>>(
    "joints",
    {},
    rcl_interfaces::msg::ParameterDescriptor()
  );

  node->declare_parameter<std::string>("end_effector", "tool0");
  node->declare_parameter<std::vector<double>>("stiffness", {300,300,300, 20,20,20});
  node->declare_parameter<std::vector<double>>("damping_factors", {1,1,1, 1,1,1});
  node->declare_parameter<bool>("use_nullspace", false);
  node->declare_parameter<double>("nullspace_stiffness", 0.0);
  node->declare_parameter<double>("nullspace_damping", 0.0);
  node->declare_parameter<double>("update_alpha", 0.1); // smoothing

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto & j : joints_) cfg.names.push_back(j + "/effort");
  return cfg;
}

InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (auto & j : joints_)
  {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();

  // Params
  node->get_parameter("joints", joints_);
  node->get_parameter("end_effector", ee_frame_);

  std::vector<double> k, d;
  node->get_parameter("stiffness", k);
  node->get_parameter("damping_factors", d);
  if (k.size() != 6 || d.size() != 6 || joints_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid params: joints or stiffness/damping size.");
    return CallbackReturn::FAILURE;
  }
  for (int i=0;i<6;i++){ K_[i]=k[i]; Dfac_[i]=d[i]; }
  K_cmd_ = K_;
  Dfac_cmd_ = Dfac_;

  node->get_parameter("use_nullspace", use_nullspace_);
  node->get_parameter("nullspace_stiffness", kns_);
  node->get_parameter("nullspace_damping", dns_);
  node->get_parameter("update_alpha", alpha_);
  alpha_ = std::clamp(alpha_, 0.01, 1.0);

  // Load model from robot_description
  if (!load_model_from_robot_description_()) return CallbackReturn::FAILURE;

  ee_frame_id_ = model_.getFrameId(ee_frame_);
  if (ee_frame_id_ < 0 || ee_frame_id_ >= model_.nframes)
  {
    RCLCPP_ERROR(node->get_logger(), "End-effector frame '%s' not found in model.", ee_frame_.c_str());
    return CallbackReturn::FAILURE;
  }

  // Nullspace target init
  qd_ns_.setZero(model_.nq);

  // Subscribers (non-RT)
  sub_ref_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/reference_pose", 10,
      std::bind(&CartesianImpedanceController::cbReferencePose_, this, std::placeholders::_1));

  sub_set_stiffness_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "~/set_cartesian_stiffness", 10,
      std::bind(&CartesianImpedanceController::cbSetStiffness_, this, std::placeholders::_1));

  sub_set_damping_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "~/set_damping_factors", 10,
      std::bind(&CartesianImpedanceController::cbSetDamping_, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "Configured CartesianImpedanceController for %zu joints, EE='%s'.",
              joints_.size(), ee_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  // Map interfaces by name (keep indices we need in update)
  cmd_idx_.clear(); pos_idx_.clear(); vel_idx_.clear();
  cmd_idx_.reserve(joints_.size());
  pos_idx_.reserve(joints_.size());
  vel_idx_.reserve(joints_.size());

  // command_interfaces_ and state_interfaces_ are aligned with requested names order,
  // so we can just index sequentially.
  // Order in command_interface_configuration/state_interface_configuration is joints_ sequence.
  for (size_t i=0; i<joints_.size(); ++i)
  {
    cmd_idx_.push_back(i);           // effort
    pos_idx_.push_back(2*i + 0);     // position
    vel_idx_.push_back(2*i + 1);     // velocity
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Zero efforts on deactivate
  for (size_t i=0; i<joints_.size(); ++i)
  {
    command_interfaces_[cmd_idx_[i]].set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

return_type CartesianImpedanceController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const int N = static_cast<int>(joints_.size());
  if (N == 0) return return_type::OK;

  // Smoothly apply commanded gains (first-order filter)
  K_    = (1.0 - alpha_) * K_    + alpha_ * K_cmd_;
  Dfac_ = (1.0 - alpha_) * Dfac_ + alpha_ * Dfac_cmd_;

  // Read states
  Eigen::VectorXd q(N), dq(N);
  for (int i=0;i<N;i++)
  {
    q[i]  = state_interfaces_[pos_idx_[i]].get_value();
    dq[i] = state_interfaces_[vel_idx_[i]].get_value();
  }

  // FK & Jacobian
  Eigen::Matrix<double,6,1> x_now, twist;
  Eigen::Matrix<double,6,Eigen::RowMajor> J;
  compute_fk_jacobian_(q, dq, x_now, twist, J);

  // If no reference ever set, take current pose as reference to avoid jump
  if (!ref_pose_set_) { x_ref_ = x_now; ref_pose_set_ = true; }

  // Errors
  Eigen::Matrix<double,6,1> e    = x_ref_ - x_now;
  Eigen::Matrix<double,6,1> edot = xd_ref_ - twist;

  // Dabs = factor * 2*sqrt(K) (mass-normalized critical damping scaling)
  Eigen::Matrix<double,6,1> Dabs;
  for (int i=0;i<6;i++) Dabs[i] = Dfac_[i] * 2.0 * std::sqrt(std::max(1e-6, K_[i]));

  // Cartesian wrench
  Eigen::Matrix<double,6,1> W = K_.cwiseProduct(e) + Dabs.cwiseProduct(edot);

  // Map to joint torques
  Eigen::VectorXd tau = J.transpose() * W;

  // Gravity compensation (only gravity; better than full RNEA for impedance baseline)
  pinocchio::computeGeneralizedGravity(model_, data_, q);
  tau += data_.g; // data_.g contains generalized gravity torques

  // Nullspace posture (optional)
  if (use_nullspace_)
  {
    Eigen::VectorXd tau_ns = kns_ * (qd_ns_ - q) - dns_ * dq;

    // Damped pseudoinverse for stability
    const double lambda = 1e-6;
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd inv = (JJt + lambda * Eigen::MatrixXd::Identity(6,6)).inverse();
    Eigen::MatrixXd Jpinv = J.transpose() * inv; // damped pinv
    Eigen::MatrixXd Nproj = Eigen::MatrixXd::Identity(N,N) - J.transpose() * Jpinv.transpose();

    tau += Nproj * tau_ns;
  }

  // Write efforts
  for (int i=0;i<N;i++) command_interfaces_[cmd_idx_[i]].set_value(tau[i]);

  return return_type::OK;
}

bool CartesianImpedanceController::load_model_from_robot_description_()
{
  auto node = get_node();
  std::string urdf_xml;
  if (!node->get_parameter("robot_description", urdf_xml))
  {
    // Try to get parameter from global namespace
    auto params = node->get_parameters({"robot_description"});
    if (params.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "Parameter 'robot_description' not set.");
      return false;
    }
  }

  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Pinocchio buildModelFromXML failed: %s", e.what());
    return false;
  }
  return true;
}

void CartesianImpedanceController::compute_fk_jacobian_(
    const Eigen::VectorXd & q, const Eigen::VectorXd & dq,
    Eigen::Matrix<double,6,1> & x_now, Eigen::Matrix<double,6,1> & twist,
    Eigen::Matrix<double,6,Eigen::RowMajor> & J)
{
  pinocchio::forwardKinematics(model_, data_, q, dq, Eigen::VectorXd::Zero(q.size()));
  pinocchio::updateFramePlacements(model_, data_);
  const auto & oMf = data_.oMf[ee_frame_id_];

  // Pose as [pos, rotvec]
  x_now.head<3>() = oMf.translation();
  x_now.tail<3>() = rotmatToRotvec_(oMf.rotation());

  // Jacobian in WORLD frame
  pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::WORLD, J);

  // Twist = J * dq
  twist = J * dq;
}

Eigen::Vector3d CartesianImpedanceController::rotmatToRotvec_(const Eigen::Matrix3d & R)
{
  Eigen::AngleAxisd aa(R);
  return aa.axis() * aa.angle();
}

void CartesianImpedanceController::cbReferencePose_(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Store desired as [pos, rotvec] in WORLD
  Eigen::Quaterniond qd(msg->pose.orientation.w, msg->pose.orientation.x,
                        msg->pose.orientation.y, msg->pose.orientation.z);
  qd.normalize();
  Eigen::Matrix3d Rd = qd.toRotationMatrix();

  Eigen::Matrix<double,6,1> new_ref;
  new_ref.head<3>() = Eigen::Vector3d(msg->pose.position.x,
                                      msg->pose.position.y,
                                      msg->pose.position.z);
  new_ref.tail<3>() = rotmatToRotvec_(Rd);

  // Smooth set (atomic behavior via swap of entire vector)
  x_ref_ = (1.0 - alpha_) * x_ref_ + alpha_ * new_ref;
  ref_pose_set_ = true;
}

void CartesianImpedanceController::cbSetStiffness_(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  K_cmd_[0] = msg->wrench.force.x;
  K_cmd_[1] = msg->wrench.force.y;
  K_cmd_[2] = msg->wrench.force.z;
  K_cmd_[3] = msg->wrench.torque.x;
  K_cmd_[4] = msg->wrench.torque.y;
  K_cmd_[5] = msg->wrench.torque.z;
}

void CartesianImpedanceController::cbSetDamping_(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  Dfac_cmd_[0] = msg->wrench.force.x;
  Dfac_cmd_[1] = msg->wrench.force.y;
  Dfac_cmd_[2] = msg->wrench.force.z;
  Dfac_cmd_[3] = msg->wrench.torque.x;
  Dfac_cmd_[4] = msg->wrench.torque.y;
  Dfac_cmd_[5] = msg->wrench.torque.z;
}

} // namespace cartesian_impedance_controller
