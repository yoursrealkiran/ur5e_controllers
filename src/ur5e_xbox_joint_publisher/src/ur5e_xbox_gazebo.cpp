#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <array>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class UR5eXboxJointPublisher : public rclcpp::Node {
public:
  UR5eXboxJointPublisher()
  : Node("ur5e_xbox_joint_publisher"),
    joint_names_({"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                  "wrist_1_joint","wrist_2_joint","wrist_3_joint"})
  {
    // Arm Publisher
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/ur5e_arm_controller/joint_trajectory", 10);
      
    // MODIFICATION: Add publisher for the Robotiq Gripper
    // Even though it's one joint, we use JointTrajectory to match the controller type
    gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/gripper_controller/joint_trajectory", 10);
      
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&UR5eXboxJointPublisher::joy_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(20ms, std::bind(&UR5eXboxJointPublisher::publish_trajectory, this));
    RCLCPP_INFO(this->get_logger(), "UR5e Xbox Gazebo Controller with Gripper Support started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double scale = 0.015;
    
    // RB Button for boost
    bool boost = msg->buttons.size() > 5 ? msg->buttons[5] : 0;
    if (boost) scale *= 3.0;

    // Mapping logic for Arm
    deltas_[0] = msg->axes[0] * scale;
    deltas_[1] = msg->axes[1] * scale;
    deltas_[2] = msg->axes[3] * scale;
    deltas_[3] = msg->axes[4] * scale;
    deltas_[4] = (msg->axes[5] - msg->axes[2]) * 0.5 * scale;
    deltas_[5] = (msg->buttons[3] - msg->buttons[2]) * scale; // Moved rotation to X/Y buttons to free A/B

    // MODIFICATION: Gripper Logic
    // Button 0 = A (Close), Button 1 = B (Open)
    if (msg->buttons[0]) {
      gripper_pos_ = 0.8;  // 0.8 radians is approximately fully closed for 2F-85
    } else if (msg->buttons[1]) {
      gripper_pos_ = 0.0;  // 0.0 is fully open
    }
  }

  void publish_trajectory()
  {
    // 1. ARM MOVEMENT (Always active)
    auto arm_msg = trajectory_msgs::msg::JointTrajectory();
    arm_msg.joint_names.assign(joint_names_.begin(), joint_names_.end()); // The 6 arm joints
    trajectory_msgs::msg::JointTrajectoryPoint arm_point;

    for (size_t i = 0; i < 6; ++i) {
      positions_[i] += deltas_[i]; // Keep adding deltas even if they are 0
      arm_point.positions.push_back(positions_[i]);
    }
    arm_point.time_from_start = rclcpp::Duration(30ms);
    arm_msg.points.push_back(arm_point);
    pub_->publish(arm_msg);

    // 2. GRIPPER MOVEMENT (Parallel execution)
    auto grip_msg = trajectory_msgs::msg::JointTrajectory();
    grip_msg.joint_names = {"robotiq_85_left_knuckle_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint grip_point;
    
    grip_point.positions.push_back(gripper_pos_); // Uses the A/B button state
    grip_point.time_from_start = rclcpp::Duration(50ms);
    grip_msg.points.push_back(grip_point);
    gripper_pub_->publish(grip_msg);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  // MODIFICATION: Gripper Publisher Handle
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::array<std::string,6> joint_names_;
  std::array<double,6> positions_{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  std::array<double,6> deltas_{};

  // MODIFICATION: Internal state for gripper position
  double gripper_pos_ = 0.0; 
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5eXboxJointPublisher>());
  rclcpp::shutdown();
  return 0;
}