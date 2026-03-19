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
    // Change: Publish to the controller's command topic instead of joint_states
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/ur5e_arm_controller/joint_trajectory", 10);
      
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&UR5eXboxJointPublisher::joy_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(20ms, std::bind(&UR5eXboxJointPublisher::publish_trajectory, this));
    RCLCPP_INFO(this->get_logger(), "UR5e Xbox Gazebo Controller started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double scale = 0.015;
    // Check if RB is pressed for boost
    bool boost = msg->buttons.size() > 5 ? msg->buttons[5] : 0;
    if (boost) scale *= 3.0;

    // Mapping logic (keeping your original mapping)
    deltas_[0] = msg->axes[0] * scale;
    deltas_[1] = msg->axes[1] * scale;
    deltas_[2] = msg->axes[3] * scale;
    deltas_[3] = msg->axes[4] * scale;
    deltas_[4] = (msg->axes[5] - msg->axes[2]) * 0.5 * scale;
    deltas_[5] = (msg->buttons[0] - msg->buttons[1]) * scale;
  }

  void publish_trajectory()
  {
    auto message = trajectory_msgs::msg::JointTrajectory();
    message.header.stamp = this->get_clock()->now();
    message.joint_names.assign(joint_names_.begin(), joint_names_.end());

    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // Update internal positions based on deltas
    for (size_t i = 0; i < positions_.size(); ++i) {
      positions_[i] += deltas_[i];
      point.positions.push_back(positions_[i]);
    }

    // Crucial for Gazebo: Tell the controller how fast to get to this point.
    // Since we loop at 20ms (50Hz), a 25-30ms buffer ensures smooth motion.
    point.time_from_start = rclcpp::Duration(0, 25000000); // 25 nanoseconds

    message.points.push_back(point);
    pub_->publish(message);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<std::string,6> joint_names_;
  std::array<double,6> positions_{0.0, -1.57, 1.57, -1.57, -1.57, 0.0}; // Default UR5e "Upright" pose
  std::array<double,6> deltas_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5eXboxJointPublisher>());
  rclcpp::shutdown();
  return 0;
}