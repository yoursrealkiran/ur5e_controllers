#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <array>
#include <string>
#include <chrono>
using namespace std::chrono_literals;  // <-- needed for 20ms

class UR5eXboxJointPublisher : public rclcpp::Node {
public:
  UR5eXboxJointPublisher()
  : Node("ur5e_xbox_joint_publisher"),
    joint_names_({"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                  "wrist_1_joint","wrist_2_joint","wrist_3_joint"})
  {
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&UR5eXboxJointPublisher::joy_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(20ms, std::bind(&UR5eXboxJointPublisher::publish_joint_state, this));
    RCLCPP_INFO(this->get_logger(), "UR5e Xbox Joint Publisher started");
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double scale = 0.015;
    bool boost = msg->buttons.size() > 5 ? msg->buttons[5] : 0; // RB
    if (boost) scale *= 3.0;

    deltas_[0] = msg->axes[0] * scale;
    deltas_[1] = msg->axes[1] * scale;
    deltas_[2] = msg->axes[3] * scale;
    deltas_[3] = msg->axes[4] * scale;
    deltas_[4] = (msg->axes[5] - msg->axes[2]) * 0.5 * scale;
    deltas_[5] = (msg->buttons[0] - msg->buttons[1]) * scale;
  }

  void publish_joint_state()
  {
    auto now = this->get_clock()->now();
    sensor_msgs::msg::JointState js;
    js.header.stamp = now;

    js.name.assign(joint_names_.begin(), joint_names_.end());  // ✅ FIXED
    for (size_t i = 0; i < positions_.size(); ++i)
      positions_[i] += deltas_[i];
    js.position.assign(positions_.begin(), positions_.end());

    pub_->publish(js);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<std::string,6> joint_names_;
  std::array<double,6> positions_{};
  std::array<double,6> deltas_{};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5eXboxJointPublisher>());
  rclcpp::shutdown();
  return 0;
}

