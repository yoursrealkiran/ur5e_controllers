#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR5eXboxTeleop(Node):
    def __init__(self):
        super().__init__('ur5e_xbox_teleop')
        self.pub = self.create_publisher(JointTrajectory, '/ur5e_joint_velocity_controller/joint_trajectory', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.scale = 0.6
        self.get_logger().info("UR5e Xbox teleop node started")

    def joy_callback(self, msg):
        jt = JointTrajectory()
        jt.joint_names = self.joint_names

        v = [0.0] * 6
        v[0] = msg.axes[0] * self.scale      # left stick X
        v[1] = msg.axes[1] * self.scale      # left stick Y
        v[2] = msg.axes[3] * self.scale      # right stick X
        v[3] = msg.axes[4] * self.scale      # right stick Y
        v[4] = (msg.axes[5] - msg.axes[2]) * 0.5 * self.scale  # triggers
        v[5] = (msg.buttons[0] - msg.buttons[1]) * self.scale  # A/B buttons

        point = JointTrajectoryPoint()
        point.velocities = v
        point.time_from_start.sec = 1
        jt.points.append(point)
        self.pub.publish(jt)

def main(args=None):
    rclpy.init(args=args)
    node = UR5eXboxTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
