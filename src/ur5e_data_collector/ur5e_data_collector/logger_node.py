import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, Joy
import message_filters
import cv2
import csv
import os
from cv_bridge import CvBridge
from datetime import datetime

# TF2 imports for End-Effector tracking
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class UR5eDataLogger(Node):
    def __init__(self):
        super().__init__('ur5e_data_logger')
        self.bridge = CvBridge()
        
        # State Variables
        self.is_recording = False
        self.last_button_state = 0
        self.episode_path = ""
        self.frame_count = 0

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the exact joints we want to log
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
            'robotiq_85_left_knuckle_joint'
        ]

        # Subscriptions
        self.img_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.joint_sub = message_filters.Subscriber(self, JointState, '/joint_states')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Synchronize Image and Joints (Approximate time)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.img_sub, self.joint_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info("🚀 Data Logger Ready. Press Xbox 'Back' (Button 6) to START/STOP.")

    def joy_cb(self, msg):
        # Edge Detection for Button 6 (Back/View button)
        current_button_state = msg.buttons[6]
        if current_button_state == 1 and self.last_button_state == 0:
            if not self.is_recording:
                self.start_episode()
            else:
                self.stop_episode()
        self.last_button_state = current_button_state

    def start_episode(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.episode_path = os.path.expanduser(f"~/robotic_data/episode_{timestamp}")
        os.makedirs(os.path.join(self.episode_path, "frames"), exist_ok=True)
        
        self.csv_file = open(os.path.join(self.episode_path, "data.csv"), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Header: Step, 7 Joints, 3 Position (XYZ), 4 Orientation (Quaternion)
        header = ['step'] + self.joint_names + ['ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw']
        self.csv_writer.writerow(header)
        
        self.is_recording = True
        self.frame_count = 0
        self.get_logger().info(f"🔴 RECORDING STARTED: {self.episode_path}")

    def stop_episode(self):
        self.is_recording = False
        self.csv_file.close()
        self.get_logger().info(f"⬜ RECORDING STOPPED. Saved {self.frame_count} steps to {self.episode_path}")

    def sync_callback(self, img_msg, joint_msg):
        if not self.is_recording:
            return

        # 1. Get Cartesian End-Effector Pose via TF
        try:
            # base_link to tool0 (the output flange of the UR5e)
            t = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            ee_pose = [
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
            ]
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"TF Lookup failed: {e}")
            return

        # 2. Get Joint Angles (Mapped by Name for accuracy)
        joint_dict = dict(zip(joint_msg.name, joint_msg.position))
        ordered_joints = [joint_dict.get(name, 0.0) for name in self.joint_names]

        gripper_pos = joint_dict.get('robotiq_85_left_knuckle_joint', 0.0)
        gripper_binary = 1 if gripper_pos > 0.1 else 0  # Simple threshold

        # 3. Save Image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img_name = f"frame_{self.frame_count:04d}.jpg"
        cv2.imwrite(os.path.join(self.episode_path, "frames", img_name), cv_image)

        # 4. Write CSV Row
        self.csv_writer.writerow([self.frame_count] + ordered_joints + ee_pose + [gripper_binary])
        
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = UR5eDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_recording:
            node.stop_episode()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()