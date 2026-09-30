"""Record camera frames labeled with teleop commands for Chronos training.

Output layout matches what Deep-Learning-Chronos' chronoslearn.py reads:
    <data_root>/<YYYY-MM-DD-HH-MM-SS>/images/000000.jpg
    <data_root>/<YYYY-MM-DD-HH-MM-SS>/labels.csv   (image, steering, throttle)
Press the record button on the gamepad to start/stop a session.
"""
from datetime import datetime
from pathlib import Path

import cv2 as cv
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Joy

from glitchmk2_bringup.chronos_utils import resize_frame, twist_to_labels


class ChronosRecorder(Node):

    def __init__(self):
        super().__init__(node_name='chronos_recorder')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('record_button', 0)  # A on Xbox
        self.declare_parameter('data_root', '~/glitchmk2_data')
        self.declare_parameter('image_width', 180)
        self.declare_parameter('image_height', 200)
        self.declare_parameter('max_linear_vel', 0.35)  # scale_linear_turbo in gamepad.yaml
        self.declare_parameter('max_angular_vel', 0.5)  # scale_angular in gamepad.yaml
        self.declare_parameter('record_rate', 20.0)  # Hz
        self.declare_parameter('cmd_timeout', 0.5)  # sec, older cmd_vel counts as stopped
        self.declare_parameter('skip_idle', True)  # drop frames while the robot is stopped

        def param(name):
            return self.get_parameter(name).value
        self.record_button = param('record_button')
        self.data_root = Path(param('data_root')).expanduser()
        self.width = param('image_width')
        self.height = param('image_height')
        self.max_lin = param('max_linear_vel')
        self.max_ang = param('max_angular_vel')
        self.min_period = 1.0 / param('record_rate')
        self.cmd_timeout = param('cmd_timeout')
        self.skip_idle = param('skip_idle')

        self.bridge = CvBridge()
        self.create_subscription(
            Image, param('image_topic'), self.save_frame, qos_profile_sensor_data)
        self.create_subscription(Twist, param('cmd_vel_topic'), self.store_cmd, 10)
        self.create_subscription(Joy, param('joy_topic'), self.check_button, 10)

        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.cmd_ts = None
        self.last_save_ts = None
        self.prev_button = 0
        self.session_dir = None
        self.labels_file = None
        self.frame_count = 0
        self.get_logger().info(
            f'Chronos recorder ready. Press button {self.record_button} to record.')

    def store_cmd(self, msg):
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z
        self.cmd_ts = self.get_clock().now()

    def check_button(self, msg):
        if self.record_button >= len(msg.buttons):
            return
        pressed = msg.buttons[self.record_button]
        if pressed and not self.prev_button:  # rising edge
            if self.labels_file is None:
                self.start_session()
            else:
                self.stop_session()
        self.prev_button = pressed

    def start_session(self):
        stamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.session_dir = self.data_root / stamp
        (self.session_dir / 'images').mkdir(parents=True, exist_ok=True)
        self.labels_file = open(self.session_dir / 'labels.csv', 'a')
        self.frame_count = 0
        self.get_logger().info(f'Recording to {self.session_dir}')

    def stop_session(self):
        self.labels_file.close()
        self.labels_file = None
        self.get_logger().info(
            f'Stopped recording: {self.frame_count} frames in {self.session_dir}')

    def current_cmd(self, now):
        if self.cmd_ts is None:
            return 0.0, 0.0
        if (now - self.cmd_ts).nanoseconds * 1e-9 > self.cmd_timeout:
            return 0.0, 0.0
        return self.lin_vel, self.ang_vel

    def save_frame(self, msg):
        if self.labels_file is None:
            return
        now = self.get_clock().now()
        if self.last_save_ts is not None:
            if (now - self.last_save_ts).nanoseconds * 1e-9 < self.min_period:
                return
        lin_vel, ang_vel = self.current_cmd(now)
        if self.skip_idle and lin_vel == 0.0 and ang_vel == 0.0:
            return
        steering, throttle = twist_to_labels(lin_vel, ang_vel, self.max_lin, self.max_ang)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_name = f'{self.frame_count:06d}.jpg'
        cv.imwrite(
            str(self.session_dir / 'images' / image_name),
            resize_frame(frame, self.width, self.height),
        )
        self.labels_file.write(f'{image_name},{steering:.4f},{throttle:.4f}\n')
        self.frame_count += 1
        self.last_save_ts = now

    def destroy_node(self):
        if self.labels_file is not None:
            self.stop_session()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = ChronosRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
