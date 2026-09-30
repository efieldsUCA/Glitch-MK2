"""Drive Glitch from camera frames with a Chronos behavioral cloning model.

Starts paused. Gamepad controls (button indices are parameters):
    engage_button   toggle autopilot on/off
    stop_button     pause immediately
    enable_button   teleop deadman (LB); while held the pilot stops publishing
                    so the human command wins
"""
from pathlib import Path
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Joy
import torch

from glitchmk2_bringup.chronos_net import ChronosResNetLite
from glitchmk2_bringup.chronos_utils import frame_to_input, labels_to_twist


class ChronosPilot(Node):

    def __init__(self):
        super().__init__(node_name='chronos_pilot')
        self.declare_parameter('model_path', '~/glitchmk2_models/pilot.pth')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('engage_button', 7)  # Start/Menu on Xbox
        self.declare_parameter('stop_button', 1)  # B on Xbox
        self.declare_parameter('enable_button', 4)  # LB, teleop deadman in gamepad.yaml
        self.declare_parameter('image_width', 180)
        self.declare_parameter('image_height', 200)
        self.declare_parameter('max_linear_vel', 0.35)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('throttle_limit', 0.6)  # fraction of max_linear_vel
        self.declare_parameter('image_timeout', 0.5)  # sec without frames before pausing
        self.declare_parameter('num_threads', 4)

        def param(name):
            return self.get_parameter(name).value
        self.engage_button = param('engage_button')
        self.stop_button = param('stop_button')
        self.enable_button = param('enable_button')
        self.width = param('image_width')
        self.height = param('image_height')
        self.max_lin = param('max_linear_vel')
        self.max_ang = param('max_angular_vel')
        self.throttle_limit = param('throttle_limit')
        self.image_timeout = param('image_timeout')

        torch.set_num_threads(param('num_threads'))
        model_path = Path(param('model_path')).expanduser()
        self.model = ChronosResNetLite()
        self.model.load_state_dict(
            torch.load(model_path, weights_only=True, map_location=torch.device('cpu')))
        self.model.eval()

        self.bridge = CvBridge()
        self.cmd_publisher = self.create_publisher(Twist, param('cmd_vel_topic'), 1)
        self.create_subscription(
            Image, param('image_topic'), self.drive, qos_profile_sensor_data)
        self.create_subscription(Joy, param('joy_topic'), self.check_buttons, 10)
        self.create_timer(0.1, self.watchdog)

        self.engaged = False
        self.human_override = False
        self.prev_engage = 0
        self.image_ts = None
        self.infer_ms = 0.0
        self.get_logger().info(
            f'Chronos pilot loaded {model_path}. Paused; press button '
            f'{self.engage_button} to engage.')

    def set_engaged(self, engaged, reason=''):
        if engaged == self.engaged:
            return
        self.engaged = engaged
        if not engaged:
            self.cmd_publisher.publish(Twist())  # one zero so the base stops
        self.get_logger().info(
            f"Autopilot {'ENGAGED' if engaged else 'paused'} {reason}".strip())

    def check_buttons(self, msg):
        def button(i):
            return msg.buttons[i] if 0 <= i < len(msg.buttons) else 0
        if button(self.stop_button):
            self.set_engaged(False, '(stop button)')
        engage = button(self.engage_button)
        if engage and not self.prev_engage:
            self.set_engaged(not self.engaged, '(engage button)')
        self.prev_engage = engage
        self.human_override = bool(button(self.enable_button))

    def watchdog(self):
        if not self.engaged:
            return
        if self.image_ts is None or time.monotonic() - self.image_ts > self.image_timeout:
            self.set_engaged(False, '(no camera frames)')

    def drive(self, msg):
        self.image_ts = time.monotonic()
        if not self.engaged or self.human_override:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        start = time.monotonic()
        with torch.no_grad():
            pred = self.model(torch.from_numpy(frame_to_input(frame, self.width, self.height)))
        steering, throttle = map(float, pred.squeeze())
        self.infer_ms = 0.9 * self.infer_ms + 100.0 * (time.monotonic() - start)
        throttle = max(-self.throttle_limit, min(self.throttle_limit, throttle))
        lin_vel, ang_vel = labels_to_twist(steering, throttle, self.max_lin, self.max_ang)
        cmd = Twist()
        cmd.linear.x = lin_vel
        cmd.angular.z = ang_vel
        self.cmd_publisher.publish(cmd)
        self.get_logger().debug(
            f'steer {steering:.3f} throttle {throttle:.3f} infer {self.infer_ms:.1f} ms')


def main(args=None):
    rclpy.init(args=args)
    pilot = ChronosPilot()
    try:
        rclpy.spin(pilot)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            pilot.cmd_publisher.publish(Twist())
        pilot.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
