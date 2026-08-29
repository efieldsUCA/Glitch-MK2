import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


class ArucoDetector(Node):
    """ opencv > 4.7 ArUCo marker detection
        NOTE: before starting this node, start the camera_node from camera_ros package
    """

    def __init__(self):
        super().__init__('minimal_subscriber')
        reliable_qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE
        )
        self.camera_node_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.detect_aruco,
            reliable_qos,
        )  # WARNING: Need to start camera_node first
        self.detection_publisher = self.create_publisher(
            Image,
            '/camera/aruco_detection',
            reliable_qos,
        )
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        aruco_params = cv.aruco.DetectorParameters()
        self.aruco_detector = cv.aruco.ArucoDetector(aruco_dict, aruco_params)

    def wait_for_all_acked(self):
        self.get_logger().info('Waiting for all messages to be acknowledged...')
        acknowledged = self.detection_publisher.wait_for_all_acked(Duration(seconds=3))
        if acknowledged:
            self.get_logger().info('All messages acknowledged.')
        else:
            self.get_logger().warn('Not all messages were acknowledged.')

    def detect_aruco(self, msg):
        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().debug(f"Image shape:\n---\n{cv_img.shape}")
        corners, ids, reject_candidates = self.aruco_detector.detectMarkers(cv_img)
        if ids is not None:
            self.get_logger().info(f"detected markers:\n---\n{ids}")
            for i in range(len(ids)):
                top_left_coords = corners[i][0][0].astype(int)
                bot_right_coords = corners[i][0][2].astype(int)
                cv_img = cv.rectangle(cv_img, top_left_coords, bot_right_coords, (131, 44, 88), 5)
                cv.putText(
                    cv_img,
                    str(ids[i][0]),
                    (top_left_coords[0], top_left_coords[1] - 10),  # text pose
                    cv.FONT_HERSHEY_SIMPLEX,
                    1.0,            # Font scale
                    (131, 44, 88),  # Color (matching your box)
                    2,              # Thickness
                    cv.LINE_AA      # Anti-aliased line for cleaner text
                )
        ros_img_msg = CvBridge().cv2_to_imgmsg(cv_img, encoding='bgr8')
        self.detection_publisher.publish(ros_img_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
