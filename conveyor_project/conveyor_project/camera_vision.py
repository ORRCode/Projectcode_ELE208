
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TwistStamped
from cv_bridge import CvBridge


class CameraVision(Node):

    def __init__(self):
        super().__init__('camera_vision')

        self.bridge = CvBridge()

        # Camera position in world coordinates
        self.camera_x = 0.8
        self.camera_y = 1.0
        self.camera_z = 1.8

        # Top surface of package
        self.package_top_z = 0.45

        # Fallback intrinsics matching 640x480, HFOV 1.57
        # Replace with actual CameraInfo when available.
        self.fx = 320.25
        self.fy = 320.25
        self.cx = 320.0
        self.cy = 240.0

        self.have_camera_info = False

        # Assumes downward optical axis and image axes
        # aligned with world X/Y. Verify before robot control.
        self.image_x_to_world_x = 1.0
        self.image_y_to_world_y = 1.0

        self.last_position = None
        self.last_time = None

        self.image_sub = self.create_subscription(
            Image,
            '/conveyor_camera/image',
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/conveyor_camera/camera_info',
            self.info_callback,
            10
        )

        self.position_pub = self.create_publisher(
            PointStamped,
            '/package/vision_position',
            10
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/package/vision_velocity',
            10
        )

        self.get_logger().info(
            'Camera vision started. Waiting for images...'
        )

    def info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.have_camera_info = True

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion: {exc}')
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Red wraps around hue=0 in OpenCV
        lower_red_1 = np.array([0, 100, 80])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 100, 80])
        upper_red_2 = np.array([179, 255, 255])

        mask = (
            cv2.inRange(hsv, lower_red_1, upper_red_1)
            | cv2.inRange(hsv, lower_red_2, upper_red_2)
        )

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel
        )

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            self.last_position = None
            self.last_time = None
            return

        contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(contour) < 100:
            return

        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return

        u = moments['m10'] / moments['m00']
        v = moments['m01'] / moments['m00']

        # Distance from camera to top of package
        depth = self.camera_z - self.package_top_z

        # Pinhole projection (initial calibration)
        camera_dx = (u - self.cx) * depth / self.fx
        camera_dy = (v - self.cy) * depth / self.fy

        world_x = (
            self.camera_x
            + self.image_x_to_world_x * camera_dx
        )
        world_y = (
            self.camera_y
            + self.image_y_to_world_y * camera_dy
        )

        # Publish detected position
        position_msg = PointStamped()
        position_msg.header = msg.header
        position_msg.header.frame_id = 'world'

        position_msg.point.x = float(world_x)
        position_msg.point.y = float(world_y)
        position_msg.point.z = self.package_top_z

        self.position_pub.publish(position_msg)

        # Estimate velocity from consecutive detections
        timestamp = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        if (
            self.last_position is not None
            and self.last_time is not None
        ):
            dt = timestamp - self.last_time

            if 0.005 < dt < 0.5:
                vx = (
                    world_x - self.last_position[0]
                ) / dt

                vy = (
                    world_y - self.last_position[1]
                ) / dt

                velocity_msg = TwistStamped()
                velocity_msg.header = position_msg.header

                velocity_msg.twist.linear.x = float(vx)
                velocity_msg.twist.linear.y = float(vy)

                self.velocity_pub.publish(velocity_msg)

        self.last_position = (world_x, world_y)
        self.last_time = timestamp


def main(args=None):
    rclpy.init(args=args)
    node = CameraVision()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
