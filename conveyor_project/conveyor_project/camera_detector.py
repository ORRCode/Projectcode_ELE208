
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraDetector(Node):

    def __init__(self):
        super().__init__("camera_detector")

        self.bridge = CvBridge()

        # Camera position in world coordinates
        self.camera_x = 0.8
        self.camera_y = 1.0
        self.camera_z = 1.8

        # Top surface of package
        self.package_z = 0.45

        # Camera horizontal field of view
        self.horizontal_fov = 1.57

        self.subscription = self.create_subscription(
            Image,
            "/conveyor_camera/image",
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointStamped,
            "/detected_package",
            10
        )

        self.get_logger().info(
            "Camera detector started - world XYZ coordinates"
        )

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red package
        mask1 = cv2.inRange(
            hsv,
            np.array([0, 100, 100]),
            np.array([10, 255, 255])
        )

        mask2 = cv2.inRange(
            hsv,
            np.array([170, 100, 100]),
            np.array([180, 255, 255])
        )

        mask = mask1 | mask2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            cv2.imshow("Conveyor Camera Detection", image)
            cv2.waitKey(1)
            return

        contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(contour) < 100:
            return

        M = cv2.moments(contour)

        if M["m00"] == 0:
            return

        u = M["m10"] / M["m00"]
        v = M["m01"] / M["m00"]

        height, width = image.shape[:2]

        # Camera focal length in pixels
        fx = width / (
            2 * np.tan(self.horizontal_fov / 2)
        )

        fy = fx

        # Distance from camera to package top
        distance = self.camera_z - self.package_z

        # Correct camera-to-world transformation
        # Image up    -> World +X
        # Image right -> World -Y

        package_x = self.camera_x + (
            height / 2 - v
        ) * distance / fy

        package_y = self.camera_y + (
            width / 2 - u
        ) * distance / fx

        # Publish detected package position
        point = PointStamped()
        point.header.stamp = msg.header.stamp
        point.header.frame_id = "world"

        point.point.x = float(package_x)
        point.point.y = float(package_y)
        point.point.z = self.package_z

        self.publisher.publish(point)

        # Draw detection
        cv2.drawContours(
            image, [contour], -1, (0, 255, 0), 2
        )

        cv2.circle(
            image,
            (int(u), int(v)),
            6,
            (0, 255, 0),
            -1
        )

        cv2.putText(
            image,
            f"X={package_x:.2f}  Y={package_y:.2f}  Z={self.package_z:.2f}",
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        cv2.imshow("Conveyor Camera Detection", image)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)
    node = CameraDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
