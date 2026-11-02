
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, TwistStamped


class PackageTracker(Node):

    def __init__(self):
        super().__init__('package_tracker')

        self.declare_parameter('prediction_time', 0.5)
        self.prediction_time = float(
            self.get_parameter('prediction_time').value
        )

        self.position = None
        self.velocity = None

        self.position_sub = self.create_subscription(
            PointStamped,
            '/package/vision_position',
            self.position_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/package/vision_velocity',
            self.velocity_callback,
            10
        )

        self.prediction_pub = self.create_publisher(
            PointStamped,
            '/package/predicted_position',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_prediction
        )

        self.get_logger().info('Package tracker started')

    def position_callback(self, msg):
        self.position = msg

    def velocity_callback(self, msg):
        self.velocity = msg

    def publish_prediction(self):
        if self.position is None or self.velocity is None:
            return

        dt = self.prediction_time

        predicted = PointStamped()
        predicted.header = self.position.header

        predicted.point.x = (
            self.position.point.x
            + self.velocity.twist.linear.x * dt
        )

        predicted.point.y = (
            self.position.point.y
            + self.velocity.twist.linear.y * dt
        )

        predicted.point.z = self.position.point.z

        self.prediction_pub.publish(predicted)


def main(args=None):
    rclpy.init(args=args)
    node = PackageTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
