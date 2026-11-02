
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class ConveyorController(Node):

    def __init__(self):
        super().__init__("conveyor_controller")

        self.declare_parameter("speed", 0.3)

        self.speed = float(
            self.get_parameter("speed").value
        )

        # Conveyor transport enabled by default.
        self.running = True

        # True while the package is attached to the robot.
        self.package_attached = False

        self.publisher = self.create_publisher(
            Twist,
            "/package/cmd_vel",
            10,
        )

        self.control_subscriber = self.create_subscription(
            Bool,
            "/conveyor/run",
            self.control_callback,
            10,
        )

        self.vacuum_subscriber = self.create_subscription(
            Bool,
            "/vacuum/active",
            self.vacuum_callback,
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_velocity,
        )

        self.get_logger().info(
            f"Conveyor started: {self.speed:.2f} m/s towards -X"
        )

    def control_callback(self, msg):
        self.running = bool(msg.data)

        state = (
            "RUNNING"
            if self.running
            else "STOPPED"
        )

        self.get_logger().info(
            f"Conveyor: {state}"
        )

    def vacuum_callback(self, msg):
        self.package_attached = bool(msg.data)

        if self.package_attached:
            self.get_logger().info(
                "Vacuum activated - package transport disabled"
            )
        else:
            self.get_logger().info(
                "Vacuum deactivated - package transport enabled"
            )

    def publish_velocity(self):
        msg = Twist()

        if self.running and not self.package_attached:
            msg.linear.x = -self.speed
        else:
            msg.linear.x = 0.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ConveyorController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
