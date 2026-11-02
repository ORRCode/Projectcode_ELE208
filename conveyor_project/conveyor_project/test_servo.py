
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.msg import JointJog


class ServoTest(Node):

    def __init__(self):
        super().__init__("servo_test")

        self.set_parameters([
            Parameter("use_sim_time", value=True)
        ])

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.publisher = self.create_publisher(
            JointJog,
            "/servo_node/delta_joint_cmds",
            qos
        )

        self.start_time = time.monotonic()
        self.published_count = 0

        self.timer = self.create_timer(
            0.02, self.publish_command
        )

        self.get_logger().info(
            "Servo test started. Waiting for simulation clock..."
        )

    def publish_command(self):

        elapsed = time.monotonic() - self.start_time
        stamp = self.get_clock().now()

        if elapsed >= 5.0:
            self.get_logger().info(
                f"Test finished. Published "
                f"{self.published_count} commands."
            )
            self.timer.cancel()
            return

        if stamp.nanoseconds == 0:
            return

        msg = JointJog()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "base_link"

        msg.joint_names = ["shoulder_pan_joint"]
        msg.velocities = [0.05]

        self.publisher.publish(msg)
        self.published_count += 1

        if self.published_count % 50 == 0:
            self.get_logger().info(
                f"Published {self.published_count} commands. "
                f"Simulation time: {stamp.nanoseconds / 1e9:.2f}s. "
                f"Subscribers: {self.publisher.get_subscription_count()}"
            )


def main(args=None):

    rclpy.init(args=args)
    node = ServoTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
