
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


class ServoXYZTest(Node):
    def __init__(self):
        super().__init__("servo_xyz_test")

        # use_sim_time is already declared by ROS 2
        self.set_parameters([
            Parameter("use_sim_time", value=True)
        ])

        self.publisher = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10
        )

        self.client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type"
        )

    def activate_cartesian_mode(self):
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Servo service is not available")

        request = ServoCommandType.Request()
        request.command_type = 1  # TWIST mode

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=10.0
        )

        if not future.done() or future.result() is None:
            raise RuntimeError("Failed to switch Servo command type")

        if not future.result().success:
            raise RuntimeError("Servo rejected Cartesian mode")

    def move(self, vx, vy, vz, duration):
        period = 0.02
        start = time.monotonic()

        self.get_logger().info(
            f"Moving TCP: vx={vx}, vy={vy}, vz={vz}"
        )

        while rclpy.ok() and time.monotonic() - start < duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            msg.twist.linear.x = float(vx)
            msg.twist.linear.y = float(vy)
            msg.twist.linear.z = float(vz)

            self.publisher.publish(msg)
            time.sleep(period)

        # Stop TCP movement
        for _ in range(10):
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"

            self.publisher.publish(msg)
            time.sleep(period)


def main():
    rclpy.init()
    node = ServoXYZTest()

    try:
        node.activate_cartesian_mode()

        node.get_logger().info("Cartesian Servo mode activated")

        node.move(0.02, 0.00, 0.00, 2.0)
        node.move(0.00, 0.02, 0.00, 2.0)
        node.move(0.00, 0.00, 0.02, 2.0)

        node.get_logger().info("XYZ test finished")

    except Exception as error:
        node.get_logger().error(str(error))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
