
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity


class CollisionChecker(Node):
    def __init__(self):
        super().__init__("collision_checker")

        self.joint_state = None

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.client = self.create_client(
            GetStateValidity,
            "/check_state_validity"
        )

    def joint_callback(self, msg):
        self.joint_state = msg


def main():
    rclpy.init()
    node = CollisionChecker()

    print("Waiting for joint states...")

    while rclpy.ok() and node.joint_state is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not rclpy.ok():
        node.destroy_node()
        return

    print("Joint states received.")

    if not node.client.wait_for_service(timeout_sec=5.0):
        print("ERROR: MoveIt service not available.")
        node.destroy_node()
        rclpy.shutdown()
        return

    request = GetStateValidity.Request()
    request.robot_state.joint_state = node.joint_state
    request.group_name = "ur_manipulator"

    print("Checking collisions...")

    future = node.client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        print("ERROR: Collision check failed.")
    else:
        result = future.result()

        print("\n========== COLLISION RESULT ==========")
        print("Robot state valid:", result.valid)

        if result.contacts:
            for contact in result.contacts:
                print(
                    f"COLLISION: "
                    f"{contact.contact_body_1} <--> "
                    f"{contact.contact_body_2}"
                )
                print(f"  Penetration depth: {contact.depth:.6f} m")
        elif not result.valid:
            print("State invalid, but no collision contacts returned.")
        else:
            print("No collisions detected.")

        print("======================================")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
