
from std_msgs.msg import Bool, Empty


class VacuumGripper:
    """
    Simulated Robotiq EPick vacuum gripper.

    Uses Gazebo Harmonic DetachableJoint to attach and
    detach the package.

    ROS topics:
        /vacuum/attach  -> Gazebo attach
        /vacuum/detach  -> Gazebo detach
        /vacuum/active  -> Conveyor controller

    The conveyor itself is never stopped.
    """

    def __init__(
        self,
        node,
        robot_model="ur",
        gripper_link="epick_link",
        package_model="package",
        package_link="package_link",
    ):
        self.node = node

        self.robot_model = robot_model
        self.gripper_link = gripper_link
        self.package_model = package_model
        self.package_link = package_link

        self.attach_publisher = node.create_publisher(
            Empty,
            "/vacuum/attach",
            10,
        )

        self.detach_publisher = node.create_publisher(
            Empty,
            "/vacuum/detach",
            10,
        )

        self.active_publisher = node.create_publisher(
            Bool,
            "/vacuum/active",
            10,
        )

        self.attached = False

        self.node.get_logger().info(
            "Vacuum gripper initialized - Gazebo DetachableJoint"
        )

    def attach(self):
        if self.attached:
            return

        # Stop velocity commands to the package before attaching.
        active_msg = Bool()
        active_msg.data = True
        self.active_publisher.publish(active_msg)

        # Attach package to EPick.
        self.attach_publisher.publish(Empty())

        self.attached = True

        self.node.get_logger().info(
            "Vacuum ON - attach command sent to Gazebo"
        )

    def detach(self):
        if not self.attached:
            return

        # Release package from EPick.
        self.detach_publisher.publish(Empty())

        self.attached = False

        self.node.get_logger().info(
            "Vacuum OFF - detach command sent to Gazebo"
        )
