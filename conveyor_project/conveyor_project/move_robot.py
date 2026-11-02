
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')

        # Koble til MoveIt sin action-server
        self.client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info('Ventar på MoveIt...')
        self.client.wait_for_server()
        self.get_logger().info('MoveIt er klar!')

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):

        goal = MoveGroup.Goal()

        # Planlegg OG utfør bevegelsen
        goal.planning_options.plan_only = False

        request = goal.request
        request.group_name = 'ur_manipulator'
        request.pipeline_id = 'ompl'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0

        # Køyr med 10 % hastigheit og akselerasjon
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        # Bruk den faktiske robotposisjonen som start
        request.start_state.is_diff = True

        # Ønskt TCP-posisjon
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        # Posisjonskrav for TCP
        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = 'tcp_link'
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        region = BoundingVolume()
        region.primitives.append(sphere)
        region.primitive_poses.append(pose.pose)

        position_constraint.constraint_region = region

        # Orienteringskrav for TCP
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = 'tcp_link'
        orientation_constraint.orientation = pose.pose.orientation

        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        # Kombiner posisjon og orientering
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(constraints)

        self.get_logger().info(
            f'Flyttar TCP til X={x}, Y={y}, Z={z}'
        )

        # Send målet til MoveIt
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('MoveIt avviste målet')
            return

        self.get_logger().info(
            'MoveIt har godteke målet. Ventar på resultat...'
        )

        # Vent til MoveIt er ferdig
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_response = result_future.result()

        if result_response is None:
            self.get_logger().error('Fekk ikkje resultat frå MoveIt')
            return

        result = result_response.result

        if result.error_code.val == 1:
            self.get_logger().info(
                'SUKSESS! MoveIt rapporterer at bevegelsen er utført!'
            )
        else:
            self.get_logger().error(
                f'MoveIt-bevegelsen feila: {result.error_code.val}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = MoveRobot()

    # Test: TCP 5 cm opp frå tidlegare målt posisjon
    node.move_to_pose(
        x=0.653,
        y=0.174,
        z=0.790,
        qx=-0.7071,
        qy=0.7071,
        qz=0.001,
        qw=0.0
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
