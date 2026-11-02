
import rclpy
from rclpy.node import Node

from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddCollisionObjects(Node):

    def __init__(self):
        super().__init__('add_collision_objects')

        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        self.get_logger().info('Ventar på MoveIt planning scene...')
        self.client.wait_for_service()

    def create_box(self, name, x, y, z, sx, sy, sz):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = name

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [sx, sy, sz]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(shape)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def add_objects(self):

        scene = PlanningScene()
        scene.is_diff = True

        # Transportband: same posisjon som i Gazebo
        conveyor = self.create_box(
            'conveyor',
            0.8, 1.0, 0.2,
            2.0, 0.5, 0.1
        )

        # Europall: same posisjon som i Gazebo
        pallet = self.create_box(
            'euro_pallet',
            0.0, -0.8, 0.072,
            1.2, 0.8, 0.144
        )

        scene.world.collision_objects.extend([
            conveyor,
            pallet
        ])

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response is not None and response.success:
            self.get_logger().info(
                'Transportband og europall lagt inn i MoveIt!'
            )
        else:
            self.get_logger().error(
                'Klarte ikkje å oppdatere planning scene.'
            )


def main(args=None):
    rclpy.init(args=args)

    node = AddCollisionObjects()
    node.add_objects()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
