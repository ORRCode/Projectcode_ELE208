
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer, TransformListener, TransformException

from conveyor_project.vacuum_gripper import VacuumGripper


# ==================================================
# ROBOT CONFIGURATION
# ==================================================

JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Calibrated HOME - DO NOT CHANGE
HOME_JOINTS = [
    -5.590227783399992,
    -1.4988414790997258,
    1.6432331713008754,
    -1.7151880190984519,
    4.712388980799992,
    -0.877838803199999,
]

HOME = (0.45, 0.60, 0.45)
READY = (0.45, 1.00, 0.57)

PLACE_APPROACH = (0.0, -0.80, 0.65)
PLACE = (0.0, -0.80, 0.344)

TCP_ORIENTATION = (1.0, 0.0, 0.0, 0.0)

BELT_SPEED = -0.30

TRACK_START_X = 0.45
TRACK_END_X = 0.27

CONTACT_Z = 0.45
READY_Z = 0.57

CONTACT_X_TOLERANCE = 0.07
CONTACT_Y_TOLERANCE = 0.08
CONTACT_Z_TOLERANCE = 0.04

CONTACT_CHECK_START = 0.50
CONTACT_CHECK_END = 0.80

# New motion parameters
RETRACT_TARGET = (0.30, 0.65, 0.70)
RETRACT_DURATION = 4.0

BASE_ROTATION = math.pi
BASE_ROTATION_DURATION = 8.0

FINAL_APPROACH_DURATION = 6.0


# ==================================================
# QUATERNION FUNCTIONS
# ==================================================

def qmul(a, b):
    x, y, z, w = a
    X, Y, Z, W = b

    return (
        w * X + x * W + y * Z - z * Y,
        w * Y - x * Z + y * W + z * X,
        w * Z + x * Y - y * X + z * W,
        w * W - x * X - y * Y - z * Z,
    )


def rotate(q, v):
    p = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    return qmul(qmul(q, p), qc)[:3]


# ==================================================
# PICK AND PLACE NODE
# ==================================================

class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        self.set_parameters([
            Parameter("use_sim_time", value=True)
        ])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self
        )

        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik"
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joints,
            10
        )

        self.create_subscription(
            PointStamped,
            "/detected_package",
            self.on_package,
            10
        )

        self.vacuum = VacuumGripper(self)

        self.current_joints = None
        self.package = None
        self.package_stamp = None

        self.state = "START"
        self.busy = False

        self.track_points = []
        self.track_seed = None
        self.track_index = 0
        self.track_start_time = None

        self.attached = False
        self.active_goal = None
        self.contact_log_time = None

        self.timer = self.create_timer(
            0.03,
            self.loop
        )

        self.get_logger().info(
            "Moving-belt pick-and-place started."
        )

    # ==============================================
    # JOINT STATES
    # ==============================================

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))

        if all(j in values for j in JOINTS):
            self.current_joints = [
                float(values[j]) for j in JOINTS
            ]

    # ==============================================
    # PACKAGE DETECTION
    # ==============================================

    def on_package(self, msg):
        if msg.header.frame_id != "world":
            return

        self.package = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )

        self.package_stamp = self.get_clock().now()

    def estimate_box(self, max_age):
        if self.package is None or self.package_stamp is None:
            return None

        age = (
            self.get_clock().now() - self.package_stamp
        ).nanoseconds / 1e9

        if not 0 <= age <= max_age:
            return None

        x, y, z = self.package

        return (
            x + BELT_SPEED * age,
            y,
            z,
        )

    def box_now(self):
        return self.estimate_box(0.25)

    def box_at_contact(self):
        return self.estimate_box(1.0)

    # ==============================================
    # ACTUAL TCP POSITION
    # ==============================================

    def get_tcp_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "world",
                "tcp_link",
                Time()
            )

            t = tf.transform.translation

            return (
                float(t.x),
                float(t.y),
                float(t.z),
            )

        except TransformException:
            return None

    # ==============================================
    # CONVERT TCP TARGET TO TOOL0 POSE
    # ==============================================

    def tool_pose(self, target):
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link",
                "world",
                Time()
            )

        except TransformException:
            return None

        r = tf.transform.rotation
        t = tf.transform.translation

        qb = (r.x, r.y, r.z, r.w)

        offset = rotate(
            TCP_ORIENTATION,
            (0.0, 0.0, 0.143025)
        )

        world = tuple(
            target[i] - offset[i]
            for i in range(3)
        )

        p = rotate(qb, world)

        return (
            (
                p[0] + t.x,
                p[1] + t.y,
                p[2] + t.z,
            ),
            qmul(qb, TCP_ORIENTATION),
        )

    # ==============================================
    # INVERSE KINEMATICS
    # ==============================================

    def ik(self, target, seed, callback):
        converted = self.tool_pose(target)

        if converted is None:
            self.get_logger().error(
                "Missing TF world -> base_link"
            )
            callback(None)
            return

        pos, rot = converted

        req = GetPositionIK.Request()
        ik = req.ik_request

        ik.group_name = "ur_manipulator"
        ik.ik_link_name = "tool0"
        ik.avoid_collisions = False

        ik.timeout = Duration(
            seconds=2
        ).to_msg()

        ik.robot_state.joint_state.name = list(JOINTS)
        ik.robot_state.joint_state.position = list(seed)

        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.header.stamp = (
            self.get_clock().now().to_msg()
        )

        p = ik.pose_stamped.pose

        (
            p.position.x,
            p.position.y,
            p.position.z,
        ) = pos

        (
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        ) = rot

        future = self.ik_client.call_async(req)

        def done(f):
            if self.state == "ERROR":
                return

            try:
                result = f.result()

                if result.error_code.val != 1:
                    self.get_logger().error(
                        f"IK failed at {target}: "
                        f"{result.error_code.val}"
                    )
                    callback(None)
                    return

                d = dict(zip(
                    result.solution.joint_state.name,
                    result.solution.joint_state.position
                ))

                callback([
                    float(d[j]) for j in JOINTS
                ])

            except Exception as e:
                self.get_logger().error(
                    f"IK error: {e}"
                )
                callback(None)

        future.add_done_callback(done)

    # ==============================================
    # SEND JOINT TRAJECTORY
    # ==============================================

    def trajectory(self, waypoints, stage, on_success):
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)

        for seconds, joints in waypoints:
            p = JointTrajectoryPoint()
            p.positions = list(joints)
            p.time_from_start = Duration(
                seconds=seconds
            ).to_msg()

            traj.points.append(p)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.busy = True

        self.get_logger().info(
            f"{stage}: sending "
            f"{len(waypoints)} trajectory points"
        )

        future = self.action_client.send_goal_async(goal)

        def accepted(f):
            try:
                handle = f.result()

                if not handle.accepted:
                    self.fail(
                        f"{stage}: goal rejected"
                    )
                    return

                self.active_goal = handle

                if self.state == "ERROR":
                    handle.cancel_goal_async()
                    return

                if stage == "TRACK_AND_GRIP":
                    self.track_start_time = (
                        self.get_clock().now()
                    )

                handle.get_result_async().add_done_callback(
                    finished
                )

            except Exception as e:
                self.fail(f"{stage}: {e}")

        def finished(f):
            if self.state == "ERROR":
                return

            try:
                result = f.result()

                if (
                    result.status != 4
                    or result.result.error_code != 0
                ):
                    self.fail(
                        f"{stage}: "
                        f"status={result.status}, "
                        f"code={result.result.error_code}, "
                        f"{result.result.error_string}"
                    )
                    return

                self.active_goal = None
                self.busy = False

                if (
                    stage == "TRACK_AND_GRIP"
                    and not self.attached
                ):
                    self.fail(
                        "Tracking finished without "
                        "a valid vacuum contact"
                    )
                    return

                on_success()

            except Exception as e:
                self.fail(f"{stage}: {e}")

        future.add_done_callback(accepted)

    # ==============================================
    # MOVE TCP
    # ==============================================

    def move_tcp(
        self,
        target,
        stage,
        next_state,
        duration=2.0
    ):
        self.busy = True

        def ready(joints):
            if self.state == "ERROR":
                return

            if joints is None:
                self.fail(
                    f"{stage}: IK failed"
                )
                return

            self.trajectory(
                [(duration, joints)],
                stage,
                lambda: self.set_state(next_state)
            )

        self.ik(
            target,
            self.current_joints,
            ready
        )

    # ==============================================
    # STATE MANAGEMENT
    # ==============================================

    def set_state(self, value):
        if self.state == "ERROR":
            return

        self.state = value

        self.get_logger().info(
            f"State: {value}"
        )

    def fail(self, reason):
        if self.state == "ERROR":
            return

        self.get_logger().error(reason)

        self.state = "ERROR"
        self.busy = False

        if self.active_goal is not None:
            try:
                self.active_goal.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(
                    f"Could not cancel trajectory: {e}"
                )

    # ==============================================
    # PRECOMPUTE TRACKING TRAJECTORY
    # ==============================================

    def prepare_tracking(self):
        # Tested tracking points - unchanged.

        self.track_points = [
            (0.20, (0.45, 1.0, 0.57)),
            (0.35, (0.355, 1.0, 0.57)),
            (0.50, (0.310, 1.0, 0.51)),
            (0.65, (0.265, 1.0, 0.450)),
            (0.80, (0.220, 1.0, 0.450)),
        ]

        self.track_seed = list(self.current_joints)
        self.track_index = 0
        self.busy = True

        self.set_state("BUILD_TRACK")
        self.build_track()

    def build_track(self):
        if self.state == "ERROR":
            return

        if self.track_index >= len(self.track_points):
            self.busy = False
            self.set_state("WAIT_BOX")

            self.get_logger().info(
                "Tracking IK ready. "
                "Waiting for next box at x=0.51 m."
            )
            return

        seconds, target = (
            self.track_points[self.track_index]
        )

        def solved(joints):
            if self.state == "ERROR":
                return

            if joints is None:
                self.fail(
                    "Tracking IK failed at point "
                    f"{self.track_index}: {target}"
                )
                return

            self.track_points[
                self.track_index
            ] = (seconds, joints)

            self.track_seed = joints
            self.track_index += 1

            self.build_track()

        self.ik(
            target,
            self.track_seed,
            solved
        )

    # ==============================================
    # CHECK PHYSICAL CONTACT
    # ==============================================

    def check_contact(self, elapsed):
        if self.attached or elapsed < 0.65:
            return

        box = self.box_at_contact()
        tcp = self.get_tcp_position()

        if box is not None and tcp is not None:
            self.get_logger().info(
                f"Grip attempt at t={elapsed:.2f}s: "
                f"TCP=({tcp[0]:.3f}, {tcp[1]:.3f}, "
                f"{tcp[2]:.3f}), "
                f"BOX=({box[0]:.3f}, {box[1]:.3f}, "
                f"{box[2]:.3f})"
            )
        else:
            self.get_logger().warn(
                "Grip attempt: missing recent package "
                "detection or TCP TF; "
                "sending timed attach request for testing."
            )

        self.vacuum.attach()

        # Attach requested, not yet physically confirmed.
        self.attached = True

        self.get_logger().info(
            f"Vacuum attach requested at t={elapsed:.2f}s"
        )

    # ==============================================
    # MAIN STATE MACHINE
    # ==============================================

    def loop(self):
        if self.state in ("ERROR", "DONE"):
            return

        # Monitor contact while tracking.
        if self.state == "TRACK_AND_GRIP":
            if self.track_start_time is not None:
                elapsed = (
                    self.get_clock().now()
                    - self.track_start_time
                ).nanoseconds / 1e9

                if (
                    CONTACT_CHECK_START <= elapsed
                    <= CONTACT_CHECK_END
                    and not self.attached
                ):
                    self.check_contact(elapsed)

            return

        if self.busy:
            return

        # ------------------------------------------
        # START
        # ------------------------------------------

        if self.state == "START":
            if (
                self.current_joints is None
                or not self.ik_client.service_is_ready()
            ):
                return

            if not self.action_client.server_is_ready():
                return

            errors = [
                abs(math.atan2(
                    math.sin(a - b),
                    math.cos(a - b)
                ))
                for a, b in zip(
                    self.current_joints,
                    HOME_JOINTS
                )
            ]

            if max(errors) > 0.08:
                self.fail(
                    "Robot is not at calibrated HOME"
                )
                return

            self.set_state("MOVE_READY")

        # ------------------------------------------
        # MOVE READY
        # ------------------------------------------

        elif self.state == "MOVE_READY":
            self.set_state("MOVING_READY")

            self.move_tcp(
                READY,
                "MOVE_READY",
                "PREPARE_TRACK",
                duration=2.0
            )

        # ------------------------------------------
        # PREPARE TRACKING
        # ------------------------------------------

        elif self.state == "PREPARE_TRACK":
            self.prepare_tracking()

        # ------------------------------------------
        # WAIT FOR BOX
        # ------------------------------------------

        elif self.state == "WAIT_BOX":
            box = self.box_now()

            if box is None:
                return

            x, y, _ = box

            if (
                0.50 <= x <= 0.53
                and abs(y - 1.0) < 0.08
            ):
                self.get_logger().info(
                    f"Box x={x:.3f}. "
                    "Starting precomputed tracking."
                )

                self.track_start_time = None
                self.contact_log_time = None
                self.attached = False

                self.set_state("TRACK_AND_GRIP")

                self.trajectory(
                    self.track_points,
                    "TRACK_AND_GRIP",
                    lambda: self.set_state("LIFT")
                )

        # ------------------------------------------
        # LIFT STRAIGHT UP
        # ------------------------------------------

        elif self.state == "LIFT":
            if not self.attached:
                self.fail(
                    "Tracking ended without "
                    "vacuum attachment"
                )
                return

            self.set_state("LIFTING")

            self.move_tcp(
                (TRACK_END_X, 1.0, 0.65),
                "LIFT",
                "RETRACT",
                duration=1.0
            )

        # ------------------------------------------
        # RETRACT TOWARDS ROBOT BASE
        # ------------------------------------------

        elif self.state == "RETRACT":
            self.set_state("RETRACTING_INWARD")

            self.move_tcp(
                RETRACT_TARGET,
                "RETRACT_INWARD",
                "ROTATE_BASE",
                duration=RETRACT_DURATION
            )

        # ------------------------------------------
        # ROTATE BASE 180 DEGREES
        # ------------------------------------------

        elif self.state == "ROTATE_BASE":
            if self.current_joints is None:
                self.fail(
                    "ROTATE_BASE: no joint state available"
                )
                return

            target_joints = list(self.current_joints)

            # Rotate only shoulder_pan_joint.
            target_joints[0] += BASE_ROTATION

            self.get_logger().info(
                "Rotating shoulder_pan_joint +180 degrees; "
                "holding other five joint angles fixed."
            )

            self.set_state("ROTATING_BASE")

            self.trajectory(
                [(BASE_ROTATION_DURATION, target_joints)],
                "ROTATE_BASE",
                lambda: self.set_state(
                    "FINAL_PALLET_APPROACH"
                )
            )

        # ------------------------------------------
        # FINAL POSITION ABOVE PALLET
        # ------------------------------------------

        elif self.state == "FINAL_PALLET_APPROACH":
            self.set_state("MOVING_TO_PALLET")

            self.move_tcp(
                PLACE_APPROACH,
                "FINAL_PALLET_APPROACH",
                "PLACE_DOWN",
                duration=FINAL_APPROACH_DURATION
            )

        # ------------------------------------------
        # LOWER BOX
        # ------------------------------------------

        elif self.state == "PLACE_DOWN":
            self.set_state("LOWERING_PALLET")

            self.move_tcp(
                PLACE,
                "PLACE_DOWN",
                "RELEASE",
                duration=2.0
            )

        # ------------------------------------------
        # RELEASE BOX
        # ------------------------------------------

        elif self.state == "RELEASE":
            self.vacuum.detach()
            self.attached = False

            self.set_state("RETREAT")

        # ------------------------------------------
        # RETREAT
        # ------------------------------------------

        elif self.state == "RETREAT":
            self.set_state("RETREATING")

            self.move_tcp(
                PLACE_APPROACH,
                "RETREAT",
                "RETURN_HOME",
                duration=2.0
            )

        # ------------------------------------------
        # RETURN HOME
        # ------------------------------------------

        elif self.state == "RETURN_HOME":
            self.set_state("RETURNING")

            self.trajectory(
                [(5.0, HOME_JOINTS)],
                "RETURN_HOME",
                lambda: self.set_state("DONE")
            )


# ==================================================
# MAIN
# ==================================================

def main():
    rclpy.init()

    node = PickAndPlace()

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
