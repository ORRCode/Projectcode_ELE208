
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import xacro
import yaml


def load_yaml(path):
    with open(path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    project_share = get_package_share_directory(
        "conveyor_project"
    )

    moveit_share = get_package_share_directory(
        "ur_moveit_config"
    )

    urdf_path = os.path.join(
        project_share,
        "urdf",
        "conveyor_ur10e.urdf.xacro"
    )

    srdf_path = os.path.join(
        project_share,
        "config",
        "conveyor_robot.srdf.xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            xacro.process_file(
                urdf_path,
                mappings={
                    "ur_type": "ur10e",
                    "name": "ur"
                }
            ).toxml(),
            value_type=str
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            xacro.process_file(srdf_path).toxml(),
            value_type=str
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            os.path.join(
                project_share,
                "config",
                "kinematics.yaml"
            )
        )
    }

    ompl_config = load_yaml(
        os.path.join(
            moveit_share,
            "config",
            "ompl_planning.yaml"
        )
    )

    joint_limits = load_yaml(
        os.path.join(
            moveit_share,
            "config",
            "joint_limits.yaml"
        )
    )

    controllers = load_yaml(
        os.path.join(
            project_share,
            "config",
            "moveit_controllers.yaml"
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_pipelines": ["ompl"],
                "default_planning_pipeline": "ompl",
                "ompl": ompl_config,
                "robot_description_planning": joint_limits,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "use_sim_time": True,
            },
            controllers,
        ],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/conveyor_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        output="screen",
    )

    camera_detector = Node(
        package="conveyor_project",
        executable="camera_detector",
        name="camera_detector",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    conveyor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="conveyor_bridge",
        arguments=[
            "/package/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        output="screen",
    )

    conveyor_controller = Node(
        package="conveyor_project",
        executable="conveyor_controller",
        name="conveyor_controller",
        output="screen",
        parameters=[
            {"speed": 0.3},
            {"use_sim_time": True},
        ],
    )

    # ROS -> Gazebo attach command.
    vacuum_attach_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="vacuum_attach_bridge",
        arguments=[
            "/vacuum/attach@std_msgs/msg/Empty]gz.msgs.Empty",
        ],
        output="screen",
    )

    # ROS -> Gazebo detach command.
    vacuum_detach_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="vacuum_detach_bridge",
        arguments=[
            "/vacuum/detach@std_msgs/msg/Empty]gz.msgs.Empty",
        ],
        output="screen",
    )

    # Gazebo -> ROS attachment state.
    vacuum_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="vacuum_state_bridge",
        arguments=[
            "/vacuum/state@std_msgs/msg/String[gz.msgs.StringMsg",
        ],
        output="screen",
    )

    return LaunchDescription([
        move_group,
        camera_bridge,
        camera_detector,
        conveyor_bridge,
        conveyor_controller,
        vacuum_attach_bridge,
        vacuum_detach_bridge,
        vacuum_state_bridge,
    ])
