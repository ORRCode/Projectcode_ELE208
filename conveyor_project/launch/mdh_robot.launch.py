
import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    package_name = "conveyor_project"
    package_path = get_package_share_directory(package_name)

    # Les inn den nye MDH-roboten med ros2_control
    xacro_file = os.path.join(
        package_path,
        "urdf",
        "mdh_robot.urdf.xacro"
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    # Publiser robotmodellen og transformasjonane
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    # Start Gazebo med den eksisterande tomme verda
    world_file = os.path.join(
        package_path,
        "worlds",
        "conveyor_world.sdf"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-r " + world_file
        }.items()
    )

    # Set roboten inn i Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "mdh_robot",
            "-z", "0.375"
        ],
        output="screen"
    )

    # Klokke frå Gazebo
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60"
        ],
        output="screen"
    )

    # Seks-aksa Joint Trajectory Controller
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60"
        ],
        output="screen"
    )

    # Start armkontrolleren etter joint_state_broadcaster
    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        clock_bridge,

        # Vent til Gazebo har fått tid til å starte
        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        ),

        # Vent til roboten er sett inn
        TimerAction(
            period=12.0,
            actions=[joint_state_broadcaster]
        ),

        start_arm_controller,
    ])
