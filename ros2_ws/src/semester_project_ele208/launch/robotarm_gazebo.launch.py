import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PKG = 'semester_project_ele208'

# Må vere likt <world name="..."> i my_world.sdf
WORLD_NAME = 'my_world'


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robotarm.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Lar Gazebo finne meshes i pakka (model://semester_project_ele208/...)
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share))

    # Gazebo med world-fila, køyrer med ein gong (-r)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {world}'}.items())

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen')

    # Set roboten inn i verda frå robot_description
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', WORLD_NAME, '-name', 'arm',
                   '-topic', 'robot_description'],
        output='screen')

    # Bridge mellom Gazebo og ROS 2
    # [ = Gazebo -> ROS,  ] = ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/conveyor_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vacuum/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/vacuum/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/conveyor_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/belt/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen')

    # Kontrollarar (ros2_control)
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'])

    jtc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller',
                   '--controller-manager', '/controller_manager'])

    # Start kontrollarane i rett rekkjefølgje
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[jtc]))

    return LaunchDescription([
        resource_path,
        gz_sim,
        rsp,
        spawn,
        bridge,
        after_spawn,
        after_jsb,
    ])
