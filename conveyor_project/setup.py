
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'conveyor_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),

        # EPick and pallet mesh files
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl')),

        # URDF/Xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),

        # MoveIt and camera configuration files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xacro')),

        # ROS 2 launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='student',
    maintainer_email='student@todo.todo',

    description='UR10e conveyor pick-and-place simulation with EPick gripper and camera vision',

    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
    'console_scripts': [
        'move_robot = conveyor_project.move_robot:main',
        'add_collision_objects = conveyor_project.add_collision_objects:main',
        'camera_vision = conveyor_project.camera_vision:main',
        'conveyor_controller = conveyor_project.conveyor_controller:main',
        'package_tracker = conveyor_project.package_tracker:main',
        'pick_and_place = conveyor_project.pick_and_place:main',
        'test_servo = conveyor_project.test_servo:main',
        'check_collisions = conveyor_project.check_collisions:main',
        'test_servo_xyz = conveyor_project.test_servo_xyz:main',
        'camera_detector = conveyor_project.camera_detector:main',
    ],
},
)
