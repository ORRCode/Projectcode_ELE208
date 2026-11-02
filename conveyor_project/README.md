# UR10e Conveyor Pick & Place – ROS 2 Jazzy

## Project description

This project simulates a UR10e robot with a Robotiq EPick vacuum gripper performing pick-and-place operations on a moving conveyor belt using ROS 2 Jazzy, Gazebo Harmonic and MoveIt 2.

The robot detects a moving package using a simulated camera, tracks the package, attaches it using a Gazebo DetachableJoint, lifts it, retracts towards the base, rotates 180 degrees and places the package on a pallet.

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic
- Universal Robots ROS 2 simulation packages
- MoveIt 2 and UR MoveIt configuration
- ros_gz_bridge and ros_gz_sim
- Python dependencies used by conveyor_project

The project also depends on the Universal Robots simulation and description packages. These must be installed separately.

## Startup instructions

Open four terminals.

### Terminal 1 – Gazebo simulation

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=67
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix conveyor_project)/share:${GZ_SIM_RESOURCE_PATH}"

ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur10e \
  world_file:=$(ros2 pkg prefix conveyor_project)/share/conveyor_project/worlds/conveyor_world.sdf \
  description_file:=$(ros2 pkg prefix conveyor_project)/share/conveyor_project/urdf/conveyor_ur10e.urdf.xacro
```

Press Play in Gazebo.

### Terminal 2 – MoveIt, camera and conveyor

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=67

ros2 launch conveyor_project conveyor_moveit.launch.py
```

### Terminal 3 – Detach the package

The Gazebo DetachableJoint initially attaches the package to the robot. Detach it to allow conveyor movement.

```bash
gz topic -t /vacuum/detach \
  -m gz.msgs.Empty \
  -p "unused: true"
```

Verify that the red package moves along the conveyor.

### Terminal 4 – Run Pick & Place

```bash
cd ~/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=67

ros2 run conveyor_project pick_and_place
```

The robot should execute the complete sequence and finish with:

```text
State: DONE
```

## Main project files

| File | Description |
|---|---|
| `conveyor_world.sdf` | Gazebo world, conveyor, package, camera and pallet |
| `conveyor_ur10e.urdf.xacro` | UR10e robot and EPick attachment configuration |
| `conveyor_moveit.launch.py` | MoveIt, camera and bridge startup |
| `conveyor_controller.py` | Package velocity control |
| `camera_detector.py` | Camera-based package detection |
| `vacuum_gripper.py` | Gazebo attachment and detachment |
| `pick_and_place.py` | Robot pick-and-place state machine |

## Important notes

- ROS_DOMAIN_ID must be 67 in the ROS 2 terminals.
- The package must be detached after starting Gazebo.
- Do not change the calibrated HOME joint configuration without recalibration.
- The current application performs one pick-and-place cycle per run.
- After a complete cycle, restart the simulation to reset the robot and package to their initial positions.
