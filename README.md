# Installation

```bash
sudo apt install ros-noetic-desktop-full git-lfs
# Clone this repository
cd <REPOSITORY_DIR>
source /opt/ros/noetic/setup.bash
catkin_make -j8
```

# Execution

## Starting the world

```bash
cd <REPOSITORY_DIR>
source devel/setup.bash
source GAZEBO_MODEL_PATH=<REPOSITORY_DIR>/YCB_Models:$GAZEBO_MODEL_PATH
roslaunch gazebo_ros empty_world.launch extra_gazebo_args:="--verbose" world_name:=$(rosls cobotics_control/worlds/factory.sdf)
```

## Spawning the Robotnik base with Kuka arm

```bash
cd <REPOSITORY_DIR>
source devel/setup.bash
source GAZEBO_MODEL_PATH=<REPOSITORY_DIR>/YCB_Models:$GAZEBO_MODEL_PATH
roslaunch cobotics_control spawn_rbkairos_kuka_iiwa_robotiq.launch
```

## Controlling the Robotnik base

```bash
rostopic pub -1 /rbkairos/robotnik_base_control/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0    # Base Forward Velocity
  y: 0.0    # Base Sideways Velocity
  z: 0.0    # Kuka Arm Elevation
angular:
  x: 0.0
  y: 0.0
  z: 0.0    # Base Rotation"
```

## Controlling the Kuka arm

```bash
rostopic pub -1 /rbkairos/iiwa/move_arm_controller/command trajectory_msgs/JointTrajectory "joint_names:
    - 'iiwa_joint_1'
    - 'iiwa_joint_2'
    - 'iiwa_joint_3'
    - 'iiwa_joint_4'
    - 'iiwa_joint_5'
    - 'iiwa_joint_6'
    - 'iiwa_joint_7'
points:
    - positions: [-1.317271741096274,-0.03825659634384859,-0.06871045076758708,1.4776782454071498,1.1784731623551794,-1.3853232334327261,-1.409537799205671]
      velocities: [0,0,0,0,0,0,0]
      accelerations: [0,0,0,0,0,0,0]
      effort: [0,0,0,0,0,0,0]
      time_from_start: {secs: 1, nsecs: 5000000}"
```

