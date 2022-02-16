# Installation

```bash
# Pre-steps needed to install ROS stuff
echo "deb http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros-focal.list
apt update
apt install gnupg2
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt install curl
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# Install ROS
sudo apt install ros-noetic-desktop-full git-lfs
sudo apt install ros-noetic-object-recognition-msgs ros-noetic-moveit-core ros-noetic-moveit-ros-perception ros-noetic-moveit-ros-planning-interface ros-noetic-velocity-controllers ros-noetic-twist-mux python3-rostopic ros-noetic-effort-controllers ros-noetic-position-controllers

# Clone this repository
cd <REPOSITORY_DIR>
source /opt/ros/noetic/setup.bash
catkin_make -j8

# If you are planning on using the working memory module, take a look at the "Working Memory Module" section below and follow the installation instructions
```

# Execution

## Starting the world

```bash
cd <REPOSITORY_DIR>
source devel/setup.bash
export GAZEBO_MODEL_PATH=<REPOSITORY_DIR>/YCB_Models:$GAZEBO_MODEL_PATH
roslaunch gazebo_ros empty_world.launch extra_gazebo_args:="--verbose" world_name:=$(rosls cobotics_control/worlds/factory.sdf)
```

### Other worlds

Here's a list of other worlds available. To use them, replace `cobotics_control/worlds/factory.sdf` in the above command with the desired world.

- `cobotics_control/worlds/factory_02_populated.sdf`: Factory with two separate areas, one populated with moving humans, one empty
- `cobotics_control/worlds/factory_03_populated.sdf`: Same as above, but with a wall separating both factory areas

## Spawning the Robotnik base with Kuka arm

```bash
cd <REPOSITORY_DIR>
source devel/setup.bash
source GAZEBO_MODEL_PATH=<REPOSITORY_DIR>/YCB_Models:$GAZEBO_MODEL_PATH
roslaunch cobotics_control spawn_rbkairos_kuka_iiwa_robotiq.launch
```

## Spawning the Robotnik base with UR5 arm

```bash
cd <REPOSITORY_DIR>
source devel/setup.bash
source GAZEBO_MODEL_PATH=<REPOSITORY_DIR>/YCB_Models:$GAZEBO_MODEL_PATH
roslaunch cobotics_control spawn_rbkairos_ur5.launch
```

## Controlling the Robotnik base

Moving the base:

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

Teleporting the base to a position:

```bash
rostopic pub -1 /rbkairos/teleport geometry_msgs/Pose "position:
  x: 1.0    # Base x translation
  y: 0.0    # Base y translation
  z: 1.0    # Base z translation
orientation:    # Base Rotation (Note: only rotation around z-axis is taken into account, the rest are ignored)
  x: 0.0    
  y: 0.0    
  z: 0.0    
  w: 0.0"
```


## Controlling the Kuka arm

```bash
rostopic pub -1 /rbkairos/move_arm_controller/command trajectory_msgs/JointTrajectory "joint_names:
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


## Controlling the Robotiq gripper

```bash
rostopic pub /rbkairos/move_gripper_controller/command trajectory_msgs/JointTrajectory "joint_names:
    - palm_finger_1_joint
    - palm_finger_2_joint
    - finger_1_joint_1
    - finger_1_joint_2
    - finger_1_joint_3
    - finger_2_joint_1
    - finger_2_joint_2
    - finger_2_joint_3
    - finger_middle_joint_1
    - finger_middle_joint_2
    - finger_middle_joint_3
points:
    - positions: [ -1000, -1000, -1000,-1000,-1000, -1000,-1000,-1000, -1000,-1000,-10 ]
      velocities: [ 0,0, 0,0,0, 0,0,0, 0,0,0 ]
      accelerations: [ 0,0, 0,0,0, 0,0,0, 0,0,0 ]
      effort: [ 0,0, 0,0,0, 0,0,0, 0,0,0 ]
      time_from_start: {secs: 0, nsecs: 5000000}
"
```

## Additional Gazebo startup parameters

Append the following to the gazebo startup parameters to change behavior:

- `gui:=false` Start without GUI
- `physics:=bullet` Use Bullet instead of ODE as the phyiscs engine

Example:

```bash
roslaunch gazebo_ros empty_world.launch extra_gazebo_args:="--verbose" world_name:=$(rosls cobotics_control/worlds/factory.sdf) gui:=false
```

# Working Memory Module

## Installation

The working memory module can be found under `src/working_memory`. 

Installation requires the creation of a python venv:

```
sudo apt install python3.8-venv

python3.8 -m venv <REPOSITORY_DIR>/src/working_memory/venv
source <REPOSITORY_DIR>/src/working_memory/venv/bin/activate

pip install -r  <REPOSITORY_DIR>/src/working_memory/src/working_memory/requirements.txt
```

## Execution

The working memory module can be started by executing

```
./src/working_memory/scripts/working_memory.sh
```

It subscribes to `wm_human_detection` (`std_msgs/Int8`) and publishes `wm_speed` (`std_msgs/Int8`).

`wm_human_detection` should receive `1` on human detection, `0` when status remains unchanged, and `-1` when no human is detected.

`wm_speed` publishes `-1` when the robot can move fast, and `1` when it should move slow.

