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
rostopic pub -1 /rbkairos/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0    # Base Forward Velocity
  y: 0.0    # Base Sideways Velocity
  z: 0.0    
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


## Controlling the UR5 arm

```bash
rostopic pub -1 /rbkairos/arm/pos_traj_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'rbkairos_arm_elbow_joint'
- 'rbkairos_arm_shoulder_lift_joint'
- 'rbkairos_arm_shoulder_pan_joint'
- 'rbkairos_arm_wrist_1_joint'
- 'rbkairos_arm_wrist_2_joint'
- 'rbkairos_arm_wrist_3_joint'
points:
- positions: [2.6, -2.5, 0, -1.57, 0, 0]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: [0, 0, 0, 0, 0, 0]
  time_from_start: {secs: 5, nsecs: 0}"
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

## Docker

```bash
cd <REPOSITORY_DIR>/docker_sim
docker build -f Dockerfile -t promen_aid_factory_sim_controllers:latest .
./docker_run.sh -it promen_aid_factory_sim_controllers:latest bash
terminator -u
export GAZEBO_MODEL_PATH=/home/ros/factory_ws/YCB_Models:$GAZEBO_MODEL_PATH
```

Then launch the world and control the robot as indicated above.

