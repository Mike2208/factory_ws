#!/usr/bin/env python

import sys
import copy
import rospy
import json
import yaml
# sys.path.append("/home/erdi/Desktop/Storage/Temporary/GIT/Github/moveit/moveit_commander/src")
import moveit_commander
import math
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Image
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PointStamped
import genpy
from moveit_msgs.msg._RobotTrajectory import RobotTrajectory
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from threading import Lock
import std_msgs
import std_srvs
from gazebo_msgs.srv import GetModelState
from cle_ros_msgs.srv import SetDuration, SetDurationRequest, SetDurationResponse
from time import sleep
import numpy as np
from specs import localize_target
from cv_bridge import CvBridge
import torch

velLock = Lock()
targetPoseLock = Lock()
moveLock = Lock()

targetPoint = PointStamped()
targetPose = Pose()
targetVel = Point()
robotPose = Pose(position=Point(x=1.0, y=-0.75, z=0.8))

grasp_speed = 10
return_speed = 1.0

speed = 6.5
trajectoryIndex = 13
execTime = genpy.Duration()

camImageNum = 0

gzLock = Lock()
gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def getObjectPointFromImage(camera):
    """Get Object Pose from camera image"""
    max_pix_value = 1.0
    normalizer = 255.0 / max_pix_value
    cam_img = CvBridge().imgmsg_to_cv2(camera, 'rgb8') / normalizer
    cam_img = torch.tensor(cam_img).permute(2, 1, 0)

    targ = localize_target(cam_img)

    point = PointStamped(header=camera.header, point=Point(x=(1.581 - targ[0]/154.29), y=(-0.16 - targ[1]/154.29), z=0.964))
    return point


def updateObjectVelAndPose(camera):
    """Compute object velocity from two different objects"""
    global targetPoint, targetVel

    newTargetPoint = getObjectPointFromImage(camera)
    with gzLock:
        gz_pose = gazebo_model_state.call(model_name="ball", relative_entity_name='')
    newTargetPoint.header = gz_pose.header
    newTargetPoint.point = gz_pose.pose.position
    if not (math.isnan(newTargetPoint.point.x) or math.isnan(newTargetPoint.point.y) or math.isnan(newTargetPoint.point.z)):
        diffTime = rospy.Time(secs=newTargetPoint.header.stamp.secs, nsecs=newTargetPoint.header.stamp.nsecs) - rospy.Time(secs=targetPoint.header.stamp.secs, nsecs=targetPoint.header.stamp.nsecs)
        if diffTime.to_sec() > 0.001:
            with velLock:
                targetVel.x = (newTargetPoint.point.x - targetPoint.point.x) / (diffTime.to_sec())
                targetVel.y = (newTargetPoint.point.y - targetPoint.point.y) / (diffTime.to_sec())
                targetVel.y = (newTargetPoint.point.z - targetPoint.point.z) / (diffTime.to_sec())

            targetPoint = copy.deepcopy(newTargetPoint)


def computeTargetPose(graspTime):
    """Compute object position in the future, after a duration of graspTime"""
    global targetPoint, targetVel, targetPose
    futureTime = graspTime + rospy.Time.now() - rospy.Time(secs=targetPoint.header.stamp.secs, nsecs=targetPoint.header.stamp.nsecs)
    with velLock:
        targetPose.position.x = targetPoint.point.x + targetVel.x*futureTime.to_sec() - robotPose.position.x
        targetPose.position.y = targetPoint.point.y + targetVel.y*futureTime.to_sec() - robotPose.position.y
        targetPose.position.z = downPoses[trajectoryIndex].position.z
        targetPose.orientation = copy.deepcopy(downPoses[trajectoryIndex].orientation)

    return targetPose

def updateTargetPose(camera):
    """Callback function for camera image subscriber. Will update velocity and pose every 5 frames"""
    global camImageNum
    if camImageNum < 5:
        camImageNum += 1
    else:
        camImageNum = 0
        updateObjectVelAndPose(camera)


def createPoseFromJSONObj(posejsonobj):
    """Create Pose object from JSON"""
    return Pose(position=Point(x=posejsonobj["position"]["x"], y=posejsonobj["position"]["y"], z=posejsonobj["position"]["z"]),
                orientation=Quaternion(x=posejsonobj["orientation"]["x"], y=posejsonobj["orientation"]["y"], z=posejsonobj["orientation"]["z"], w=posejsonobj["orientation"]["w"]))


def getDistSquared(pose1, pose2):
    return (pose1.position.x-pose2.position.x)**2 + (pose1.position.y-pose2.position.y)**2


def execTrajectoryPath():
    """
    Execute trajectory with the given index. Assumes the arm is at uppose, then moves it to a position above the
    conveyor. There, the gripper is closed, and the arm subsequently moved back up to uppose. Lastly, the gripper is
    opened again, releasing any object
    """
    global execTime, targetVel
    try:
        with gzLock:
            curTargetPose = gazebo_model_state.call(model_name="ball", relative_entity_name='')
    except rospy.ServiceException:
        curTargetPose = None

    # Local TF to arm position
    curTargetVel = curTargetPose.twist.linear
    curTargetPos = curTargetPose.pose
    curTargetPos.position.x -= 1.0
    curTargetPos.position.y -= -0.75
    curTargetPos.position.z -= 0.8

    targetTrajIndex = 0
    targetTrajDist = abs(curTargetPos.position.y - downPoses[targetTrajIndex].position.y)
    for i in range(1, len(downPoses)):
        curDist = abs(curTargetPos.position.y - downPoses[i].position.y)
        if curDist < targetTrajDist:
            targetTrajIndex = i
            targetTrajDist = curDist

    if targetTrajDist > 0.05:
        rospy.logwarn("Minimum Distance between object and gripper is too large: " + str(targetTrajDist) +
                      "\nRecompute trajectories")

    targetTraj = adjustSpeed(downTrajectories[targetTrajIndex], downTrajectories[targetTrajIndex].joint_trajectory.points[-1].time_from_start / execTime)
    iiwa_group.execute(targetTraj)

    gripperTraj = adjustSpeed(grasp_group.plan(close_gripper_target), grasp_speed)
    startTime = rospy.Time.now()
    try:
        with gzLock:
            curPoseMsg = gazebo_model_state.call(model_name="ball", relative_entity_name='')
    except rospy.ServiceException:
        curPoseMsg = None

    # Local TF to arm position
    if curPoseMsg is not None:
        curPoseMsg.pose.position.x -= 1.0
        curPoseMsg.pose.position.y -= -0.75
        curPoseMsg.pose.position.z -= 0.8
    while curPoseMsg is not None and curPoseMsg.pose.position.x + curTargetVel.x*rospy.Duration(0.1).to_sec() < downPoses[trajectoryIndex].position.x:
        # Wait at most 5 seconds for object, then move arm back to start position without trying to grasp
        try:
            with gzLock:
                curPoseMsg = gazebo_model_state.call(model_name="ball", relative_entity_name='')
        except rospy.ServiceException:
            curPoseMsg = None

        if curPoseMsg is not None:
            # Local TF to arm position
            curPoseMsg.pose.position.x -= 1.0
            curPoseMsg.pose.position.y -= -0.75
            curPoseMsg.pose.position.z -= 0.8

        if rospy.Time.now() - startTime > rospy.Duration(secs=1):
            iiwa_group.execute(adjustSpeed(iiwa_group.plan(upJointState), return_speed))
            grasp_group.go(open_gripper_target)
            return

    grasp_group.execute(gripperTraj)
    iiwa_group.execute(adjustSpeed(iiwa_group.plan(upJointState), return_speed))
    grasp_group.go(open_gripper_target)
    return


def adjustSpeed(old_traj, speed):
    """Recalculate a trajectory with new speed"""
    new_traj = RobotTrajectory()
    new_traj.joint_trajectory = copy.deepcopy(old_traj.joint_trajectory)
    n_joints = len(old_traj.joint_trajectory.joint_names)
    n_points = len(old_traj.joint_trajectory.points)

    for i in range(n_points):
        new_traj.joint_trajectory.points[i].time_from_start = old_traj.joint_trajectory.points[i].time_from_start / speed
        new_traj.joint_trajectory.points[i].velocities = [old_traj.joint_trajectory.points[i].velocities[j] * speed for j in range(n_joints)]
        new_traj.joint_trajectory.points[i].accelerations = [old_traj.joint_trajectory.points[i].accelerations[j] * speed for j in range(n_joints)]
        new_traj.joint_trajectory.points[i].positions = [old_traj.joint_trajectory.points[i].positions[j] for j in range(n_joints)]

    return new_traj


def callback(data):
    """
    Execute motion to center of conveyor belt if trigger is received.
    While executing a motion, ignore any additional triggers
    """
    if data.data != 0:
        if moveLock.acquire(False):
            global speed
            try:
                execTrajectoryPath()
                sleep(0.01)
            except:
                moveLock.release()
                raise

            moveLock.release()


def setTargetPose(data):
    """Set the target position. Keep orientation the same (gripper facing down toward conveyor)"""
    global targetPose
    with targetPoseLock:
        if not (math.isnan(data.data[-3]) or math.isnan(data.data[-2])):
            # Extract last frame's position, relative to the robot
            targetPose.position.x = data.data[-3] - robotPose.position.x
            targetPose.position.y = data.data[-2] - robotPose.position.y
            # NOTE: Keep the end effector z position the same. Just above the conveyor belt
            # targetPose.position.z = data.data[-1] + 0.1
        #else:
        #    rospy.logwarn("Grasp motion planner received array with undefined target position values")


def service_callback(data):
    """Set trajectory execution time"""
    global execTime, speed
    execTime = copy.deepcopy(data.duration.data)
    speed = downTrajectories[trajectoryIndex].joint_trajectory.points[-1].time_from_start / execTime

    return SetDurationResponse(success=True)


def add_conveyor_collision(scene, robot):
    """Create and add a collision box to the planning scene that prevents collision with the conveyor belt"""
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.0
    p.pose.position.y = -0.55
    p.pose.position.z = -0.19

    scene.add_box("conveyor", p, (2.5, 0.8, 0.6))

    sleep(2)


if __name__ == '__main__':
    jsonFileName = sys.argv[1]

    # Start Moveit and initialize this node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grasp_motion', anonymous=True)

    # Identify robot
    robot = moveit_commander.RobotCommander(ns="/iiwa", robot_description="/iiwa/robot_description")

    # Initiate planning scene
    scene = moveit_commander.PlanningSceneInterface(ns="/iiwa", synchronous=True)
    add_conveyor_collision(scene, robot)

    maxAttempst = 5

    # Get IIWA arm planning group
    attempts = 0
    iiwa_group_name = "iiwa_plan_group"
    while True:
        try:
            iiwa_group = moveit_commander.MoveGroupCommander(iiwa_group_name, ns="/iiwa",
                                                             robot_description="/iiwa/robot_description")
            break
        except:
            if attempts < maxAttempst and not rospy.is_shutdown():
                rospy.logwarn("Failed to load iiwa planner. Retrying...")
                attempts += 1
            else:
                raise


    # Get Gripper planning group
    attempts = 0
    grasp_group_name = "grasp_plan_group"
    while True:
        try:
            grasp_group = moveit_commander.MoveGroupCommander(grasp_group_name, ns="/iiwa",
                                                              robot_description="/iiwa/robot_description")
            break
        except:
            if attempts < maxAttempst and not rospy.is_shutdown():
                rospy.logwarn("Failed to load grasp planner. Retrying...")
                attempts += 1
            else:
                raise

    # Load saved trajectories
    with open(jsonFileName, "r") as jsonFile:
        trajectories = json.load(jsonFile)

    # Set Positions
    uppose = createPoseFromJSONObj(trajectories["up_pose"])
    upJointState = trajectories["up_joint_state"]
    downPoses = [createPoseFromJSONObj(x) for x in trajectories["down_poses"]]

    # Load trajectories from start to position over conveyor belt
    downTrajectories = []
    for trajObj in trajectories["down_trajectories"]:
        trajectory = RobotTrajectory()
        genpy.message.fill_message_args(trajectory, trajObj)
        downTrajectories.append(trajectory)

    # Load trajectories from positions over the conveyor back to the start pose
    upTrajectories = []
    for trajObj in trajectories["up_trajectories"]:
        trajectory = RobotTrajectory()
        genpy.message.fill_message_args(trajectory, trajObj)
        upTrajectories.append(trajectory)

    # Load Open and closed gripper targets
    close_gripper_target = grasp_group.get_named_target_values("gripper_closed")
    open_gripper_target = grasp_group.get_named_target_values("gripper_open")

    # Setup speed and execution time
    execTime = genpy.Duration(0.5)
    speed = downTrajectories[trajectoryIndex].joint_trajectory.points[-1].time_from_start / execTime

    # Move iiwa arm to start pose before accepting calls
    trajectory = iiwa_group.plan(upJointState)
    iiwa_group.execute(trajectory)

    # Set first target pose to default down position
    targetPose = copy.deepcopy(downPoses[trajectoryIndex])

    # Create a subscriber that executes grasp motions
    adaptive_sub = rospy.Subscriber("/adaptive_trigger", std_msgs.msg.Bool, callback, queue_size=1)
    reactive_sub = rospy.Subscriber("/reactive_trigger", std_msgs.msg.Bool, callback, queue_size=1)

    camera_sub = rospy.Subscriber("/camera/image_raw", Image, updateTargetPose, queue_size=1)

    time_pub = rospy.Publisher("/traj_execution_time", std_msgs.msg.Duration, queue_size=10)

    time_service = rospy.Service("/set_traj_execution_time", SetDuration, service_callback)

    while not rospy.is_shutdown():
        # Time Pub
        duration = std_msgs.msg.Duration(execTime)
        time_pub.publish(duration)

        # Make sure arm isn't stuck
        if moveLock.acquire(False):
            try:
                iiwa_group.go(upJointState)
                grasp_group.go(open_gripper_target)
            except:
                moveLock.release()
                raise

            moveLock.release()

        sleep(10)
