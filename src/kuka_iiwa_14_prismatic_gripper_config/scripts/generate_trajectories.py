#!/usr/bin/env python

import sys
import json
import yaml
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion


def createPoseFromJSONObj(posejsonobj):
    return Pose(position=Point(x=posejsonobj["position"]["x"], y=posejsonobj["position"]["y"], z=posejsonobj["position"]["z"]),
                orientation=Quaternion(x=posejsonobj["orientation"]["x"], y=posejsonobj["orientation"]["y"], z=posejsonobj["orientation"]["z"], w=posejsonobj["orientation"]["w"]))


def createPoseFromJSONStr(posejsonstr):
    return createPoseFromJSONObj(json.loads(posejsonstr))


def createArrayFromJSONObj(posejsonobj):
    return [posejsonobj["position"]["x"], posejsonobj["position"]["y"], posejsonobj["position"]["z"], posejsonobj["orientation"]["x"],
            posejsonobj["orientation"]["y"], posejsonobj["orientation"]["z"], posejsonobj["orientation"]["w"]]


def createArrayFromJSONStr(posejsonstr):
    return createArrayFromJSONObj(json.loads(posejsonstr))


def createList(startVal, endVal, n):
    delta = endVal - startVal
    retVal = []
    for i in range(0, (n + 1)):
        retVal.append(startVal + delta * i / n)

    return retVal


jsonFileName = sys.argv[1]
print sys.argv
print jsonFileName


with open(jsonFileName, mode="r") as json_file:
    data = json.load(json_file)

uppose = createPoseFromJSONObj(data["up_pose"])

startpose = createPoseFromJSONObj(data["over_conveyor_poses"][0])
endpose = createPoseFromJSONObj(data["over_conveyor_poses"][1])

n = 20
x = createList(startpose.position.x, endpose.position.x, n)
y = createList(startpose.position.y, endpose.position.y, n)
z = createList(startpose.position.z, endpose.position.z, n)
rx = createList(startpose.orientation.x, endpose.orientation.x, n)
ry = createList(startpose.orientation.y, endpose.orientation.y, n)
rz = createList(startpose.orientation.z, endpose.orientation.z, n)
rw = createList(startpose.orientation.w, endpose.orientation.w, n)

deltaPoses = []
for i in range(0, n):
    deltaPoses.append(Pose(position=Point(x=x[i], y=y[i], z=z[i]), orientation=Quaternion(x=rx[i], y=ry[i], z=rz[i], w=rw[i])))

print deltaPoses

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('group_plan',
                anonymous=True)

robot = moveit_commander.RobotCommander(ns="/iiwa", robot_description="/iiwa/robot_description")
scene = moveit_commander.PlanningSceneInterface(ns="/iiwa")

iiwa_group = moveit_commander.MoveGroupCommander("iiwa_plan_group", ns="/iiwa", robot_description="/iiwa/robot_description")

traj = iiwa_group.plan(uppose)
iiwa_group.execute(traj)

upJointState = iiwa_group.get_current_joint_values()

deltaDownTrajs = []
for i in range(0,n):
    deltaDownTrajs.append(iiwa_group.plan(deltaPoses[i]))

deltaUpTrajs = []
for i in range(0,n):
    traj = iiwa_group.plan(deltaPoses[i])
    iiwa_group.execute(traj)
    deltaUpTrajs.append(iiwa_group.plan(uppose))

downTrajListStrings = []
for traj in deltaDownTrajs:
    downTrajListStrings.append(yaml.load(str(traj)))

json.dumps(downTrajListStrings)

upTrajListStrings = []
for traj in deltaUpTrajs:
    upTrajListStrings.append(yaml.load(str(traj)))

json.dumps(upTrajListStrings)

dataFileName = sys.argv[2]
with open(dataFileName, mode="w") as jsonFile:
    jsonFile.write(json.dumps({"up_pose": yaml.load(str(uppose)),
                               "up_joint_state": upJointState,
                               "down_poses": [yaml.load(str(x)) for x in deltaPoses],
                               "up_trajectories": upTrajListStrings,
                               "down_trajectories": downTrajListStrings}))
