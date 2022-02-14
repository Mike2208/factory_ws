#!/usr/bin/env python

import ctypes
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # suppress warnings
import numpy as np
import matplotlib.pyplot as plt

from working_memory.ros_working_memory import WorkingMemory

import rospy
from std_msgs.msg import Bool, Int8


class RosWorkingMemory():
    def __init__(self):
        rospy.init_node('working_memory')

        self.human_detection = 0

        self.out_pub = rospy.Publisher('wm_speed', Int8, queue_size=10)
        self.in_sub  = rospy.Subscriber('wm_human_detection', Int8)

        self.wm = WorkingMemory(wm_type='gru')

    def human_detection_cb(self, hd):
        self.human_detection = hd.data

    def loop(self):
        d = rospy.Duration.from_sec(0.5)
        for i in range(0,20):
            self.wm.input(self.human_detection)
            
            speed = self.wm.output()

            out = Int8()
            out.data = ctypes.c_int8(speed).value
            print(out.data)
            self.out_pub.publish(out)

            rospy.sleep(d)

            if rospy.is_shutdown():
                return -1

        return 0


if __name__ == "__main__":
    rwm = RosWorkingMemory()
    rwm.loop()
