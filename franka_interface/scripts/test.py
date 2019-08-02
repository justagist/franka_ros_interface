#! /usr/bin/env python
import time
import os
import argparse
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
import copy


class TestMotion:

    def __init__(self):


        self._joint_state = JointState()

        pub = rospy.Publisher('/joint_states_desired', JointState, queue_size = 1)
        rospy.Subscriber('/joint_states', JointState, self.callback, queue_size=1)    

    def callback(self, msg):

        joint_state = copy.deepcopy(msg)

        joint_state.position




if __name__ == '__main__':
    
    rospy.init_node('test')

    a = TestMotion()
    # print len(a._joint_state.position)
    rospy.spin()
