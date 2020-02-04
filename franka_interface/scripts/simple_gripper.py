#! /usr/bin/env python

import rospy, IPython
from franka_interface import GripperInterface

if __name__ == '__main__':
    
    rospy.init_node("gripper_test")

    hand = GripperInterface()
    IPython.embed()
    #hand.open()
    #hand.close()
