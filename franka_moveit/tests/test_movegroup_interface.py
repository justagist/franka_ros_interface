import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == '__main__':
    
    from franka_moveit import PandaMoveGroupInterface

    rospy.init_node("test_moveit")
    mvt = PandaMoveGroupInterface()

    g = mvt.gripper_group
    r = mvt.arm_group


