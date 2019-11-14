#! /usr/bin/env python

#
# @package: franka_interface
# @metapackage: franka_ros_interface 
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#
# @info: 
#   Utility script for stopping gripper.
#     Usage:
#         stop_gripper.py <arg>
#
#     @Args:
#         <arg> :  -r (--release): optional argument to make the gripper release and go to max width 
#
import rospy
import argparse
import actionlib
from actionlib_msgs.msg import GoalStatus

from franka_gripper.msg import ( MoveAction, MoveGoal,
                                 StopAction, StopGoal )

def _active_cb():
    rospy.loginfo("GripperInterface: '{}' request active.".format(_caller))

def _feedback_cb(msg):
    rospy.loginfo("GripperInterface: '{}' request feedback: \n\t{}".format(_caller,msg))

def _done_cb(status, result):
    rospy.loginfo("GripperInterface: '{}' complete. Result: \n\t{}".format(_caller, result))

if __name__ == '__main__':
    
    rospy.init_node("gripper_stop_node")

    parser = argparse.ArgumentParser("Stop current gripper action")
    parser.add_argument('-r', '--release', action='store_true',
                        help='Print current robot state')

    args = parser.parse_args(rospy.myargv()[1:])


    _caller = "stop_gripper"
    stop_action_client = actionlib.SimpleActionClient("/franka_ros_interface/franka_gripper/stop", StopAction)
    stop_action_client.wait_for_server()

    stop_action_client.send_goal(StopGoal(), done_cb =_done_cb, active_cb = _active_cb, feedback_cb = _feedback_cb)

    result = stop_action_client.wait_for_result(rospy.Duration(15.))

    if args.release and result:
        _caller = "gripper_release"
        move_action_client = actionlib.SimpleActionClient("/franka_ros_interface/franka_gripper/move", MoveAction)
        move_action_client.wait_for_server()

        goal = MoveGoal()
        goal.width = 0.5
        goal.speed = 0.05

        move_action_client.send_goal(goal, done_cb =_done_cb, active_cb = _active_cb, feedback_cb = _feedback_cb)

        result = move_action_client.wait_for_result(rospy.Duration(15.))





