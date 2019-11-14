#! /usr/bin/env python

# 
# Modified from: Rethink Robotics Intera SDK
# Modified by: Saif Sidhik <sxs1412@bham.ac.uk>
# Modified for: franka_ros_interface
# 
# @package: franka_interface
# @metapackage: franka_ros_interface 
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#
# @info: 
#   Utility script for enabling, disabling and checking status of robot.
#     Usage:
#         enable_robot.py <arg>
#
#     @Args:
#         <arg> :  -s (--state) / -e (--enable) / -d (--disable) / -r (--reset) / -S (--stop) 
#
#     Not implemented: Disabling robot.
#     Todo: Change reset request to service request instead of publishing message to topic.
#

import argparse
import os
import sys

import rospy

import franka_interface

def main():
    rospy.init_node('panda_robot_enable')
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--state', const='state',
                        dest='actions', action='append_const',
                        help='Print current robot state')
    parser.add_argument('-e', '--enable', const='enable',
                        dest='actions', action='append_const',
                        help='Enable the robot')
    parser.add_argument('-d', '--disable', const='disable',
                        dest='actions', action='append_const',
                        help='Disable the robot')
    parser.add_argument('-r', '--reset', const='reset',
                        dest='actions', action='append_const',
                        help='Reset the robot')
    parser.add_argument('-S', '--stop', const='stop',
                        dest='actions', action='append_const',
                        help='Stop the robot')
    args = parser.parse_args(rospy.myargv()[1:])

    if args.actions == None:
        parser.print_usage()
        parser.exit(0, "No action defined\n")

    rs = franka_interface.RobotEnable()
    try:
        for act in args.actions:
            if act == 'state':
                print rs.state()
            elif act == 'enable':
                rs.enable()
            elif act == 'disable':
                rs.disable()
            elif act == 'reset':
                rs.reset()
            elif act == 'stop':
                rs.stop()
    except Exception, e:
        rospy.logerr(e)

    return 0

if __name__ == '__main__':
    sys.exit(main())
