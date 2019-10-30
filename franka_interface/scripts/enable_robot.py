#! /usr/bin/env python
import argparse
import os
import sys

import rospy

import franka_interface

def main():
    rospy.init_node('sdk_robot_enable')
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
    # print args.actions
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
