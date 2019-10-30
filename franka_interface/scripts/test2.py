import argparse

import sys


from copy import copy


import rospy


import actionlib


from control_msgs.msg import (

    FollowJointTrajectoryAction,

    FollowJointTrajectoryGoal,

)

from trajectory_msgs.msg import (

    JointTrajectoryPoint,

)


class Trajectory(object):

    def __init__(self):

        ns = '/position_joint_trajectory_controller/'

        self._client = actionlib.SimpleActionClient(

            ns + "follow_joint_trajectory",

            FollowJointTrajectoryAction,

        )

        self._goal = FollowJointTrajectoryGoal()

        self._goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
  'panda_joint7']

        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))

        if not server_up:

            rospy.logerr("Timed out waiting for Joint Trajectory"

                         " Action Server to connect. Start the action server"

                         " before running example.")

            rospy.signal_shutdown("Timed out waiting for Action Server")

            sys.exit(1)

    def add_point(self, positions, time):

        point = JointTrajectoryPoint()

        point.positions = copy(positions)

        point.time_from_start = rospy.Duration(time)

        self._goal.trajectory.points.append(point)


    def start(self):

        self._goal.trajectory.header.stamp = rospy.Time.now()

        self._client.send_goal(self._goal)


    def stop(self):

        self._client.cancel_goal()

    def wait(self, timeout=15.0):

        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):

        return self._client.get_result()

    def clear(self):

        self._goal = FollowJointTrajectoryGoal()

        self._goal.trajectory.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
  'panda_joint7']


# [0.015315478498861631, -0.7365994791956655, 0.011295900681228078, -2.318460295727378, -0.005154412497792537, 1.575781920671463, 0.7881043608970364, 0.03991252928972244, 0.03991252928972244]

def main():

    """RSDK Joint Trajectory Example: Simple Action Client


    Creates a client of the Joint Trajectory Action Server

    to send commands of standard action type,

    control_msgs/FollowJointTrajectoryAction.


    Make sure to start the joint_trajectory_action_server.py

    first. Then run this example on a specified limb to

    command a short series of trajectory points for the arm

    to follow.

    """

    # arg_fmt = argparse.RawDescriptionHelpFormatter

    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,

    #                                  description=main.__doc__)

    # required = parser.add_argument_group('required arguments')

    # required.add_argument(

    #     '-l', '--limb', required=True, choices=['left', 'right'],

    #     help='send joint trajectory to which limb'

    # )

    # args = parser.parse_args(rospy.myargv()[1:])

    # limb = args.limb

    traj = Trajectory()

    rospy.on_shutdown(traj.stop)

    # Command Current Joint Positions first

    # limb_interface = baxter_interface.limb.Limb(limb)

    current_angles = [0.015315478498861631, -0.7365994791956655, 0.011295900681228078, -2.318460295727378, -0.005154412497792537, 1.575781920671463, 0.7881043608970364]

    traj.add_point(current_angles, 0.0)


    # Command Current Joint Positions first

    # limb_interface = baxter_interface.limb.Limb(limb)

    # current_angles = [0.015315478498861631, -0.7365994791956655, 0.011295900681228078, -2.318460295727378, -0.005154412497792537, 1.575781920671463, 0.7881043608970364]

    # traj.add_point(current_angles, 0.0)


    p1 = current_angles

    traj.add_point(p1, 7.0)

    traj.add_point([x * 0.9 for x in p1], 9.0)

    traj.add_point([x * 1.1 for x in p1], 12.0)

    traj.start()

    traj.wait(15.0)

    print("Exiting - Joint Trajectory Action Test Complete")


if __name__ == "__main__":
    rospy.init_node('test')
    main()
