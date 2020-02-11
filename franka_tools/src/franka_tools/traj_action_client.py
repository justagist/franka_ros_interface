# /***************************************************************************

# 
# @package: franka_tools
# @metapackage: franka_ros_interface
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2020, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

import rospy
import sys
import actionlib
from copy import copy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class TrajActionClient(object):
    def __init__(self, joint_names, ns = "franka_ros_interface", controller_name = "position_joint_trajectory_controller"):
        self._joint_names = joint_names
        self._controller_name = controller_name

        self._client = actionlib.SimpleActionClient("/%s/follow_joint_trajectory"%(controller_name),
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(3.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def add_point(self, time, positions=None, velocities=None, torques=None):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(time)

        if "position_joint" in self._controller_name:
            if positions is None:
                raise ValueError("Positions must be specified for a position-based controller")
            point.positions = copy(positions)
            if velocities is None:
                point.velocities = [0.0001 for n in positions]
            else:
                point.velocities = copy(velocities)
        elif "velocity_joint" in self._controller_name:
            if velocities is None:
                raise ValueError("Velocities must be specified for a velocity-based controller")
            point.velocities = copy(velocities)
        elif "effort_joint_torque" in self._controller_name:
            # torques
            if torques is None:
                raise ValueError("Torques must be specified for a torque-based controller")
            point.torques = copy(torques)

        else:
            raise ValueError("Controller Type Not Recognized")

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
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = self._joint_names
