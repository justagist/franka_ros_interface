# /***************************************************************************

# 
# @package: franka_moveit
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

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from extended_planning_scene_interface import ExtendedPlanningSceneInterface


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @rtype: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
          if abs(actual[index] - goal[index]) > tolerance:
            return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class PandaMoveGroupInterface:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander()

        self._scene = ExtendedPlanningSceneInterface()

        self._arm_group = moveit_commander.MoveGroupCommander("panda_arm")

        try:
            rospy.get_param("/franka_gripper/robot_ip")
            self._gripper_group = moveit_commander.MoveGroupCommander("hand")
        except KeyError:
            rospy.loginfo(("FrankaGripper: could not detect gripper."))
            self._gripper_group = None
        except (socket.error, socket.gaierror):
            print ("Failed to connect to the ROS parameter server!\n"
           "Please check to make sure your ROS networking is "
           "properly configured:\n")
            sys.exit()

        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                           moveit_msgs.msg.DisplayTrajectory,
                                           queue_size=20)

        self._default_ee = 'panda_hand' if self._gripper_group else 'panda_link8'
        # self._arm_group.set_end_effector_link(self._default_ee)

        rospy.loginfo("PandaMoveGroupInterface: Setting default EE link to '{}' "
            "Use group.set_end_effector_link() method to change default EE.".format(self._default_ee))

        self._arm_group.set_max_velocity_scaling_factor(0.3)


    @property
    def robot_state_interface(self):
        """
            @return: The RobotCommander instance of this object
            @rype: moveit_commander.RobotCommander

            available methods: http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
        """
        return self._robot
    
    @property
    def scene(self):
        """
            @return: The RobotCommander instance of this object. This is an interface
                    to the world surrounding the robot
            @rype: moveit_commander.RobotCommander

            available_methods: http://docs.ros.org/indigo/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1PlanningSceneInterface.html
        """
        return self._scene

    @property
    def arm_group(self):
        """
        @return: The MoveGroupCommander instance of this object. This is an interface
            to one group of joints.  In this case the group is the joints in the Panda
            arm. This interface can be used to plan and execute motions on the Panda.
        @rtype: moveit_commander.MoveGroupCommander

            available_methods: http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
        """
        return self._arm_group

    @property
    def gripper_group(self):
        """
        @return: The MoveGroupCommander instance of this object. This is an interface
            to one group of joints.  In this case the group is the joints in the Panda
            arm. This interface can be used to plan and execute motions on the Panda.
        @rtype: moveit_commander.MoveGroupCommander

            available_methods: http://docs.ros.org/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
        """
        return self._gripper_group

    def go_to_joint_positions(self, positions, wait = True, tolerance = 0.005):
        """
            @return: status of joint motion plan execution
            @rtype: bool

            @param positions: target joint positions (ordered)
            @param wait: if True, function will wait for trajectory execution to complete
            @param tolerance: maximum error in final position for each joint to consider
                              task a success

            @type positions: [double]
            @type wait: bool
            @type tolerance: double
        """
        self._arm_group.clear_pose_targets()

        rospy.logdebug("Starting positions: {}".format(self._arm_group.get_current_joint_values()))

        self._arm_group.go(positions[:7], wait = wait)

        if len(positions) > 7:
            self._gripper_group.go(positions[7:], wait = wait)

        if wait:
            self._arm_group.stop()
            gripper_tolerance = True
            if len(positions)> 7: 
                self._gripper_group.stop()
                gripper_tolerance = all_close(positions[7:], self._gripper_group.get_current_joint_values(), tolerance)
            return all_close(positions[:7], self._arm_group.get_current_joint_values(), tolerance) and gripper_tolerance

        return True

    def plan_cartesian_path(self, poses):
        waypoints = []
        for pose in poses:
            waypoints.append(copy.deepcopy(pose))

        plan, fraction = self._arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def set_velocity_scale(self, value, group = "arm"):
        """
            Set the max velocity scale for executing planned motion.
            @param value: scale value (allowed (0,1] )
        """
        if group == "arm":
            self._arm_group.set_max_velocity_scaling_factor(value)
        elif group == "gripper":
            self._gripper_group.set_max_velocity_scaling_factor(value)
        else:
            raise ValueError("PandaMoveGroupInterface: Invalid group specified!")
        rospy.logdebug("PandaMoveGroupInterface: Setting velocity scale to {}".format(value))

    def plan_joint_path(self, joint_position):
        """
        @return plan for executing joint trajectory
        """
        return self._arm_group.plan(joint_position)

    def go_to_poses(self, poses):
        plan, _ = self.plan_cartesian_path(poses)
        raise NotImplementedError
        """
            
        """

    def close_gripper(self, wait = False):
        """
            Using named states defined in urdf.

            NOTE: If this named state is not found, your ros environment is
            probably not using the right panda_moveit_config package. Ensure
            that sourced package is from this repo -->
            https://github.com/justagist/panda_moveit_config

        """
        self._gripper_group.set_named_target("close")
        return self._gripper_group.go(wait = wait)

    def open_gripper(self, wait = False):
        """
            Using named states defined in urdf.

            NOTE: If this named state is not found, your ros environment is
            probably not using the right panda_moveit_config package. Ensure
            that sourced package is from this repo -->
            https://github.com/justagist/panda_moveit_config
        """
        self._gripper_group.set_named_target("open")
        return self._gripper_group.go(wait = wait)

    def display_trajectory(self, plan):

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self._robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self._display_trajectory_publisher.publish(display_trajectory)

    def move_to_neutral(self, wait = True):
        """
            Send arm group to neutral pose defined using named state in urdf.
        """
        self._arm_group.set_named_target("ready")
        return self._arm_group.go(wait = wait)

    def execute_plan(self, plan, group = "arm", wait = True):
        if group == "arm":
            self._arm_group.execute(plan, wait = wait)
        elif group == "gripper":
            self._gripper_group.execute(plan, wait = wait)
        else:
            raise ValueError("PandaMoveGroupInterface: Invalid group specified!")






        




    
    


