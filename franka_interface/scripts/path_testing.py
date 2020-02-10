#! /usr/bin/env python


"""
 @info: 
   commands robot to move to neutral pose

"""

import rospy
import copy
import IPython
from franka_interface import ArmInterface, GripperInterface

if __name__ == '__main__':
    rospy.init_node("path_testing")
    arm = ArmInterface()
    hand = GripperInterface()

    #starting_pose = arm.joint_angles()
    #raw_input("Move to neutral?")
    #arm.move_to_neutral()
    #raw_input("Move back to start?")
    #arm.move_to_joint_positions(starting_pose)
    starting_pose = arm.joint_angles()

    q0 = {'panda_joint1':  1.598,
          'panda_joint2':  0.065,
          'panda_joint3':  0.675,
          'panda_joint4': -2.189,
          'panda_joint5': -0.065,
          'panda_joint6':  2.234,
          'panda_joint7': -1.518}

    q1 = {'panda_joint1':  1.771,
          'panda_joint2':  0.398,
          'panda_joint3':  1.670,
          'panda_joint4': -1.476,
          'panda_joint5': -0.400, 
          'panda_joint6':  1.426,
          'panda_joint7': -0.474}

    q2 = copy.deepcopy(starting_pose)
    q2['panda_joint2'] += 0.2
    q2['panda_joint6'] -= 0.4
    q2['panda_joint5'] -= 0.4
    
    position_path = [q0, q1, q2]
    #raw_input("Execute path?")
    #arm.execute_position_path(position_path)
   

    v0 = {'panda_joint1':  0.0,
          'panda_joint2':  0.0,
          'panda_joint3':  0.0,
          'panda_joint4':  0.0,
          'panda_joint5':  0.0, 
          'panda_joint6':  0.0,
          'panda_joint7':  0.0}

    t0 = {'panda_joint1': 50,
          'panda_joint2': 50,
          'panda_joint3': 50,
          'panda_joint4': 50,
          'panda_joint5': 10,
          'panda_joint6': 10,
          'panda_joint7': 10}
    IPython.embed()
