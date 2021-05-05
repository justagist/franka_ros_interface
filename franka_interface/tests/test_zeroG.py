import rospy
import numpy as np
from franka_interface import ArmInterface

"""
:info:
    This code is for testing the torque controller. The robot should be in zeroG mode, i.e.
    It should stay still when no external force is acting, but should be very compliant
    (almost no resistance) when pushed. This code sends zero torque command continuously,
    which means the internal gravity compensation is the only torque being sent to the 
    robot. 

    Test by pushing robot.

    DO NOT RUN THIS CODE WITH CUSTOM END-EFFECTOR UNLESS YOU KNOW WHAT YOU'RE DOING!

    WARNING: This code only works if the robot model is good! If you have installed custom
        end-effector, the gravity compensation may not be good unless you have incorporated
        the model to the FCI via Desk!!
"""

if __name__ == '__main__':
    rospy.init_node("test_zeroG")
    r = ArmInterface() # create arm interface instance (see https://justagist.github.io/franka_ros_interface/DOC.html#arminterface for all available methods for ArmInterface() object)

    rospy.loginfo("Commanding...\n")

    r.move_to_neutral() # move robot to neutral pose

    rate = rospy.Rate(1000)

    joint_names = r.joint_names()

    while not rospy.is_shutdown():

        r.set_joint_torques(dict(zip(joint_names, [0.0]*7))) # send 0 torques
        rate.sleep()