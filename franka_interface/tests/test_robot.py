import rospy
import numpy as np
from franka_interface import ArmInterface

if __name__ == '__main__':
    rospy.init_node("test_robot")
    r = ArmInterface()
    cm = r.get_controller_manager()

    rospy.loginfo("Commanding...\n")
    elapsed_time_ = rospy.Duration(0.0)
    period = rospy.Duration(0.005)

    initial_pose = r.joint_angles()

    count = 0
    rate = rospy.Rate(1000)

    vals = r.joint_angles()
    while not rospy.is_shutdown():

        elapsed_time_ += period

        delta = 3.14 / 16.0 * (1 - np.cos(3.14 / 5.0 * elapsed_time_.to_sec())) * 0.2

        for j in r.joint_names():
            if j == r.joint_names()[4]:
                vals[j] = initial_pose[j] - delta
            else:
                vals[j] = initial_pose[j] + delta

        r.set_joint_velocities(vals)
        rate.sleep()