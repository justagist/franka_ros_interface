import rospy

from franka_interface import ArmInterface

if __name__ == '__main__':
	rospy.init_node("test_robot")
	r = ArmInterface()
	cm = r.get_controller_manager()