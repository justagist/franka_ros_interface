
from robot_params import RobotParams


from copy import deepcopy
import rospy



class Arm(object):

    """ 
    Interface Class for an arm of Franka Panda robot
    """

    def __init__(self):

        params = RobotParams()

        joint_names = params.get_joint_names()
        if not joint_names:
            rospy.logerr("Cannot detect joint names for arm on this "
                         "robot. Exiting Arm.init().")
                         
            return   

        self.name = params.get_robot_name()