import sys
import rospy
import socket

def _log_networking_error():
    print ("Failed to connect to the ROS parameter server!\n"
           "Please check to make sure your ROS networking is "
           "properly configured:\n")
    sys.exit()

class RobotParams(object):
    """
    Interface class for essential ROS parameters on Intera robot.
    """

    def __init__(self, ns = ""):

        self._ns = ns
        self._robot_name = self.get_robot_name()
        self._robot_ip = self.get_robot_ip()

    def get_robot_ip(self):
        robot_ip = None
        try:
            robot_ip = rospy.get_param(self._ns + "/franka_control/robot_ip")
        except KeyError:
            rospy.logerr("RobotParam:robot_ip cannot detect robot ip."
                         " under param /franka_control/robot_ip")
        except (socket.error, socket.gaierror):
            _log_networking_error()

        return robot_ip

    def get_joint_names(self):
        """
        Return the names of the joints for the specified
        limb from ROS parameter.

        @rtype: list [str]
        @return: ordered list of joint names from proximal to distal
                 (i.e. shoulder to wrist). joint names for limb
        """
        joint_names = list()
        try:
            joint_names = rospy.get_param(self._ns + 
                            "/franka_control/joint_names")
        except KeyError:
            rospy.logerr(("RobotParam:get_joint_names cannot detect joint_names for arm"))
        except (socket.error, socket.gaierror):
            _log_networking_error()
        return joint_names

    def get_gripper_joint_names(self):

        joint_names = list()
        try:
            joint_names = rospy.get_param(self._ns + 
                            "/franka_gripper/joint_names")
        except KeyError:
            rospy.loginfo(("RobotParam:get_gripper_joint_names cannot detect joint_names for gripper. Gripper not connected to robot."))
            return None
        except (socket.error, socket.gaierror):
            _log_networking_error()
        return joint_names

    def get_robot_name(self):
        """
        Return the name of class of robot from ROS parameter.

        @rtype: str
        @return: name of the class of robot (eg. "sawyer", "baxter", etc.)
        """
        robot_name = None
        try:
            robot_name = rospy.get_param(self._ns + "/franka_control/arm_id")
        except KeyError:
            rospy.logerr("RobotParam:get_robot_name cannot detect robot name"
                         " under param /franka_control/arm_id")
        except (socket.error, socket.gaierror):
            _log_networking_error()
        return robot_name



if __name__ == '__main__':
    
    rp = RobotParams()
    # print rp.__dict__
    print rp.get_robot_ip()
    print rp.get_robot_name()
    print rp.get_joint_names()


