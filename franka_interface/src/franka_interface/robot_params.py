import sys
import rospy
import socket
from franka_core_msgs.msg import JointLimits

def _log_networking_error():
    print ("Failed to connect to the ROS parameter server!\n"
           "Please check to make sure your ROS networking is "
           "properly configured:\n")
    sys.exit()

class RobotParams(object):
    """
    Interface class for essential ROS parameters on Intera robot.
    """

    def __init__(self):

        if self._robot_in_simulation():
            self._in_sim = True
            self._ns = "/panda_simulator"
            rospy.loginfo("Robot control running in Simulation Mode!")
            self._robot_ip="sim"
        else:
            self._ns = "/franka_ros_interface" 
            self._in_sim = False
            self._robot_ip = self.get_robot_ip()

        self._robot_name = self.get_robot_name()

    def get_base_namespace(self):
        return self._ns

    def _robot_in_simulation(self):
        sim = False
        try:
            sim = rospy.get_param("/SIMULATOR_")
        except KeyError:
            sim = False
        except (socket.error, socket.gaierror):
            _log_networking_error()

        return sim

    def get_neutral_pose(self):
        try:
            neutral_pose = rospy.get_param("/robot_config/neutral_pose")
        except KeyError:
            rospy.logerr("RobotParam:robot_ip cannot detect neutral joint pos."
                         " under param /franka_control/neutral_pose")
        except (socket.error, socket.gaierror):
            _log_networking_error()

        return neutral_pose


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
            joint_names = rospy.get_param("/robot_config/joint_names")
        except KeyError:
            rospy.logerr(("RobotParam:get_joint_names cannot detect joint_names for arm"))
        except (socket.error, socket.gaierror):
            _log_networking_error()
        return joint_names

    def get_gripper_joint_names(self):

        joint_names = list()
        try:
            joint_names = rospy.get_param("/gripper_config/joint_names")
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
        @return: name of the robot
        """
        robot_name = None
        try:
            robot_name = rospy.get_param("/robot_config/arm_id")
        except KeyError:
            rospy.logerr("RobotParam:get_robot_name cannot detect robot name"
                         " under param /robot_config/arm_id")
        except (socket.error, socket.gaierror):
            _log_networking_error()
        return robot_name

    def get_joint_limits(self):

        lims = JointLimits()
        lims.joint_names = self.get_joint_names()

        try:
            vel_lim = rospy.get_param("/robot_config/joint_config/joint_velocity_limit")
        except KeyError:
            rospy.logerr("RobotParam:get_joint_limits cannot detect robot joint velocity limits"
                         " under param /robot_config/joint_config/joint_velocity_limit")
        except (socket.error, socket.gaierror):
            _log_networking_error()

        try:
            pos_min_lim = rospy.get_param("/robot_config/joint_config/joint_position_limit/lower")
        except KeyError:
            rospy.logerr("RobotParam:get_joint_limits cannot detect robot joint position lower limits"
                         " under param /robot_config/joint_config/joint_position_limit/lower")
        except (socket.error, socket.gaierror):
            _log_networking_error()

        try:
            pos_max_lim = rospy.get_param("/robot_config/joint_config/joint_position_limit/upper")
        except KeyError:
            rospy.logerr("RobotParam:get_joint_limits cannot detect robot joint position upper limits"
                         " under param /robot_config/joint_config/joint_position_limit/upper")
        except (socket.error, socket.gaierror):
            _log_networking_error()

        try:
            eff_lim = rospy.get_param("/robot_config/joint_config/joint_effort_limit")
        except KeyError:
            rospy.logerr("RobotParam:get_joint_limits cannot detect robot joint torque limits"
                         " under param /robot_config/joint_config/joint_effort_limit")
        except (socket.error, socket.gaierror):
            _log_networking_error()

        try:
            acc_lim = rospy.get_param("/robot_config/joint_config/joint_acceleration_limit")
        except KeyError:
            rospy.logerr("RobotParam:get_joint_limits cannot detect robot joint acceleration limits"
                         " under param /robot_config/joint_config/joint_acceleration_limit")
        except (socket.error, socket.gaierror):
            _log_networking_error()


        for i in range(len(lims.joint_names)):
            lims.position_upper.append(pos_max_lim[lims.joint_names[i]])
            lims.position_lower.append(pos_min_lim[lims.joint_names[i]])
            lims.velocity.append(vel_lim[lims.joint_names[i]])
            lims.accel.append(acc_lim[lims.joint_names[i]])
            lims.effort.append(eff_lim[lims.joint_names[i]])

        return lims



if __name__ == '__main__':
    
    rp = RobotParams()
    # print rp.__dict__
    print rp.get_robot_ip()
    print rp.get_robot_name()
    print rp.get_joint_names()


