# 
# Modified from: Rethink Robotics Intera SDK
# Modified by: Saif Sidhik <sxs1412@bham.ac.uk>
# Modified for: franka_ros_interface
# 
# @package: franka_interface
# @metapackage: franka_ros_interface 
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#
# @info: 
#   Inteface Class for Franka robot arm.
#
#   Todo: send control commands (position, velocity, torque), get cartesian velocity, finish getting robot state
#

import enum
import rospy
import quaternion
import numpy as np
from copy import deepcopy
import collections, warnings
from rospy_message_converter import message_converter

from franka_core_msgs.msg import JointCommand
from franka_core_msgs.msg import RobotState, TipState
from sensor_msgs.msg import JointState

import franka_dataflow
import franka_interface
from robot_params import RobotParams

from franka_tools import FrankaFramesInterface, FrankaControllerManagerInterface

class ArmInterface(object):

    """ 
    Interface Class for an arm of Franka Panda robot
    """

    # Containers
    @enum.unique
    class RobotMode(enum.IntEnum):
        """
            Enum class for specifying and retrieving the current robot mode.
        """
        # ----- access using parameters name or value
        # ----- eg. RobotMode(0).name & RobotMode(0).value
        # ----- or  RobotMode['ROBOT_MODE_OTHER'].name & RobotMode['ROBOT_MODE_OTHER'].value

        ROBOT_MODE_OTHER                        = 0
        ROBOT_MODE_IDLE                         = 1
        ROBOT_MODE_MOVE                         = 2
        ROBOT_MODE_GUIDING                      = 3
        ROBOT_MODE_REFLEX                       = 4
        ROBOT_MODE_USER_STOPPED                 = 5
        ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY     = 6

    def __init__(self, ns = "/franka_ros_interface", synchronous_pub=False):
        """
        Constructor.
        
        @type ns: string
        @param ns: namespace used by franka_ros_interface nodes
        @type synchronous_pub: bool
        @param synchronous_pub: designates the JointCommand Publisher
            as Synchronous if True and Asynchronous if False.

            Synchronous Publishing means that all joint_commands publishing to
            the robot's joints will block until the message has been serialized
            into a buffer and that buffer has been written to the transport
            of every current Subscriber. This yields predicable and consistent
            timing of messages being delivered from this Publisher. However,
            when using this mode, it is possible for a blocking Subscriber to
            prevent the joint_command functions from exiting. Unless you need exact
            JointCommand timing, default to Asynchronous Publishing (False).

            For more information about Synchronous Publishing see:
            http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#queue_size:_publish.28.29_behavior_and_queuing
        """

        self._ns = ns
        params = RobotParams(ns)

        joint_names = params.get_joint_names()
        if not joint_names:
            rospy.logerr("Cannot detect joint names for arm on this "
                         "robot. Exiting Arm.init().")
                         
            return   

        self._joint_names = joint_names
        self.name = params.get_robot_name()
        self._joint_angle = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._cartesian_pose = dict()
        self._cartesian_velocity = dict()
        self._cartesian_effort = dict()
        self._stiffness_frame_effort = dict()
        self._errors = dict()
        self._collision_state = False
        self._tip_states = None
        self._jacobian = None
        self._cartesian_contact = None

        self._robot_mode = False

        ns = self._ns + self.name+'/'

        self._command_msg = JointCommand()

        self._frames_interface = FrankaFramesInterface()
        self._ctrl_manager = FrankaControllerManagerInterface()


        queue_size = None if synchronous_pub else 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._pub_joint_cmd = rospy.Publisher(
                ns + 'joint_command',
                JointCommand,
                tcp_nodelay=True,
                queue_size=queue_size)


        _robot_state_subscriber = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/franka_state',
            RobotState,
            self._on_robot_state,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = self._ns + '/custom_franka_state_controller/joint_states'
        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        # gripper_state_topic = '/franka_gripper/joint_states'
        # _joint_state_sub = rospy.Subscriber(
        #     gripper_state_topic,
        #     JointState,
        #     self._on_gripper_states,
        #     queue_size=1,
        #     tcp_nodelay=True)

        _cartesian_state_sub = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/tip_state',
            TipState,
            self._on_endpoint_state,
            queue_size=1,
            tcp_nodelay=True)

        # ns_pkn = "ExternalTools/" + limb + "/PositionKinematicsNode/"
        # self._iksvc = rospy.ServiceProxy(ns_pkn + 'IKService', SolvePositionIK)
        # self._fksvc = rospy.ServiceProxy(ns_pkn + 'FKService', SolvePositionFK)
        # rospy.wait_for_service(ns_pkn + 'IKService', 5.0)
        # rospy.wait_for_service(ns_pkn + 'FKService', 5.0)

        err_msg = ("%s arm init failed to get current joint_states "
                   "from %s") % (self.name.capitalize(), joint_state_topic)
        franka_dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

        err_msg = ("%s arm, init failed to get current tip_state "
                   "from %s") % (self.name.capitalize(), ns + 'tip_state')
        franka_dataflow.wait_for(lambda: len(self._cartesian_pose.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

        err_msg = ("%s arm, init failed to get current robot_state "
                   "from %s") % (self.name.capitalize(), ns + 'robot_state')
        franka_dataflow.wait_for(lambda: self._jacobian is not None,
                                 timeout_msg=err_msg, timeout=5.0)


        # if not self._frames_interface.EE_frame_is_reset():
        #     self.reset_EE_frame()

        self._configure_gripper(params.get_gripper_joint_names())

        if self.has_gripper():
            self.set_EE_frame_to_link('panda_hand')
        else:
            self.set_EE_frame_to_link('panda_link8')
        
        rospy.sleep(2.)

    def _configure_gripper(self, gripper_joint_names):

        self._gripper = franka_interface.GripperInterface(ns = self._ns, gripper_joint_names = gripper_joint_names)
        if not self._gripper.exists:
            self._gripper = None
            return

    def get_gripper(self):
        return self._gripper

    @property
    def has_gripper(self):
        return self._gripper is not None

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.

        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names

    def _on_joint_states(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def _on_robot_state(self, msg):

        self._robot_mode = self.RobotMode(msg.robot_mode)

        self._robot_mode_ok = (self._robot_mode.value != self.RobotMode.ROBOT_MODE_REFLEX) and (self._robot_mode.value != self.RobotMode.ROBOT_MODE_USER_STOPPED)

        jac = msg.O_Jac_EE
        self._jacobian = np.asarray(msg.O_Jac_EE).reshape(6,7,order = 'F')

        self._cartesian_velocity = {
                'linear': np.asarray([msg.O_dP_EE[0], msg.O_dP_EE[1], msg.O_dP_EE[2]]),
                'angular': np.asarray([msg.O_dP_EE[3], msg.O_dP_EE[4], msg.O_dP_EE[5]]) }

        self._cartesian_contact = msg.cartesian_contact
        self._cartesian_collision = msg.cartesian_collision

        self._joint_contact = msg.joint_contact
        self._joint_collision = msg.joint_collision

        self._frames_interface._update_frame_data(msg.F_T_EE)

        self.q_d = msg.q_d
        self.dq_d = msg.dq_d

        self._errors = message_converter.convert_ros_message_to_dictionary(msg.current_errors)


    # def get_current_EE_frame_transformation(self, as_mat = False):
    #     return self._flange_to_ee_trans_mat if not as_mat else np.asarray(self._flange_to_ee_trans_mat).reshape(4,4,order='F')

    def get_robot_status(self):
        """
        Return dict with all robot status information.

        @rtype: dict
        @return: ['robot_mode' (RobotMode object), 'robot_status' (bool), 'errors' (dict() of errors and their truth value), 'error_in_curr_status' (bool)]
        """
        return {'robot_mode': self._robot_mode, 'robot_status': self._robot_mode_ok, 'errors': self._errors, 'error_in_current_state' : self.error_in_current_state()}

    def in_safe_state(self):
        """
        Return True if the specified limb is in safe state (no collision, reflex, errors etc.).

        @rtype: bool
        @return: True if the arm is in safe state, False otherwise.
        """
        return self._robot_mode_ok and not self.error_in_current_state()

    def error_in_current_state(self):
        """
        Return True if the specified limb has experienced an error.

        @rtype: bool
        @return: True if the arm has error, False otherwise.
        """
        return not all([e == False for e in self._errors.values()])

    def what_errors(self):
        """
        Return list of error messages if there is error in robot state

        @rtype: [str]
        @return: list of names of current errors in robot state
        """
        return [e for e in self._errors if self._errors[e] == True] if self.error_in_current_state() else None


    def _on_endpoint_state(self, msg):

        cart_pose_trans_mat = np.asarray(msg.O_T_EE).reshape(4,4,order='F')

        self._cartesian_pose = {
            'position': cart_pose_trans_mat[:3,3],
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }



        
        #     'linear': self.Point(
        #         msg.twist.linear.x,
        #         msg.twist.linear.y,
        #         msg.twist.linear.z,
        #     ),
        #     'angular': self.Point(
        #         msg.twist.angular.x,
        #         msg.twist.angular.y,
        #         msg.twist.angular.z,
        #     ),
        # }
        # # _wrench = {'force': (x, y, z), 'torque': (x, y, z)}
        self._cartesian_effort = {
            'force': np.asarray([ msg.O_F_ext_hat_K.wrench.force.x,
                                  msg.O_F_ext_hat_K.wrench.force.y,
                                  msg.O_F_ext_hat_K.wrench.force.z]),

            'torque': np.asarray([ msg.O_F_ext_hat_K.wrench.torque.x,
                                   msg.O_F_ext_hat_K.wrench.torque.y,
                                   msg.O_F_ext_hat_K.wrench.torque.z])
        }

        self._stiffness_frame_effort = {
            'force': np.asarray([ msg.K_F_ext_hat_K.wrench.force.x,
                                  msg.K_F_ext_hat_K.wrench.force.y,
                                  msg.K_F_ext_hat_K.wrench.force.z]),

            'torque': np.asarray([ msg.K_F_ext_hat_K.wrench.torque.x,
                                   msg.K_F_ext_hat_K.wrench.torque.y,
                                   msg.K_F_ext_hat_K.wrench.torque.z])
        }


    # def _on_gripper_states(self, msg):

    #     for idx, name in enumerate(msg.name):
    #         if name in self._joint_names:
    #             self._joint_angle[name] = msg.position[idx]
    #             self._joint_velocity[name] = msg.velocity[idx]
    #             self._joint_effort[name] = msg.effort[idx]


    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self._joint_angle[joint]

    def joint_angles(self):
        """
        Return all joint angles.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return deepcopy(self._joint_angle)

    def joint_ordered_angles(self):
        """
        Return all joint angles.

        @rtype: [double]
        @return: joint angles (rad) orded by joint_names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return [self._joint_angle[name] for name in self._joint_names]

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self._joint_velocity[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return deepcopy(self._joint_velocity)

    def joint_effort(self, joint):
        """
        Return the requested joint effort.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: effort in Nm of individual joint
        """
        return self._joint_effort[joint]

    def joint_efforts(self):
        """
        Return all joint efforts.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to effort (Nm) Values
        """
        return deepcopy(self._joint_effort)

    def endpoint_pose(self):
        """
        Return Cartesian endpoint pose {position, orientation}.

        @rtype: dict({str:L{Limb.Point},str:L{Limb.Quaternion}})
        @return: position and orientation as named tuples in a dict

        C{pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}}

          - 'position': np.array of x, y, z
          - 'orientation': quaternion x,y,z,w in quaternion format

        """
        return deepcopy(self._cartesian_pose)

    def reset_EE_frame(self):
        """
        Reset EE frame to default. (defined by FrankaFramesInterface.DEFAULT_TRANSFORMATIONS.EE_FRAME global variable defined above) 

        @rtype: [bool, str]
        @return: [success status of service request, error msg if any]
        """

        if self._frames_interface.EE_frame_is_reset():
            rospy.loginfo("FrankaArm: EE Frame already reset")
            return

        active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)

        rospy.loginfo("FrankaArm: Stopping motion controllers for resetting EE frame")
        for ctrlr in active_controllers:
            self._ctrl_manager.stop_controller(ctrlr.name)
        rospy.sleep(1.)

        self._frames_interface.reset_EE_frame()

        rospy.sleep(1.)
        rospy.loginfo("FrankaArm: Restarting previously active motion controllers.")
        for ctrlr in active_controllers:
            self._ctrl_manager.start_controller(ctrlr.name)
        rospy.sleep(1.)

    def set_EE_frame(self, frame):
        """
        Set new EE frame based on the transformation given by 'frame', which is the 
        transformation matrix defining the new desired EE frame with respect to the flange frame.
        Motion controllers are stopped for switching

        @type frame: [float (16,)] / np.ndarray (4x4) 
        @param frame: transformation matrix of new EE frame wrt flange frame (column major)
        @rtype: [bool, str]
        @return: [success status of service request, error msg if any]
        """

        frame = self._frames_interface._assert_frame_validity(frame)

        active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)
        rospy.sleep(1.)
        rospy.loginfo("FrankaArm: Stopping motion controllers for changing EE frame")
        for ctrlr in active_controllers:
            self._ctrl_manager.stop_controller(ctrlr.name)
        rospy.sleep(1.)

        retval = self._frames_interface.set_EE_frame(frame)

        rospy.sleep(1.)
        rospy.loginfo("FrankaArm: Restarting previously active motion controllers.")
        for ctrlr in active_controllers:
            self._ctrl_manager.start_controller(ctrlr.name)
        rospy.sleep(1.)

        return retval

    def set_EE_frame_to_link(self, frame_name, timeout = 5.0):
        """
        Set new EE frame to the same frame as the link frame given by 'frame_name'
        Motion controllers are stopped for switching

        @type frame_name: str 
        @param frame_name: desired tf frame name in the tf tree
        @rtype: [bool, str]
        @return: [success status of service request, error msg if any]
        """
        active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)

        rospy.loginfo("FrankaArm: Stopping motion controllers for changing EE frame")
        rospy.sleep(1.)
        for ctrlr in active_controllers:
            self._ctrl_manager.stop_controller(ctrlr.name)
        rospy.sleep(1.)

        self._frames_interface.set_EE_frame_to_link(frame_name = frame_name, timeout = timeout)

        rospy.sleep(1.)
        rospy.loginfo("FrankaArm: Restarting previously active motion controllers.")
        for ctrlr in active_controllers:
            self._ctrl_manager.start_controller(ctrlr.name)
        rospy.sleep(1.)


    def get_controller_manager(self):

        return self._ctrl_manager


    def get_frames_interface(self):

        return self._frames_interface


if __name__ == '__main__':
    rospy.init_node('test')
    r = Arm()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print r.joint_angles()
        rate.sleep()
