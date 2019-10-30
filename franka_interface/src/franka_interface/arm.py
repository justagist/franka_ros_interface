import rospy
import numpy as np
import quaternion
from copy import deepcopy
import collections, warnings

from franka_core_msgs.msg import JointCommand
from franka_core_msgs.msg import RobotState, TipState
from sensor_msgs.msg import JointState

import franka_dataflow
import franka_interface
from robot_params import RobotParams


class Arm(object):

    """ 
    Interface Class for an arm of Franka Panda robot
    """
    # Containers
    Point = collections.namedtuple('Point', ['x', 'y', 'z'])

    def __init__(self, synchronous_pub=False):
        """
        Constructor.

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

        params = RobotParams()

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
        self._errors = dict()
        self._collision_state = False
        self._tip_states = None
        self._jacobian = None

        ns = self.name+'/'

        self._command_msg = JointCommand()

        queue_size = None if synchronous_pub else 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._pub_joint_cmd = rospy.Publisher(
                ns + 'joint_command',
                JointCommand,
                tcp_nodelay=True,
                queue_size=queue_size)


        _robot_state_subscriber = rospy.Subscriber(
            '/custom_franka_state_controller/franka_state',
            RobotState,
            self._on_robot_state,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = '/custom_franka_state_controller/joint_states'
        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        gripper_state_topic = '/franka_gripper/joint_states'
        _joint_state_sub = rospy.Subscriber(
            gripper_state_topic,
            JointState,
            self._on_gripper_states,
            queue_size=1,
            tcp_nodelay=True)

        _cartesian_state_sub = rospy.Subscriber(
            '/custom_franka_state_controller/tip_state',
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

        self._configure_gripper(params.get_gripper_joint_names())
        
        rospy.sleep(2.)

    def _configure_gripper(self, gripper_joint_names):

        self._gripper = franka_interface.Gripper(gripper_joint_names = gripper_joint_names)
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

        jac = msg.O_Jac_EE
        self._jacobian = np.asarray( [ [ jac[0], jac[6],  jac[12], jac[18], jac[24], jac[30], jac[36] ],
                                       [ jac[1], jac[7],  jac[13], jac[19], jac[25], jac[31], jac[37] ],
                                       [ jac[2], jac[8],  jac[14], jac[20], jac[26], jac[32], jac[38] ],
                                       [ jac[3], jac[9],  jac[15], jac[21], jac[27], jac[33], jac[39] ],
                                       [ jac[4], jac[10], jac[16], jac[22], jac[28], jac[34], jac[40] ],
                                       [ jac[5], jac[11], jac[17], jac[23], jac[29], jac[35], jac[41] ] ] )

        # print msg.O_T_EE_d


    def _on_endpoint_state(self, msg):

        cart_pose_trans_mat = np.asarray( [ [msg.O_T_EE[0],msg.O_T_EE[4],msg.O_T_EE[8],msg.O_T_EE[12] ],
                                            [msg.O_T_EE[1],msg.O_T_EE[5],msg.O_T_EE[9],msg.O_T_EE[13] ],
                                            [msg.O_T_EE[2],msg.O_T_EE[6],msg.O_T_EE[10],msg.O_T_EE[14] ],
                                            [msg.O_T_EE[3],msg.O_T_EE[7],msg.O_T_EE[11],msg.O_T_EE[15] ] ])

        self._cartesian_pose = {
            'position': cart_pose_trans_mat[:3,3],
            'orientation': quaternion.from_rotation_matrix(cart_pose_trans_mat[:3,:3]) }


        self._cartesian_velocity = {
                'linear': np.asarray([msg.O_dP_EE_d[0], msg.O_dP_EE_d[1], msg.O_dP_EE_d[2]]),
                'angular': np.asarray([msg.O_dP_EE_d[3], msg.O_dP_EE_d[4], msg.O_dP_EE_d[5]]) }
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
        # self._cartesian_effort = {
        #     'force': self.Point(
        #         msg.wrench.force.x,
        #         msg.wrench.force.y,
        #         msg.wrench.force.z,
        #     ),
        #     'torque': self.Point(
        #         msg.wrench.torque.x,
        #         msg.wrench.torque.y,
        #         msg.wrench.torque.z,
        #     ),
        # }


    def _on_gripper_states(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_angle[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]


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

          - 'position': Cartesian coordinates x,y,z in
                        namedtuple L{Limb.Point}
          - 'orientation': quaternion x,y,z,w in named tuple
                           L{Limb.Quaternion}
        """
        return deepcopy(self._cartesian_pose)



if __name__ == '__main__':
    rospy.init_node('test')
    r = Arm()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print r.joint_angles()
        rate.sleep()
