import rospy
import numpy as np
import quaternion
from copy import deepcopy
import collections, warnings

from franka_core_msgs.msg import JointCommand
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState

import franka_dataflow
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
            '/franka_state_controller/franka_states',
            FrankaState,
            self._on_robot_state,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = '/franka_state_controller/joint_states'
        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
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

        err_msg = ("%s ar, init failed to get current robot_state "
                   "from %s") % (self.name.capitalize(), ns + 'endpoint_state')
        franka_dataflow.wait_for(lambda: len(self._cartesian_pose.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)



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

        cart_pose_trans_mat = msg.O_T_EE
        self._cartesian_pose = {
            'position': self.Point(cart_pose_trans_mat[12], cart_pose_trans_mat[13], cart_pose_trans_mat[14]),
            'orientation': quaternion.from_rotation_matrix(np.asarray([[cart_pose_trans_mat[0],cart_pose_trans_mat[4],cart_pose_trans_mat[8]],
                                                                      [cart_pose_trans_mat[1],cart_pose_trans_mat[5],cart_pose_trans_mat[9]],
                                                                      [cart_pose_trans_mat[2],cart_pose_trans_mat[6],cart_pose_trans_mat[10]]]))}

        



if __name__ == '__main__':
    rospy.init_node('test')
    r = Arm()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print r._cartesian_pose
        rate.sleep()
