# @package: franka_interface
# @metapackage: franka_ros_interface 
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#
# @info: 
#   Inteface Class for Franka gripper.
#
#   Todo: send control commands (close, open, move to position)
#

import rospy
import franka_dataflow
from copy import deepcopy
from sensor_msgs.msg import JointState

class GripperInterface(object):
    """
    Interface class for a gripper on the Franka Panda Robot.
    """

    def __init__(self, gripper_joint_names, ns = 'franka', calibrate=True):
        """
        Constructor.

        @type calibrate: bool
        @param calibrate: Attempts to calibrate the gripper when initializing class (defaults True)
        """
        
        self.name = ns + '_gripper'

        ns = '/'+self.name +'/'

        self._joint_positions = dict()
        self._joint_names = gripper_joint_names
        self._joint_velocity = dict()
        self._joint_effort = dict()

        self._joint_states_state_sub = rospy.Subscriber(ns + 'joint_states', JointState, self._joint_states_callback, queue_size = 1, tcp_nodelay = True)

        # ----- Wait for the gripper device status to be true
        if not franka_dataflow.wait_for(lambda: len(self._joint_positions.keys()) > 0, timeout=2.0, timeout_msg=("FrankaGripper: Failed to get gripper. No gripper attached on the robot."), raise_on_error = False ):
            self._exists = False
            return 
        self._exists = True

    @property
    def exists(self):
        return self._exists


    def _joint_states_callback(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.

        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names

    def joint_position(self, joint):
        """
        Return the requested joint position.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: position individual joint
        """
        return self._joint_positions[joint]

    def joint_positions(self):
        """
        Return all joint positions.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to pos
        """
        return deepcopy(self._joint_positions)

    def joint_ordered_positions(self):
        """
        Return all joint positions.

        @rtype: [double]
        @return: joint positions ordered by joint_names.
        """
        return [self._joint_positions[name] for name in self._joint_names]

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


