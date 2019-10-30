
import rospy
import franka_dataflow
from sensor_msgs.msg import JointState

class Gripper(object):
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

        print "this", ns

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
    

    def joint_names(self):
        """
        Return the names of the joints.

        """
        return self._joint_names


    def _joint_states_callback(self, msg):

        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]
