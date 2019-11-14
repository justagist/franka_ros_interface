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
import actionlib
import franka_dataflow
from copy import deepcopy
from sensor_msgs.msg import JointState

from actionlib_msgs.msg import GoalStatus

from franka_gripper.msg import ( GraspAction, GraspGoal, 
                                 HomingAction, HomingGoal,   
                                 MoveAction, MoveGoal,
                                 StopAction, StopGoal,
                                 GraspEpsilon )

class GripperInterface(object):
    """
    Interface class for the gripper on the Franka Panda robot.
    """

    def __init__(self, gripper_joint_names = ['panda_finger_joint1', 'panda_finger_joint2'], ns = 'franka_ros_interface', calibrate = False):
        """
        Constructor.

        @param gripper_joint_names    : Names of the finger joints
        @param ns                     : base namespace of interface ('frank_ros_interface'/'panda_simulator')
        @param calibrate              : Attempts to calibrate the gripper when initializing class (defaults True)

        @type calibrate               : bool
        @type gripper_joint_names     : [str]
        @type ns                      : str

        """
        
        self.name = ns + '/franka_gripper'

        ns = self.name +'/'

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

        if not self._exists:
            return


        self._homing_action_client = actionlib.SimpleActionClient("{}homing".format(ns), HomingAction)

        self._grasp_action_client = actionlib.SimpleActionClient("{}grasp".format(ns), GraspAction)

        self._move_action_client = actionlib.SimpleActionClient("{}move".format(ns), MoveAction)

        self._stop_action_client = actionlib.SimpleActionClient("{}stop".format(ns), StopAction)

        self._homing_action_client.wait_for_server()
        self._grasp_action_client.wait_for_server()
        self._move_action_client.wait_for_server()
        self._stop_action_client.wait_for_server()

        if calibrate:
            self.home_joints(wait_for_result = True)



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

        @param joint    : name of a joint
        @type joint     : str

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

        @param joint    : name of a joint
        @type joint     : str

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

        @param joint    : name of a joint
        @type joint     : str

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

    def _active_cb(self):
        rospy.loginfo("GripperInterface: '{}' request active.".format(self._caller))

    def _feedback_cb(self, msg):
        rospy.loginfo("GripperInterface: '{}' request feedback: \n\t{}".format(self._caller,msg))

    def _done_cb(self, status, result):
        rospy.loginfo("GripperInterface: '{}' complete. Result: \n\t{}".format(self._caller, result))


    def home_joints(self, wait_for_result = False):
        """
        Performs homing of the gripper.
       
        After changing the gripper fingers, a homing needs to be done.
        This is needed to estimate the maximum grasping width.

        @param wait_for_result  : if True, this method will block till response is 
                                    recieved from server
        @type wait_for_result   : bool
       
        @return success
        @rtype bool      
        
        """
        self._caller = "home_joints"

        goal = HomingGoal()

        self._homing_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        if wait_for_result:
            result = self._homing_action_client.wait_for_result(rospy.Duration(15.))
            return result

        return True

    def move_joints(self, width, speed = 0.05, wait_for_result = True):
        """
        Moves the gripper fingers to a specified width.
       
        @param width            : Intended opening width. [m]
        @param speed            : Closing speed. [m/s]
        @param wait_for_result  : if True, this method will block till response is 
                                    recieved from server

        @type width             : float
        @type speed             : float
        @type wait_for_result   : bool
       
        @return True if command was successful, False otherwise.
        @rtype bool
        """
        self._caller = "move_joints"

        goal = MoveGoal()
        goal.width = width
        goal.speed = speed

        self._move_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        if wait_for_result:
            result = self._move_action_client.wait_for_result(rospy.Duration(15.))
            return result

        return True


    def stop_action(self):
        """
        Stops a currently running gripper move or grasp.
       
        @return True if command was successful, False otherwise.
        @rtype bool
        """
        self._caller = "stop_action"

        goal = StopGoal()

        self._stop_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        result = self._stop_action_client.wait_for_result(rospy.Duration(15.))
        return result

    def grasp(self, width, force, speed = 0.04, epsilon_inner = 0.005, epsilon_outer = 0.005,wait_for_result = True):
        """
        Grasps an object.
       
        An object is considered grasped if the distance $d$ between the gripper fingers satisfies
        $(\text{width} - \text{epsilon_inner}) < d < (\text{width} + \text{epsilon_outer})$.
       
        @param width        : Size of the object to grasp. [m]
        @param speed        : Closing speed. [m/s]
        @param force        : Grasping force. [N]
        @param epsilon_inner: Maximum tolerated deviation when the actual grasped width is smaller
                                than the commanded grasp width.
        @param epsilon_outer: Maximum tolerated deviation when the actual grasped width is wider
                                than the commanded grasp width.

        @type width         : float
        @type speed         : float
        @type force         : float
        @type epsilon_inner : float
        @type epsilon_outer : float

        @return True if an object has been grasped, false otherwise.
        """
        self._caller = "grasp_action"

        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner = epsilon_inner, outer = epsilon_outer)

        self._grasp_action_client.send_goal(goal, done_cb =self._done_cb, active_cb = self._active_cb, feedback_cb = self._feedback_cb)

        if wait_for_result:
            result = self._grasp_action_client.wait_for_result(rospy.Duration(15.))
            return result

        return True

