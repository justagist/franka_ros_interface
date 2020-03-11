# /***************************************************************************

# 
# @package: franka_interface
# @metapackage: franka_ros_interface
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/

"""
 @info: 
   Inteface Class for Franka robot arm.

"""


import enum
import rospy
import warnings
import quaternion
import numpy as np
from copy import deepcopy
from rospy_message_converter import message_converter

from franka_core_msgs.msg import JointCommand, RobotState, EndPointState, ImpedanceStiffness
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Wrench

import franka_control
import franka_dataflow
import franka_interface
from robot_params import RobotParams

from franka_tools import FrankaFramesInterface, FrankaControllerManagerInterface, JointTrajectoryActionClient


class TipState():

    def __init__(self, timestamp, pose, vel, O_effort, K_effort):
        self.timestamp = timestamp
        self._pose = pose
        self._velocity = vel
        self._effort = O_effort
        self._effort_in_K_frame = K_effort

    @property
    def pose(self):
        return self._pose
    @property
    def velocity(self):
        return self._velocity
    @property
    def effort(self):
        return self._effort
    @property
    def effort_in_K_frame(self):
        return self._effort_in_K_frame

    
    
    

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

        """
        self.hand = franka_interface.GripperInterface()
        
        self._params = RobotParams()

        self._ns = self._params.get_base_namespace()

        self._joint_limits = self._params.get_joint_limits()

        joint_names = self._joint_limits.joint_names
        if not joint_names:
            rospy.logerr("Cannot detect joint names for arm on this "
                         "robot. Exiting Arm.init().")
                         
            return   

        self._joint_names = joint_names
        self.name = self._params.get_robot_name()
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

        self._command_msg = JointCommand()

        # neutral pose joint positions
        self._neutral_pose_joints = self._params.get_neutral_pose()

        self._frames_interface = FrankaFramesInterface()
        self._ctrl_manager = FrankaControllerManagerInterface(ns = self._ns, sim = True if self._params._in_sim else False)

        self._speed_ratio = 0.15

        queue_size = None if synchronous_pub else 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._joint_command_publisher = rospy.Publisher(
                self._ns +'/motion_controller/arm/joint_commands',
                JointCommand,
                tcp_nodelay=True,
                queue_size=queue_size)

        self._pub_joint_cmd_timeout = rospy.Publisher(
            self._ns +'/motion_controller/arm/joint_command_timeout',
            Float64,
            latch=True,
            queue_size=10)



        self._robot_state_subscriber = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/robot_state',
            RobotState,
            self._on_robot_state,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = self._ns + '/custom_franka_state_controller/joint_states'
        self._joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        self._cartesian_state_sub = rospy.Subscriber(
            self._ns + '/custom_franka_state_controller/tip_state',
            EndPointState,
            self._on_endpoint_state,
            queue_size=1,
            tcp_nodelay=True)

        # Cartesian Impedance Controller Publishers
        self._impedance_pose_publisher = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=10)
        self._cartesian_stiffness_publisher = rospy.Publisher("impedance_stiffness", ImpedanceStiffness, queue_size=10)

        # Force Control Publisher
        self._force_controller_publisher = rospy.Publisher("wrench_target", Wrench, queue_size=10)

        rospy.on_shutdown(self._clean_shutdown)

        err_msg = ("%s arm init failed to get current joint_states "
                   "from %s") % (self.name.capitalize(), joint_state_topic)
        franka_dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

        err_msg = ("%s arm, init failed to get current tip_state "
                   "from %s") % (self.name.capitalize(), self._ns + 'tip_state')
        franka_dataflow.wait_for(lambda: len(self._cartesian_pose.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

        err_msg = ("%s arm, init failed to get current robot_state "
                   "from %s") % (self.name.capitalize(), self._ns + 'robot_state')
        franka_dataflow.wait_for(lambda: self._jacobian is not None,
                                 timeout_msg=err_msg, timeout=5.0)

        self.set_joint_position_speed(self._speed_ratio)

    def convertToDict(self, q):
        q_dict = dict()
        for i in xrange(len(q)):
            q_dict['panda_joint{}'.format(i+1)] = q[i]
        return q_dict

    def convertToList(self, q_dict):
        q = []
        sorted_keys = sorted(q_dict.keys())
        for i in sorted_keys:
            q.append(q_dict[i])
        return q


    def _clean_shutdown(self):
        self._joint_state_sub.unregister()
        self._cartesian_state_sub.unregister()
        self._pub_joint_cmd_timeout.unregister()
        self._robot_state_subscriber.unregister()
        self._joint_command_publisher.unregister()
        self._impedance_pose_publisher.unregister()
        self._cartesian_stiffness_publisher.unregister()

    def get_robot_params(self):
        return self._params

    def get_joint_limits(self):
        return self._joint_limits

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
        if self._frames_interface:
            self._frames_interface._update_frame_data(msg.F_T_EE, msg.EE_T_K)

        self.q_d = msg.q_d
        self.dq_d = msg.dq_d

        self._gravity = np.asarray(msg.gravity)

        self._errors = message_converter.convert_ros_message_to_dictionary(msg.current_errors)


    def gravity_comp(self):
        return self._gravity

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

        self._tip_states = TipState(msg.header.stamp, deepcopy(self._cartesian_pose), deepcopy(self._cartesian_velocity), deepcopy(self._cartesian_effort), deepcopy(self._stiffness_frame_effort))



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
_ns
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

    def endpoint_velocity(self):
        """
        Return Cartesian endpoint twist {linear, angular}.

        @rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        @return: linear and angular velocities as named tuples in a dict

        C{twist = {'linear': (x, y, z), 'angular': (x, y, z)}}

          - 'linear': np.array of x, y, z
          - 'angular': np.array of x, y, z (angular velocity along the axes)
        """
        return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self):
        """
        Return Cartesian endpoint wrench {force, torque}.

        @rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        @return: force and torque at endpoint as named tuples in a dict

        C{wrench = {'force': (x, y, z), 'torque': (x, y, z)}}

          - 'force': Cartesian force on x,y,z axes in np.ndarray format
          - 'torque': Torque around x,y,z axes in np.ndarray format
        """
        return deepcopy(self._cartesian_effort)

    def tip_states(self):
        """
        Return Cartesian endpoint state for a given tip name

        @rtype: TipState object
        @return: pose, velocity, effort, effort_in_K_frame
        """
        return deepcopy(self._tip_states)

    def set_command_timeout(self, timeout):
        """
        Set the timeout in seconds for the joint controller

        @type timeout: float
        @param timeout: timeout in seconds
        """
        self._pub_joint_cmd_timeout.publish(Float64(timeout))


    def set_joint_position_speed(self, speed=0.3):
        """
        Set ratio of max joint speed to use during joint position moves (only for move_to_joint_positions).

        Set the proportion of maximum controllable velocity to use
        during joint position control execution. The default ratio
        is `0.3`, and can be set anywhere from [0.0-1.0] (clipped).
        Once set, a speed ratio will persist until a new execution
        speed is set.

        @type speed: float
        @param speed: ratio of maximum joint speed for execution
                      default= 0.3; range= [0.0-1.0]
        """
        if speed > 0.3:
            rospy.logwarn("ArmInterface: Setting speed above 0.3 could be risky!! Be extremely careful.")
        self._speed_ratio = speed

    def set_joint_positions(self, positions):
        """
        Commands the joints of this limb to the specified positions.

        @type positions: [float]
        @param positions: ordered joint angles (from joint1 to joint7) to be commanded
        """
        self._command_msg.names = self._joint_names
        self._command_msg.position = [positions[j] for j in self._joint_names]
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_velocities(self, velocities):
        """
        Commands the joints of this limb to the specified velocities.

        @type velocities: dict({str:float})
        @param velocities: joint_name:velocity command
        """
        self._command_msg.names = self._joint_names
        self._command_msg.velocity = [velocities[j] for j in self._joint_names]
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_torques(self, torques):
        """
        Commands the joints of this limb to the specified torques.

        @type torques: dict({str:float})
        @param torques: joint_name:torque command
        """
        self._command_msg.names = self._joint_names
        self._command_msg.effort = [torques[j] for j in self._joint_names]
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)

    def set_joint_positions_velocities(self, positions, velocities):
        """
        Commands the joints of this limb using specified positions and velocities using impedance control.
        Command at time t is computed as 
            u_t = coriolis_factor * coriolis_t + 
                  K_p * (positions - curr_positions) + 
                  K_d * (velocities - curr_velocities)

        @type positions: [float]
        @param positions: desired joint positions as an ordered list corresponding to joints given by self.joint_names()
        @type velocities: [float]
        @param velocities: desired joint velocities as an ordered list corresponding to joints given by self.joint_names()
        """
        self._command_msg.names = self._joint_names
        self._command_msg.position = positions
        self._command_msg.velocity = velocities
        self._command_msg.mode = JointCommand.IMPEDANCE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(self._command_msg)


    def has_collided(self):
        return any(self._joint_collision) or any(self._cartesian_collision)
        

    def switchToController(self, controller_name):
        active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)
        for ctrlr in active_controllers:
            self._ctrl_manager.stop_controller(ctrlr.name)
            rospy.loginfo("ArmInterface: Stopping %s for trajectory controlling"%ctrlr.name)
            rospy.sleep(0.5)

        if not self._ctrl_manager.is_loaded(controller_name):
            self._ctrl_manager.load_controller(controller_name)
        self._ctrl_manager.start_controller(controller_name)

    def move_to_neutral(self, timeout=15.0, speed=0.15):
        """
        Command the Limb joints to a predefined set of "neutral" joint angles.
        From rosparam /franka_control/neutral_pose.

        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type speed: float
        @param speed: ratio of maximum joint speed for execution
                      default= 0.15; range= [0.0-1.0]
        """
        self.set_joint_position_speed(speed)
        self.move_to_joint_positions(self._neutral_pose_joints, timeout) if not self._params._in_sim else self.set_joint_positions(self._neutral_pose_joints)

    def genf(self, joint, angle):
        def joint_diff():
            return abs(angle - self._joint_angle[joint])
        return joint_diff

    def move_to_joint_positions(self, positions, timeout=10.0,
                                threshold=0.00085, test=None):
        """
        (Blocking) Commands the limb to the provided positions.

        Waits until the reported joint state matches that specified.

        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful [0.008726646]
        @param test: optional function returning True if motion must be aborted
        """
        switch_ctrl = True if self._ctrl_manager.current_controller != self._ctrl_manager.joint_trajectory_controller else False
        if switch_ctrl:
            self.switchToController(self._ctrl_manager.joint_trajectory_controller)
        
        min_traj_dur = 0.5
        traj_client = JointTrajectoryActionClient(joint_names = self.joint_names(), ns = self._ns)
        traj_client.clear()

        dur = []
        for j in range(len(self._joint_names)):
            dur.append(max(abs(positions[self._joint_names[j]] - self._joint_angle[self._joint_names[j]]) / self._joint_limits.velocity[j], min_traj_dur))
        traj_client.add_point(positions = [positions[n] for n in self._joint_names], time = max(dur)/self._speed_ratio)

        diffs = [self.genf(j, a) for j, a in positions.items() if j in self._joint_angle]

  
        traj_client.start() # send the trajectory action request
        fail_msg = "ArmInterface: {0} limb failed to reach commanded joint positions.".format(
                                                      self.name.capitalize())

        def test_collision():
            if self.has_collided():
                rospy.logerr(' '.join(["Collision detected.", fail_msg]))
                return True
            return False

        franka_dataflow.wait_for(
            test=lambda: test_collision() or \
                         (callable(test) and test() == True) or \
                         (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg=fail_msg,
            rate=100,
            raise_on_error=False
            )

        rospy.sleep(0.5)

        if switch_ctrl:
            self._ctrl_manager.stop_controller(self._ctrl_manager.joint_trajectory_controller)
            for ctrlr in self._ctrl_manager.list_active_controllers(only_motion_controllers = True):
                self._ctrl_manager.start_controller(ctrlr.name)
                rospy.loginfo("ArmInterface: Restaring %s"%ctrlr.name)
                rospy.sleep(0.5)

        rospy.loginfo("ArmInterface: Trajectory controlling complete")

    def execute_position_path(self, position_path, timeout=10.0,
                                threshold=0.00085, test=None):
        """
        (Blocking) Commands the limb to the provided positions.
        Waits until the reported joint state matches that specified.
        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful [0.008726646]
        @param test: optional function returning True if motion must be aborted
        """

        switch_ctrl = True if self._ctrl_manager.current_controller != self._ctrl_manager.joint_trajectory_controller else False

        if switch_ctrl:
            self.switchToController(self._ctrl_manager.joint_trajectory_controller)
        
        min_traj_dur = 0.5
        traj_client = JointTrajectoryActionClient(joint_names = self.joint_names(), ns = self._ns)
        traj_client.clear()

        time_so_far = 0
        for q in position_path: 
            dur = []
            for j in range(len(self._joint_names)):
                dur.append(max(abs(q[self._joint_names[j]] - self._joint_angle[self._joint_names[j]]) / self._joint_limits.velocity[j], min_traj_dur))

            time_so_far += max(dur)/self._speed_ratio
            traj_client.add_point(positions = [q[n] for n in self._joint_names], time = time_so_far, velocities=[0.005 for n in self._joint_names])

        diffs = [self.genf(j, a) for j, a in (position_path[-1]).items() if j in self._joint_angle] # Measures diff to last waypoint

        fail_msg = "ArmInterface: {0} limb failed to reach commanded joint positions.".format(
                                                      self.name.capitalize())
        def test_collision():
            if self.has_collided():
                rospy.logerr(' '.join(["Collision detected.", fail_msg]))
                return True
            return False

        traj_client.start() # send the trajectory action request

        franka_dataflow.wait_for(
            test=lambda: test_collision() or \
                         (callable(test) and test() == True) or \
                         (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg=fail_msg,
            rate=100,
            raise_on_error=False
            )

        rospy.sleep(0.5)

        if switch_ctrl:
            self._ctrl_manager.stop_controller(self._ctrl_manager.joint_trajectory_controller)
            for ctrlr in self._ctrl_manager.list_active_controllers(only_motion_controllers = True):
                self._ctrl_manager.start_controller(ctrlr.name)
                rospy.loginfo("ArmInterface: Restaring %s"%ctrlr.name)
                rospy.sleep(0.5)

        rospy.loginfo("ArmInterface: Trajectory controlling complete")

    def move_to_touch(self, positions, timeout=10.0, threshold=0.00085):
        """
        (Blocking) Commands the limb to the provided positions.

        Waits until the reported joint state matches that specified.

        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful [0.008726646]
        @param test: optional function returning True if motion must be aborted
        """
        switch_ctrl = True if self._ctrl_manager.current_controller != self._ctrl_manager.joint_trajectory_controller else False
        if switch_ctrl:
            self.switchToController(self._ctrl_manager.joint_trajectory_controller)
        
        min_traj_dur = 0.5
        traj_client = JointTrajectoryActionClient(joint_names = self.joint_names(), ns = self._ns)
        traj_client.clear()

        speed_ratio = 0.05 # Move slower when approaching contact
        dur = []
        for j in range(len(self._joint_names)):
            dur.append(max(abs(positions[self._joint_names[j]] - self._joint_angle[self._joint_names[j]]) / self._joint_limits.velocity[j], min_traj_dur))
        traj_client.add_point(positions = [positions[n] for n in self._joint_names], time = max(dur)/speed_ratio, velocities=[0.002 for n in self._joint_names])

        diffs = [self.genf(j, a) for j, a in positions.items() if j in self._joint_angle]
        fail_msg = "ArmInterface: {0} limb failed to reach commanded joint positions.".format(
                                                      self.name.capitalize()) 
 
        traj_client.start() # send the trajectory action request

        franka_dataflow.wait_for(
            test=lambda: self.has_collided() or \
                         (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg="Collision Detected!",
            rate=100,
            raise_on_error=False
            )

        rospy.sleep(0.5)

        if not self.has_collided():
            rospy.logerr('Move To Touch did not end in making contact') 

        if switch_ctrl:
            self._ctrl_manager.stop_controller(self._ctrl_manager.joint_trajectory_controller)
            for ctrlr in self._ctrl_manager.list_active_controllers(only_motion_controllers = True):
                self._ctrl_manager.start_controller(ctrlr.name)
                rospy.loginfo("ArmInterface: Restaring %s"%ctrlr.name)
                rospy.sleep(0.5)

        # The collision, though desirable, triggers a cartesian reflex error. We need to reset that error
        if self._robot_mode == 4:
            self.resetErrors()

        rospy.loginfo("ArmInterface: Trajectory controlling complete")

    def resetErrors(self):
        rospy.sleep(0.5)
        pub = rospy.Publisher('/franka_ros_interface/franka_control/error_recovery/goal', franka_control.msg.ErrorRecoveryActionGoal, queue_size=10)
        rospy.sleep(0.5)
        pub.publish(franka_control.msg.ErrorRecoveryActionGoal())
        rospy.loginfo("Collision Reflex was reset")



    def move_from_touch(self, positions, timeout=10.0, threshold=0.00085):
        """
        (Blocking) Commands the limb to the provided positions.

        Waits until the reported joint state matches that specified.

        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful [0.008726646]
        @param test: optional function returning True if motion must be aborted
        """

        switch_ctrl = True if self._ctrl_manager.current_controller != self._ctrl_manager.joint_trajectory_controller else False
        if switch_ctrl:
            self.switchToController(self._ctrl_manager.joint_trajectory_controller)
        
        min_traj_dur = 0.5
        traj_client = JointTrajectoryActionClient(joint_names = self.joint_names(), ns = self._ns)
        traj_client.clear()

        dur = []
        for j in range(len(self._joint_names)):
            dur.append(max(abs(positions[self._joint_names[j]] - self._joint_angle[self._joint_names[j]]) / self._joint_limits.velocity[j], min_traj_dur))
        traj_client.add_point(positions = [positions[n] for n in self._joint_names], time = max(dur)/self._speed_ratio)

        diffs = [self.genf(j, a) for j, a in positions.items() if j in self._joint_angle]
        fail_msg = "ArmInterface: {0} limb failed to reach commanded joint positions.".format(
                                                      self.name.capitalize()) 
 
        traj_client.start() # send the trajectory action request

        franka_dataflow.wait_for(
            test=lambda: (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg="Unable to complete plan!",
            rate=100,
            raise_on_error=False
            )

        rospy.sleep(0.5)

        if switch_ctrl:
            self._ctrl_manager.stop_controller(self._ctrl_manager.joint_trajectory_controller)
            for ctrlr in active_controllers:
                self._ctrl_manager.start_controller(ctrlr.name)
                rospy.loginfo("ArmInterface: Restaring %s"%ctrlr.name)
                rospy.sleep(0.5)

        rospy.loginfo("ArmInterface: Trajectory controlling complete")

    def set_cart_impedance_pose(self, pose, stiffness=None):
        switch_ctrl = True if self._ctrl_manager.current_controller != self._ctrl_manager.cartesian_impedance_controller else False
        if switch_ctrl:
            self.switchToController(self._ctrl_manager.cartesian_impedance_controller)

        if stiffness is not None:
            stiffness_gains = ImpedanceStiffness()
            stiffness_gains.x = stiffness[0]
            stiffness_gains.y = stiffness[1]
            stiffness_gains.z = stiffness[2]
            stiffness_gains.xrot = stiffness[3]
            stiffness_gains.yrot = stiffness[4]
            stiffness_gains.zrot = stiffness[5]
            self._cartesian_stiffness_publisher.publish(stiffness_gains)

        marker_pose = PoseStamped()
        marker_pose.pose.position.x = pose['position'][0]
        marker_pose.pose.position.y = pose['position'][1]
        marker_pose.pose.position.z = pose['position'][2]
        marker_pose.pose.orientation.x = pose['orientation'].x
        marker_pose.pose.orientation.y = pose['orientation'].y
        marker_pose.pose.orientation.z = pose['orientation'].z
        marker_pose.pose.orientation.w = pose['orientation'].w
        self._impedance_pose_publisher.publish(marker_pose)

    def execute_cart_impedance_traj(self, poses, stiffness=None, timing=None):
        if timing is None:
            timing = 0.5

        for i in xrange(len(poses)):
            self.set_cart_impedance_pose(poses[i], stiffness)
            rospy.sleep(timing)

    def exert_force(self, target_wrench):
        switch_ctrl = True if self._ctrl_manager.current_controller != self._ctrl_manager.force_controller else False
        if switch_ctrl:
            self.switchToController(self._ctrl_manager.force_controller)

        wrench = Wrench()
        wrench.force.x = target_wrench[0]
        wrench.force.y = target_wrench[1]
        wrench.force.z = target_wrench[2]
        wrench.torque.x = target_wrench[3]
        wrench.torque.y = target_wrench[4] 
        wrench.torque.z = target_wrench[5]
        self._force_controller_publisher.publish(wrench)


    def reset_EE_frame(self):
        """
        Reset EE frame to default. (defined by FrankaFramesInterface.DEFAULT_TRANSFORMATIONS.EE_FRAME global variable defined above) 

        @rtype: [bool, str]
        @return: [success status of service request, error msg if any]
        """
        if self._frames_interface:

            if self._frames_interface.EE_frame_is_reset():
                rospy.loginfo("ArmInterface: EE Frame already reset")
                return

            active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)

            rospy.loginfo("ArmInterface: Stopping motion controllers for resetting EE frame")
            for ctrlr in active_controllers:
                self._ctrl_manager.stop_controller(ctrlr.name)
            rospy.sleep(1.)

            retval = self._frames_interface.reset_EE_frame()

            rospy.sleep(1.)
            rospy.loginfo("ArmInterface: Restarting previously active motion controllers.")
            for ctrlr in active_controllers:
                self._ctrl_manager.start_controller(ctrlr.name)
            rospy.sleep(1.)

            return retval

        else:
            rospy.logwarn("ArmInterface: Frames changing not available in simulated environment")
            return False


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
        if self._frames_interface:

            frame = self._frames_interface._assert_frame_validity(frame)

            active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)
            rospy.sleep(1.)
            rospy.loginfo("ArmInterface: Stopping motion controllers for changing EE frame")
            for ctrlr in active_controllers:
                self._ctrl_manager.stop_controller(ctrlr.name)
            rospy.sleep(1.)

            retval = self._frames_interface.set_EE_frame(frame)

            rospy.sleep(1.)
            rospy.loginfo("ArmInterface: Restarting previously active motion controllers.")
            for ctrlr in active_controllers:
                self._ctrl_manager.start_controller(ctrlr.name)
            rospy.sleep(1.)

            return retval

        else:
            rospy.logwarn("ArmInterface: Frames changing not available in simulated environment")

    def set_EE_frame_to_link(self, frame_name, timeout = 5.0):
        """
        Set new EE frame to the same frame as the link frame given by 'frame_name'
        Motion controllers are stopped for switching

        @type frame_name: str 
        @param frame_name: desired tf frame name in the tf tree
        @rtype: [bool, str]
        @return: [success status of service request, error msg if any]
        """
        if self._frames_interface:
            retval = True
            if not self._frames_interface.EE_frame_already_set(self._frames_interface.get_link_tf(frame_name)):
                active_controllers = self._ctrl_manager.list_active_controllers(only_motion_controllers = True)

                rospy.loginfo("ArmInterface: Stopping motion controllers for changing EE frame")
                rospy.sleep(1.)
                for ctrlr in active_controllers:
                    self._ctrl_manager.stop_controller(ctrlr.name)
                rospy.sleep(1.)

                retval = self._frames_interface.set_EE_frame_to_link(frame_name = frame_name, timeout = timeout)

                rospy.sleep(1.)
                rospy.loginfo("ArmInterface: Restarting previously active motion controllers.")
                for ctrlr in active_controllers:
                    self._ctrl_manager.start_controller(ctrlr.name)
                rospy.sleep(1.)

            return retval
        else:
            rospy.logwarn("ArmInterface: Frames changing not available in simulated environment")

    def get_controller_manager(self):
        return self._ctrl_manager

    def get_frames_interface(self):
        return self._frames_interface


if __name__ == '__main__':
    rospy.init_node('test')
    r = Arm()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
