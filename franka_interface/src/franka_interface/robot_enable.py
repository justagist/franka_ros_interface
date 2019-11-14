# @package: franka_interface
# @metapackage: franka_ros_interface 
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#
# @info: 
#       Wrapper class for enabling and resetting robot state.
#
#   Todo: fix reset request to use service call instead of publishing on topic, set up stopping, redefine status class.
#


import rospy
from threading import Lock

from franka_control.msg import ErrorRecoveryActionGoal
from franka_core_msgs.msg import RobotState

import franka_dataflow
from robot_params import RobotParams

class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, reset all jrcp faults, disable the robot
    stop()    - stop the robot, similar to hitting the e-stop button
    """

    param_lock = Lock()

    def __init__(self, robot_params = None):
        """
        
        """

        if robot_params:
            self._params = robot_params
        else:
            self._params = RobotParams()

        self._ns = self._params.get_base_namespace()

        self._enabled = None

        state_topic = '{}/custom_franka_state_controller/robot_state'.format(self._ns)
        self._state_sub = rospy.Subscriber(state_topic,
                                           RobotState,
                                           self._state_callback
                                           )

        franka_dataflow.wait_for(
            lambda: not self._enabled is None,
            timeout=5.0,
            timeout_msg=("Failed to get robot state on %s" %
            (state_topic,)),
        )

    def _state_callback(self, msg):
        self._enabled = True if msg.robot_mode != 4 else False

    def is_enabled(self):
        return self._enabled
    

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('{}/franka_control/error_recovery/goal'.format(self._ns), ErrorRecoveryActionGoal,
                              queue_size=10)

        if self._enabled == status:
            rospy.loginfo("Robot is already %s"%self.state())

        franka_dataflow.wait_for(
            test=lambda: self._enabled == status,
            timeout=5.0,
            timeout_msg=("Failed to %sable robot" %
                         ('en' if status else 'dis',)),
            body=lambda: pub.publish(ErrorRecoveryActionGoal()),
        )
        rospy.loginfo("Robot %s", ('Enabled' if status else 'Disabled'))

    def state(self):
        """
        Returns the last known robot state.

        @rtype: intera_core_msgs/AssemblyState
        @return: Returns the last received AssemblyState message
        """
        return "%sabled"%('en' if self._enabled else 'dis',)

    def enable(self):
        """
        Enable all joints
        """
        if not self._enabled:
            rospy.loginfo("Robot Stopped: Attempting Reset...")
            self.reset()
        self._toggle_enabled(True)

    def disable(self):
        """
        Disable all joints
        """
        self._toggle_enabled(False)

    def reset(self):
        """
        Reset all joints.  Trigger JRCP hardware to reset all faults.  Disable
        the robot.
        """
        pass

    def stop(self):
        """
        Simulate an e-stop button being pressed.  Robot must be reset to clear
        the stopped state.
        """
        # pub = rospy.Publisher('robot/set_super_stop', Empty, queue_size=10)
        # franka_dataflow.wait_for(
        #     test=lambda: self._enabled.stopped == True,
        #     timeout=5.0,
        #     timeout_msg="Failed to stop the robot",
        #     body=pub.publish,
        # )

        pass