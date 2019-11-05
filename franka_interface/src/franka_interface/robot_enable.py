# @package: franka_interface
# @metapackage: franka_ros_interface 
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
#
# @info: 
#       Wrapper class for controlling and monitoring robot state.
#
#   Todo: fix reset request to use service call instead of publishing on topic, set up stopping, redefine status class.
#


import rospy
from threading import Lock

from franka_control.msg import ErrorRecoveryActionGoal
from franka_core_msgs.msg import RobotState

import franka_dataflow

class RobotEnable(object):
    """
    Class RobotEnable - simple control/status wrapper around robot state

    enable()  - enable all joints
    disable() - disable all joints
    reset()   - reset all joints, reset all jrcp faults, disable the robot
    stop()    - stop the robot, similar to hitting the e-stop button
    """

    param_lock = Lock()

    def __init__(self, versioned=False):
        """
        
        """
        self._enabled = None
        state_topic = 'franka_ros_interface/custom_franka_state_controller/franka_state'
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

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('franka_ros_interface/franka_control/error_recovery/goal', ErrorRecoveryActionGoal,
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
#         error_not_stopped = """\
# Robot is not in a Error State. Cannot perform Reset.
# """
#         error_estop = """\
# E-Stop is ASSERTED. Disengage E-Stop and then reset the robot.
# """
#         error_nonfatal = """Non-fatal Robot Error on reset.
# Robot reset cleared stopped state and robot can be enabled, but a non-fatal
# error persists. Check diagnostics or rethink.log for more info.
# """
#         error_env = """Failed to reset robot.
# Please verify that the ROS_IP or ROS_HOSTNAME environment variables are set
# and resolvable. For more information please visit:
# http://sdk.rethinkrobotics.com/intera/SDK_Shell
# """


#         is_reset = lambda: (self._enabled.stopped == False and
#                             self._enabled.error == False and
#                             self._enabled.estop_button == 0 and
#                             self._enabled.estop_source == 0)
#         pub = rospy.Publisher('robot/set_super_reset', Empty, queue_size=10)

#         if (not self._enabled.stopped):
#             rospy.logfatal(error_not_stopped)
#             raise IOError(errno.EREMOTEIO, "Failed to Reset due to lack of Error State.")

#         if (self._enabled.stopped and
#               self._enabled.estop_button == AssemblyState.ESTOP_BUTTON_PRESSED):
#             rospy.logfatal(error_estop)
#             raise IOError(errno.EREMOTEIO, "Failed to Reset: E-Stop Engaged")

#         rospy.loginfo("Resetting robot...")
#         try:
#             franka_dataflow.wait_for(
#                 test=is_reset,
#                 timeout=5.0,
#                 timeout_msg=error_env,
#                 body=pub.publish
#             )
#         except OSError, e:
#             if e.errno == errno.ETIMEDOUT:
#                 if self._enabled.error == True and self._enabled.stopped == False:
#                     rospy.logwarn(error_nonfatal)
#                     return False
#             raise

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