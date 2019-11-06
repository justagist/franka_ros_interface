# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[]  name
float64[] commanded_position
float64[] commanded_velocity
float64[] commanded_acceleration
float64[] commanded_effort
float64[] actual_position
float64[] actual_velocity
float64[] actual_effort
# This includes the inertial feed forward torques when applicable.
float64[] gravity_model_effort
# This is the torque required to hold the arm against gravity returned by KDL
# if the arm was stationary.  This does not include inertial feed forward
# torques (even when we have them) or any of the corrections (i.e. spring
# hysteresis, crosstalk, etc) we make to the KDL model.
float64[] gravity_only
# This is the torque produced by the interactionController plugin. When interaction
# control is on, this will be added to torque_bias in VelocityController7DOF.
float64[] interaction_torque
float64[] hysteresis_model_effort
float64[] crosstalk_model_effort
float64   hystState