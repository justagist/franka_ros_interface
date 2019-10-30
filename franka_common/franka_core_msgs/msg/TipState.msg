std_msgs/Header header

float64[16] O_T_EE # Measured end effector pose in base frame
float64[16] O_dP_EE_c # Last commanded end effector twist in base frame.
float64[16] O_dP_EE_d # Desired end effector twist in base frame.
float64[16] O_ddP_EE_c # Last commanded end effector acceleration in base frame.

geometry_msgs/WrenchStamped O_F_ext_hat_K    # Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame
geometry_msgs/WrenchStamped K_F_ext_hat_K    # Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame



