# franka_tools

Some helper classes for controlling and handling the Franka Emika Panda robot.

### controller_manager_interface

- List, start, stop, load available controllers for the robot
- Get the current controller status (commands, set points, controller gains, etc.)
- Update controller parameters through *controller_param_config_client* (see below)

### frames_interface

- Get and Set end-effector frame and stiffness frame of the robot easily
- Set the frames to known frames (such as links on the robot) directly
 
### controller_param_config_client

- Get and set the controller parameters (gains) for the active controller

### joint_trajectory_action_client

- Command robot to given joint position(s) smoothly. (Uses the FollowJointTrajectory service from ROS *control_msgs* package)
- Smoothly move to a desired (valid) pose without having to interpolate for smoothness (trajectory interpolation done internally)