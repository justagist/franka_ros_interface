# franka_interface

**See induvidual source files for more detailed documentation.**

### ArmInterface
- Interface class that can monitor and control the robot 
- Provides all required information about robot state and end-effector state
- Joint positions, velocities, and effort can be directly controlled and monitored using available methods
- Smooth interpolation of joint positions possible
- End-effector and Stiffness frames can be directly set (uses FrankaFramesInterface from *franka_ros_interface/franka_tools*)

### GripperInterface
- Interface class to monitor and control gripper
- Gripper open, close methods
- Grasp, move joints methods 

### RobotEnable
- Interface class to reset robot when in recoverable error (use *enable_robot.py* script in *scripts/*)

### RobotParams
- Collects and stores all useful information about the robot from the ROS parameter server

### Scripts

- Scripts providing simple gripper actions, and robot reset are provided
