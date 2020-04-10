Python API Documentation
========================

|release| |Code Quality|

franka_interface
----------------
.. automodule:: franka_interface
   :members:
   :show-inheritance:

ArmInterface
^^^^^^^^^^^^

- Interface class that can monitor and control the robot 
- Provides all required information about robot state and end-effector state
- Joint positions, velocities, and effort can be directly controlled and monitored using available methods
- Smooth interpolation of joint positions possible
- End-effector and Stiffness frames can be directly set (uses FrankaFramesInterface from *franka_ros_interface/franka_tools*)

.. autoclass:: ArmInterface
   :members: 
   :show-inheritance:

GripperInterface
^^^^^^^^^^^^^^^^

- Interface class to monitor and control gripper
- Gripper open, close methods
- Grasp, move joints methods 

.. autoclass:: GripperInterface
   :members:
   :show-inheritance: 

RobotEnable
^^^^^^^^^^^

- Interface class to reset robot when in recoverable error (use *enable_robot.py* script in *scripts/*)

.. autoclass:: RobotEnable
   :members:
   :show-inheritance: 

RobotParams
^^^^^^^^^^^

- Collects and stores all useful information about the robot from the ROS parameter server

.. autoclass:: RobotParams
   :members:
   :show-inheritance: 

franka_moveit
-------------
.. automodule:: franka_moveit
   :members:
   :show-inheritance:

PandaMoveGroupInterface
^^^^^^^^^^^^^^^^^^^^^^^

- Provides interface to control and plan motions using MoveIt in ROS.
- Simple methods to plan and execute joint trajectories and cartesian path.
- Provides easy reset and environment definition functionalities (See ExtendedPlanningSceneInterface below).

.. autoclass:: PandaMoveGroupInterface
   :members:
   :show-inheritance: 

Helper Functions
~~~~~~~~~~~~~~~~
.. automodule:: franka_moveit.utils
   :members: create_pose_msg, create_pose_stamped_msg

ExtendedPlanningSceneInterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Easily define scene for robot motion planning (MoveIt plans will avoid defined obstacles if possible).  

.. autoclass:: franka_moveit.ExtendedPlanningSceneInterface
   :members:
   :show-inheritance:

franka_tools
------------
.. automodule:: franka_tools
   :members:
   :show-inheritance:

CollisionBehaviourInterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Define collision and contact thresholds for the robot safety and contact detection.

.. autoclass:: CollisionBehaviourInterface
   :members:
   :show-inheritance:

FrankaControllerManagerInterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- List, start, stop, load available controllers for the robot
- Get the current controller status (commands, set points, controller gains, etc.)
- Update controller parameters through *ControllerParamConfigClient* (see below)

.. autoclass:: FrankaControllerManagerInterface
   :members:
   :show-inheritance:

ControllerParamConfigClient
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Get and set the controller parameters (gains) for the active controller

.. autoclass:: ControllerParamConfigClient
   :members:
   :show-inheritance: 

FrankaFramesInterface
^^^^^^^^^^^^^^^^^^^^^

- Get and Set end-effector frame and stiffness frame of the robot easily
- Set the frames to known frames (such as links on the robot) directly

.. autoclass:: FrankaFramesInterface
   :members:
   :show-inheritance: 

JointTrajectoryActionClient
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Command robot to given joint position(s) smoothly. (Uses the FollowJointTrajectory service from ROS *control_msgs* package)
- Smoothly move to a desired (valid) pose without having to interpolate for smoothness (trajectory interpolation done internally)

.. autoclass:: JointTrajectoryActionClient
   :members:
   :show-inheritance: 

.. |Code Quality| image:: https://api.codacy.com/project/badge/Grade/ec16a09639d341358b73cb8cdaa57d2e    
   :target: https://www.codacy.com/manual/justagist/franka_ros_interface?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=justagist/franka_ros_interface&amp;utm_campaign=Badge_Grade
.. |release| image:: https://img.shields.io/github/v/release/justagist/franka_ros_interface?include_prereleases   :alt: GitHub release (latest by date including pre-releases)