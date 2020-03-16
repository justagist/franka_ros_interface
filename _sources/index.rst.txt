.. franka_ros_interface documentation master file, created by
   sphinx-quickstart on Fri Mar 13 18:33:23 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to franka_ros_interface's documentation!
================================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:


franka_interface
================
.. automodule:: franka_interface
   :members:

ArmInterface
------------
.. automodule:: franka_interface
   :members:
   
- Interface class that can monitor and control the robot 
- Provides all required information about robot state and end-effector state
- Joint positions, velocities, and effort can be directly controlled and monitored using available methods
- Smooth interpolation of joint positions possible
- End-effector and Stiffness frames can be directly set (uses FrankaFramesInterface from *franka_ros_interface/franka_tools*)


.. autoclass:: ArmInterface
   :members: 

GripperInterface
----------------
.. automodule:: franka_interface
   :members:

- Interface class to monitor and control gripper
- Gripper open, close methods
- Grasp, move joints methods 

.. autoclass:: GripperInterface
   :members: 

RobotEnable
-----------
.. automodule:: franka_interface
   :members:

- Interface class to reset robot when in recoverable error (use *enable_robot.py* script in *scripts/*)

.. autoclass:: RobotEnable
   :members: 

RobotParams
-----------
.. automodule:: franka_interface
   :members:

- Collects and stores all useful information about the robot from the ROS parameter server

.. autoclass:: RobotParams
   :members: 

franka_moveit
=============
.. automodule:: franka_moveit
   :members:

PandaMoveGroupInterface
-----------------------
.. automodule:: franka_moveit
   :members:

- Provides interface to control and plan motions using MoveIt in ROS.
- Simple methods to plan and execute joint trajectories and cartesian path.
- Provides easy reset and environment definition functionalities (See ExtendedPlanningSceneInterface below).

.. autoclass:: PandaMoveGroupInterface
   :members: 

ExtendedPlanningSceneInterface
------------------------------
.. automodule:: franka_moveit
   :members:

- Easily define scene for robot motion planning (MoveIt plans will avoid defined obstacles if possible).  

.. autoclass:: ExtendedPlanningSceneInterface
   :members: 

franka_tools
============
.. automodule:: franka_tools
   :members:

CollisionBehaviourInterface
---------------------------
.. automodule:: franka_tools
   :members:

- Define collision and contact thresholds for the robot safety and contact detection.

.. autoclass:: CollisionBehaviourInterface
   :members: 

FrankaControllerManagerInterface
--------------------------------
.. automodule:: franka_tools
   :members:

- List, start, stop, load available controllers for the robot
- Get the current controller status (commands, set points, controller gains, etc.)
- Update controller parameters through *ControllerParamConfigClient* (see below)

.. autoclass:: FrankaControllerManagerInterface
   :members: 

ControllerParamConfigClient
---------------------------
.. automodule:: franka_tools
   :members:

- Get and set the controller parameters (gains) for the active controller

.. autoclass:: ControllerParamConfigClient
   :members: 

FrankaFramesInterface
---------------------
.. automodule:: franka_tools
   :members:

- Get and Set end-effector frame and stiffness frame of the robot easily
- Set the frames to known frames (such as links on the robot) directly

.. autoclass:: FrankaFramesInterface
   :members: 

JointTrajectoryActionClient
---------------------------
.. automodule:: franka_tools
   :members:

- Command robot to given joint position(s) smoothly. (Uses the FollowJointTrajectory service from ROS *control_msgs* package)
- Smoothly move to a desired (valid) pose without having to interpolate for smoothness (trajectory interpolation done internally)

.. autoclass:: JointTrajectoryActionClient
   :members: 


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
