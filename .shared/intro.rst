(*Version 1.0.0*)

A ROS API for the Franka Emika Panda robot, extending the `franka_ros`_ to expose more information about the robot, and
additionally providing low level control of the robot using ROS and `Python API <Python API Documentation_>`_.

Provides controlling and managing the Franka Emika Panda robot (real and
simulated). Contains exposed controllers for the robot (joint position,
velocity, torque), interfaces for the gripper, controller manager,
coordinate frames, etc, MoveIt! and Trajectory Action Client interfaces. 
Provides almost complete sim-to-real /
real-to-sim transfer of code with the `panda_simulator`_
package.

Documentation Page: https://justagist.github.io/franka_ros_interface

Features
--------

-  Low-level *controllers* (joint position, velocity, torque, impedance)
   available that can be controlled through ROS topics (including
   position control for gripper).
-  Real-time *robot state* (end-effector state, joint state, controller
   state, etc.) available through ROS topics.
-  Python API to monitor and control the robot using any of the
   available controllers.
-  Python API for managing controllers, coordinate frames, controlling
   and monitoring the gripper.
-  The `panda_simulator`_ package (which is Gazebo-based
   simulator for the robot) can also be controlled using this package
   (ROS and Python interface), providing almost complete sim-to-real
   transfer of code.

Go to `Project Source Code`_.

.. include:: ../.shared/links.rst