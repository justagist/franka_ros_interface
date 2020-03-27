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

.. _panda_simulator: https://github.com/justagist/panda_simulator
.. _Python API Documentation: https://justagist.github.io/franka_ros_interface/DOC.html
.. _franka_ros: https://frankaemika.github.io/docs/franka_ros.html
