Franka ROS Interface
====================

|release| |doi|


A ROS interface library for the Franka Emika Panda robot, extending the `franka_ros`_ to expose more information about the robot, and
providing low-level control of the robot using ROS and `Python API <Python API Documentation_>`_.

.. image:: https://raw.githubusercontent.com/justagist/franka_ros_interface/master/assets/panda_robot_demo.gif
   :target: https://youtu.be/4bEVysUIvOY

Provides utilites for controlling and managing the Franka Emika Panda robot (real and
`simulated <panda_simulator_>`_). Contains exposed controllers for the robot (joint position,
velocity, torque), interfaces for the gripper, controller manager,
coordinate frames interface, etc. Also provides utilities to control the robot using 
MoveIt! and ROS Trajectory Action & ActionClient. 
This package also provides almost complete sim-to-real /
real-to-sim transfer of code with the `panda_simulator`_
package.

A more unified ROS Python interface built over this package is available in the `panda_robot`_ package, which provides a more intuitive interface class that combines the different API classes in this package. Simple demo and `basic usage <https://justagist.github.io/panda_robot/#basic-usage>`_ are also shown.

.. image:: https://raw.githubusercontent.com/justagist/franka_ros_interface/master/assets/panda_simulator.gif
   :target: https://www.youtube.com/watch?v=NdSbXC0r7tU

**Features**

-  Low-level *controllers* (joint position, velocity, torque, impedance)
   available that can be controlled through ROS topics and Python API (including
   position control for gripper).
-  Real-time *robot state* (end-effector state, joint state, controller
   state, etc.) available through ROS topics and Python API.
-  Python API for managing controllers, coordinate frames, collision behaviour, 
   controlling and monitoring the gripper.
-  Python API classes and utility functions to control the robot using 
   MoveIt! and ROS Trajectory Action Service. 
-  The `panda_simulator`_ package (which is Gazebo-based
   simulator for the robot) can also be controlled using this package
   (ROS and Python interface), providing almost complete sim-to-real
   transfer of code.

.. image:: https://raw.githubusercontent.com/justagist/franka_ros_interface/master/assets/ts_demo.gif
   :target: https://youtu.be/a_HEmYzqEnk

Go to `Project Source Code`_.

.. toctree::
   :maxdepth: 2
   :caption: Contents:
   :numbered:

   Setup Instructions<instructions>
   DOC

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Go to `Project Source Code`_.

**LICENSE:**

|License|
Copyright (c) 2019-2020, Saif Sidhik

|doi|

.. _panda_simulator: https://github.com/justagist/panda_simulator
.. _panda_robot: https://github.com/justagist/panda_robot
.. _KDL library: http://wiki.ros.org/kdl
.. _franka_panda_description: https://github.com/justagist/franka_panda_description
.. _franka_ros: https://frankaemika.github.io/docs/franka_ros.html
.. _this paper: https://hal.inria.fr/hal-02265293/document

.. _Python Documentation: https://justagist.github.io/franka_ros_interface

.. _FCI documentation: https://frankaemika.github.io/docs/installation_linux.html
.. _franka_panda_description: https://github.com/justagist/franka_panda_description
.. _Related Packages: #related-packages
.. _Environments: #the-frankash-environments
.. _install from source: https://frankaemika.github.io/docs/installation_linux.html#building-from-source

.. _Python API Documentation: https://justagist.github.io/franka_ros_interface/DOC.html
.. _Project Source Code: https://github.com/justagist/franka_ros_interface

.. |Build Status| image:: https://travis-ci.org/justagist/franka_ros_interface.svg?branch=master
   :target: https://travis-ci.org/justagist/franka_ros_interface
.. |License| image:: https://img.shields.io/badge/License-Apache2.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
.. |Code Quality| image:: https://api.codacy.com/project/badge/Grade/ec16a09639d341358b73cb8cdaa57d2e    
   :target: https://www.codacy.com/manual/justagist/franka_ros_interface?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=justagist/franka_ros_interface&amp;utm_campaign=Badge_Grade
.. |doi| image:: https://zenodo.org/badge/199485892.svg
   :target: https://zenodo.org/badge/latestdoi/199485892
.. |release| image:: https://img.shields.io/github/v/release/justagist/franka_ros_interface?include_prereleases   :alt: GitHub release (latest by date including pre-releases)

