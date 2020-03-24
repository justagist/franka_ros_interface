Franka ROS Interface |Build Status| |Code Quality|
==================================================

|License|

A ROS API for the Franka Emika Panda robot, extending the `franka_ros`_ to expose more information about the robot, and
providing low level control of the robot using ROS and Python API.

Provides controlling and managing the Franka Emika Panda robot (real and
simulated). Contains exposed controllers for the robot (joint position,
velocity, torque), interfaces for the gripper, controller manager,
coordinate frames, etc. Provides almost complete sim-to-real /
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

Continuous Integration Builds
-----------------------------

ROS Kinetic / Melodic: |Build Status|


Installation
------------

Dependencies
^^^^^^^^^^^^

-  *libfranka* (``sudo apt install ros-$ROS_DISTRO-libfranka`` or
   `install from source`_).
-  *franka-ros* (``sudo apt install ros-$ROS_DISTRO-franka-ros`` or
   `install from source`_)
-  (optional, but recommended) `franka_panda_description`_ (See `Related
   Packages`_ section for information about package). **NOTE**: If you
   do not want to use the *franka_panda_description* package, make sure
   you modify the ``franka_interface/launch/interface.launch`` file and
   replace all occurences of ``franka_panda_description`` with
   ``franka_description`` (two occurences).

Once the above dependencies are installed, the package can be installed
using catkin:

::

   $ cd <catkin_ws>
   $ git clone https://github.com/justagist/franka_ros_interface src/franka_ros_interface
   $ catkin build franka_ros_interface # or catkin_make
   $ source devel/setup.bash

After building the package:

-  Copy/move the *franka.sh* file to the root of the catkin_ws
   ``$ cp src/franka_ros_interface/franka.sh ./``
-  Change the values in the copied file (described in the file).


Usage
-----

The 'driver' node can be started by running (can only be used if run in
'master' environment - see `Environments`_ section below):

::

   $ roslaunch franka_interface interface.launch # (use argument load_gripper:=false for starting without gripper)

This exposes a variety of ROS topics and services for communicating with
and controlling the robot. This can be accessed and modified using ROS
topics and services (see below too find out about some of the available
topics and services), or using the provided Python API (see
*franka_interface* and *franka_tools* directories).

.. _the-frankash-environments:

The *franka.sh* environments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the values are correctly modified in the ``franka.sh`` file, different environments can be
set for controlling the robot by sourcing this file.

-  For instance, running ``./franka.sh master`` would start an
   environment assuming that the computer is directly connected to the
   robot (requires Real-Time kernel set up as described in the `FCI
   documentation`_).
-  On the other hand, ``./franka.sh slave`` would start an environment
   assuming that the robot is not connected directly to the computer,
   but to another computer in the network (whose IP must be specified in
   *franka.sh*). This way, if the 'master' is connected to the robot and
   running the driver node (see below), the 'slave' can control the
   robot (**no need for Real Time kernel!**) as long as they are in the
   same network.
-  Simulation environment can be started by running ``./franka.sh sim``
   (only required when using `panda_simulator`_ package).


Some useful ROS topics
^^^^^^^^^^^^^^^^^^^^^^

Published Topics:
'''''''''''''''''

+----------------------------------+----------------------------------+
| ROS Topic                        | Data                             |
+==================================+==================================+
| */                               | gravity, coriolis, jacobian,     |
| franka_ros_interface/custom_fran | cartesian velocity, etc.         |
| ka_state_controller/robot_state* |                                  |
+----------------------------------+----------------------------------+
| */franka_ros_interface/custom_fr | end-effector pose, wrench, etc.  |
| anka_state_controller/tip_state* |                                  |
+----------------------------------+----------------------------------+
| */fr                             | joint positions, velocities,     |
| anka_ros_interface/joint_states* | efforts                          |
+----------------------------------+----------------------------------+
| */franka_ros_interf              | joint positions, velocities,     |
| ace/franka_gripper/joint_states* | efforts of gripper joints        |
+----------------------------------+----------------------------------+

Subscribed Topics:
''''''''''''''''''

+----------------------------------+----------------------------------+
| ROS Topic                        | Data                             |
+==================================+==================================+
| */franka_ros_interface/motio     | command the robot using the      |
| n_controller/arm/joint_commands* | currently active controller      |
+----------------------------------+----------------------------------+
| */franka_ros_interface/franka_g  | (action msg) command the joints  |
| ripper/[move/grasp/stop/homing]* | of the gripper                   |
+----------------------------------+----------------------------------+

Other topics for changing the controller gains (also dynamically
configurable), command timeout, etc. are also available.

ROS Services:
^^^^^^^^^^^^^

Controller manager service can be used to switch between all available
controllers (joint position, velocity, effort). Gripper joints can be
controlled using the ROS ActionClient. Other services for changing
coordinate frames, adding gripper load configuration, etc. are also
available.

Python API
^^^^^^^^^^
Most of the above services and topics are wrapped using simple Python
classes or utility functions, providing more control and simplicity.
See `Python API Documentation`_.
Refer induvidual Python files in *franka_interface* and *franka_tools*
directories for more details.

Related Packages
----------------

-  `panda_simulator`_ : A Gazebo simulator for the Franka Emika Panda
   robot with ROS interface, providing exposed controllers and real-time
   robot state feedback similar to the real robot when using the
   *franka_ros_interface* package. Provides almost complete real-to-sim
   transfer of code.
-  `panda_robot`_ : Python interface providing higher-level control of
   the robot integrated with its gripper control, controller manager,
   coordinate frames manager, etc. with safety checks and other helper
   utilities. It also provides the kinematics and dynamics of the robot
   using the `KDL library`_.
-  `franka_panda_description`_ : Robot description package modified from
   `franka_ros`_ package to include dynamics parameters for the robot
   arm (as estimated in `this paper`_). Also includes transmission and
   control definitions required for the `panda_simulator`_ package.

License
-------
|License|

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

.. |Build Status| image:: https://travis-ci.org/justagist/franka_ros_interface.svg?branch=master
   :target: https://travis-ci.org/justagist/franka_ros_interface
.. |License| image:: https://img.shields.io/badge/License-Apache%202.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
.. |Code Quality| image:: https://api.codacy.com/project/badge/Grade/ec16a09639d341358b73cb8cdaa57d2e    
   :target: https://www.codacy.com/manual/justagist/franka_ros_interface?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=justagist/franka_ros_interface&amp;utm_campaign=Badge_Grade

