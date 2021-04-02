
Setup Instructions
==================

**Note that version v0.7.1 requires franka_ros version v0.7.1. For the most recent instructions (including instructions for older versions of franka_ros), it is better to follow instructions from the** `README in the source repository <Project Source Code_>`_ **which is more regularly updated.**

Installation
------------

|release|

ROS Kinetic / Melodic: |Build Status|

**NOTE:** *Tested on Ubuntu 18.04 with ROS Melodic. Version for ROS Kinetic is not maintained anymore. The latest updates to the package may not be compatible with Kinetic.*

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

.. _the-frankash-environments:

The *franka.sh* environments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note:: This section is NOT required when using PandaSimulator (see `README in the source repository <Project Source Code_>`_).

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
   (only required when using `Panda Simulator <panda_simulator_>`_ package).

Starting the Franka ROS Interface 'Driver'
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note:: This section is NOT required when using PandaSimulator (see `README in the source repository <Project Source Code_>`_).

The 'driver' node can be started by running (can only be used if run in
'master' environment - see `Environments`_ section above):

::

   $ roslaunch franka_interface interface.launch # (use argument load_gripper:=false for starting without gripper)

Available keyword arguments for launch file:

- ``load_gripper``: start driver node with the Franka gripper (default: ``true``).
- ``start_controllers``: load the available controllers to the controller manager (default: ``true``).
- ``start_moveit``: start moveit server along with the driver node (default: ``true``).
- ``load_demo_planning_scene``: loads a default planning scene for MoveIt planning with simple objects for collision avoidance (default: ``true``). See ``franka_moveit/scripts/create_demo_planning_scene.py``.

This exposes a variety of ROS topics and services for communicating with
and controlling the robot. 

Controlling and Monitoring the Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the 'driver' is running, the robot can be controlled from another terminal by running in 'master' environment (if running in the same machine as 'driver'), or 'remote' environment (if using a different connected computer). The robot can then be controlled and monitored using ROS
topics and services (see below too find out about some of the available
topics and services), or using the provided `Python API <Python API Documentation_>`_.

Basic usage of the API is shown in the `test_robot.py <franka_interface/tests/test_robot.py>`_ example file.
See `documentation <Python API Documentation_>`_ for all available methods and functionalities. More usage
examples can be found in the `PandaRobot <panda_robot_>`_ package.


Some useful ROS topics
^^^^^^^^^^^^^^^^^^^^^^

Published Topics:
'''''''''''''''''

+----------------------------------------------------------------------------+----------------------------------------------------------+
| ROS Topic                                                                  | Data                                                     |
+============================================================================+==========================================================+
| */franka\_ros\_interface/custom\_franka\_state\_controller/robot\_state*   | gravity, coriolis, jacobian, cartesian velocity, etc.    |
+----------------------------------------------------------------------------+----------------------------------------------------------+
| */franka\_ros\_interface/custom\_franka\_state\_controller/tip\_state*     | end-effector pose, wrench, etc.                          |
+----------------------------------------------------------------------------+----------------------------------------------------------+
| */franka\_ros\_interface/joint\_states*                                    | joint positions, velocities, efforts                     |
+----------------------------------------------------------------------------+----------------------------------------------------------+
| */franka\_ros\_interface/franka\_gripper/joint\_states*                    | joint positions, velocities, efforts of gripper joints   |
+----------------------------------------------------------------------------+----------------------------------------------------------+

Subscribed Topics:
''''''''''''''''''

+----------------------------------------------------------------------+-----------------------------------------------------------+
| ROS Topic                                                            | Data                                                      |
+======================================================================+===========================================================+
| */franka\_ros\_interface/motion\_controller/arm/joint\_commands*     | command the robot using the currently active controller   |
+----------------------------------------------------------------------+-----------------------------------------------------------+
| */franka\_ros\_interface/franka\_gripper/[move/grasp/stop/homing]*   | (action msg) command the joints of the gripper            |
+----------------------------------------------------------------------+-----------------------------------------------------------+

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
`Python API Documentation`_.

Most of the above services and topics are wrapped using simple Python
classes or utility functions, providing more control and simplicity.
Refer README files in individual subpackages. More usage
examples can be found in the `PandaRobot <panda_robot_>`_ package
(see package description below).

Related Packages
----------------

-  `Panda Simulator <panda_simulator_>`_ : A Gazebo simulator for the Franka Emika Panda
   robot with ROS interface, providing exposed controllers and real-time
   robot state feedback similar to the real robot when using the
   *franka_ros_interface* package. Provides almost complete real-to-sim
   transfer of code.
-  `PandaRobot <panda_robot_>`_ : Python interface providing higher-level control of
   the robot integrated with its gripper control, controller manager,
   coordinate frames manager, etc. with safety checks and other helper
   utilities. It also provides the kinematics and dynamics of the robot
   using the `KDL library`_. It is built over Franka ROS Interface and 
   provides a more intuitive and unified single-class interface.
-  `franka_panda_description`_ : Robot description package modified from
   `franka_ros`_ package to include dynamics parameters for the robot
   arm (as estimated in `this paper`_). Also includes transmission and
   control definitions required for the `panda_simulator`_ package.

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

