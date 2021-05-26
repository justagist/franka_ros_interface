# Franka ROS Interface [![Release](https://img.shields.io/badge/release-v0.7.1-blue.svg)](https://github.com/justagist/franka_ros_interface/tree/v0.7.1-dev) [![ROS Version](https://img.shields.io/badge/ROS-Melodic,%20Noetic-brightgreen.svg?logo=ros)](https://ros.org/) [![Python 2.7+, 3.6+](https://img.shields.io/badge/python-2.7+,%203.6+-blue.svg?logo=python)](https://www.python.org/downloads/release/python-360/)

[![Codacy Badge](https://app.codacy.com/project/badge/Grade/ec16a09639d341358b73cb8cdaa57d2e)](https://www.codacy.com/manual/justagist/franka_ros_interface?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=justagist/franka_ros_interface&amp;utm_campaign=Badge_Grade)
[![Build Status](https://img.shields.io/travis/com/justagist/franka_ros_interface/v0.7.1-dev?logo=travis-ci)](https://travis-ci.com/justagist/franka_ros_interface)

A ROS interface library for the Franka Emika Panda robot (**_real and [simulated][ps-repo]_**), extending the [franka-ros][franka-ros] library to expose more information about the robot, and
providing low-level control of the robot using ROS and [Python API][fri-doc].

Franka ROS Interface provides utilites for controlling and managing the Franka Emika Panda robot. Contains exposed customisable controllers for the robot (joint position,
velocity, torque), interfaces for the gripper, controller manager, coordinate frames interface, etc. Also provides utilities to control the robot using 'MoveIt!' and ROS Trajectory Action & ActionClient. This package also provides almost complete sim-to-real / real-to-sim transfer of code with the [Panda Simulator][ps-repo] package.

Documentation Page: <https://justagist.github.io/franka_ros_interface>

**This branch requires franka_ros release version 0.7.1** [![franka_ros_version](https://img.shields.io/badge/franka_ros-v0.7.1%20release-yellow.svg)](https://github.com/frankaemika/franka_ros/tree/49e5ac1055e332581b4520a1bd9ac8aaf4580fb1). (For older franka_ros versions, try building this package from the corresponding branches of this repo. All functionalities may not be available in older versions.)

A more unified ROS Python interface built over this package is available at [PandaRobot](https://github.com/justagist/panda_robot), which provides a more intuitive interface class that combines the different API classes in this package. Simple demos are also available.

## Features

- Low-level *controllers* (joint position, velocity, torque, impedance) available that can be controlled through ROS topics and Python API (including position control for gripper).
- Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics and Python API.
- Python API for managing controllers, coordinate frames, collision behaviour, controlling and monitoring the gripper.
- Python API classes and utility functions to control the robot using MoveIt! and ROS Trajectory Action Service.
- The [panda_simulator][ps-repo] package (which is Gazebo-based simulator for the robot) can also be controlled using this package (ROS and Python interface), providing almost complete sim-to-real transfer of code.

**Demo Using [PandaRobot API](https://github.com/justagist/panda_robot) and [Panda Simulator][ps-repo]**

  ![vid](assets/panda_robot_demo.gif)
 Watch video [here](https://youtu.be/4bEVysUIvOY)

  ![vid](assets/panda_simulator.gif)
 Watch video [here](https://www.youtube.com/watch?v=NdSbXC0r7tU)

  ![vid](assets/ts_demo.gif)
 Watch video [here](https://youtu.be/a_HEmYzqEnk)

## Installation

This branch works with ROS Melodic and ROS Noetic. [![Build Status](https://img.shields.io/travis/com/justagist/franka_ros_interface/v0.7.1-dev?logo=travis-ci)](https://travis-ci.com/justagist/franka_ros_interface)

**NOTE:** Tested on:

| ROS Version | Python Version | Franka ROS Branch                                                             |
|-------------|----------------|-------------------------------------------------------------------------------|
| Melodic     | 2.7+           | [melodic-devel](https://github.com/frankaemika/franka_ros/tree/melodic-devel) |
| Noetic      | 3.6+           | [noetic-devel](https://github.com/frankaemika/franka_ros/tree/noetic-devel)   |

### Dependencies

- ROS Melodic / Noetic (preferably the 'desktop-full' version)
- *libfranka (required version >= 0.8.0)* (`sudo apt install ros-$ROS_DISTRO-libfranka` or [install from source][libfranka-doc]). *Use the release version if building from source for reliable build.*
- *franka-ros* v0.7.1 (`sudo apt install ros-$ROS_DISTRO-franka-ros` or [install from source][franka-ros]). *Make sure to use the appropriate branch of franka_ros (`melodic-devel` or `noetic-devel`) depending on your ROS version.*
- *panda_moveit_config* (`sudo apt install ros-$ROS_DISTRO-panda-moveit-config` or [install from source](https://github.com/ros-planning/panda_moveit_config) (the 'melodic-devel' branch also works for ROS Noetic))
- [*franka_panda_description*][fpd-repo] (Clone this repository to the `src` folder of your workspace. See [Related Packages](#related-packages) section for information about package). **NOTE**: Installing this package is optional, but recommended. If you do not want to use the *franka_panda_description* package, make sure you modify the `franka_interface/launch/interface.launch` file and replace all occurences of `franka_panda_description` with `franka_description` (two occurences).

Once the above dependencies are installed, the package can be installed using catkin:

```sh
   cd <catkin_ws>
   git clone -b v0.7.1-dev https://github.com/justagist/franka_ros_interface src/franka_ros_interface
   catkin build # or catkin_make (catkin build is recommended)
   source devel/setup.bash
```

*Note: Python code in this package is written to be compatible with both Python 2 and 3, so make sure you have the Python `future` module installed (`pip install future` (or `pip3 install ..`)).*

After building the package (this is not required if using with simulated robot):

- Copy/move the *franka.sh* file to the root of the catkin_ws `$ cp src/franka_ros_interface/franka.sh ./`
- Change the values in the copied file (described in the file).

## Usage

**NOTE: For using this package with [Panda Simulator][ps-repo], the following sections are not required; all the required "driver" nodes are started along with the simulation launch file, and the Franka ROS Interface API (as well as [PandaRobot](https://github.com/justagist/panda_robot) API) can be directly used. Follow instructions in the [demos](https://github.com/justagist/panda_simulator#demos) section in the Panda Simulator package. See their corresponding [source files](https://github.com/justagist/panda_simulator/tree/melodic-devel/panda_simulator_examples/scripts) for usage examples.**

### The *franka.sh* environments

Once the values are correctly modified in the `franka.sh` file, different environments can be set for controlling the robot by sourcing this file.

- For instance, running `./franka.sh master` would start an environment assuming that the computer is directly connected to the robot (requires Real-Time kernel set up as described in the [FCI documentation][libfranka-doc]).
- On the other hand, `./franka.sh remote` would start an environment assuming that the robot is not connected directly to the computer, but to another computer in the network (whose IP must be specified in *franka.sh*). This way, if the 'master' is connected to the robot and running the driver node (see below), the 'remote' can control the robot (**_no need for Real Time kernel!_**) as long as they are in the same network.
- ~Simulation environment can be started by running `./franka.sh sim` (only required when using [panda_simulator][ps-repo] package).~ *Not required anymore*

More information regarding the usage of `franka.sh` can be found within the [franka.sh file](franka.sh).

### Starting the Franka ROS Interface 'Driver'

The 'driver' node can be started by running (can only be used if run in 'master' environment - see [Environments](#the-frankash-environments) section above):

```sh
    roslaunch franka_interface interface.launch # (use argument load_gripper:=false for starting without gripper)
```

Available keyword arguments for launch file:

- `load_gripper`: start driver node with the Franka gripper (default: `true`).
- `start_controllers`: load the available controllers to the controller manager (default: `true`).
- `start_moveit`: start moveit server along with the driver node (default: `true`).
- `load_demo_planning_scene`: loads a default planning scene for MoveIt planning with simple objects for collision avoidance (default: `true`). See [create_demo_planning_scene.py](franka_moveit/scripts/create_demo_planning_scene.py).

This starts the robot controllers and drivers to expose a variety of ROS topics and services for communicating with and controlling the robot.

### Controlling and Monitoring the Robot

Once the 'driver' is running, the robot can be controlled from another terminal by running in 'master' environment (if running in the same machine as 'driver'), or 'remote' environment (if using a different connected computer). The robot can then be controlled and monitored using ROS topics and services (see below to find out about some of the available topics and services), or using the provided [Python API][fri-doc] (also see [*PandaRobot*](https://github.com/justagist/panda_robot)).

Example of the robot working with MoveIt can be found by running `roslaunch franka_moveit demo_moveit.launch`.

Basic usage of the Python API is shown in the [`demo_interface.py` example file](franka_interface/tests/demo_interface.py). An example of controlling the robot joints with keyboard using direct position control is shown in [`demo_joint_positions_keyboard.py`](https://github.com/justagist/franka_ros_interface/blob/master/franka_interface/tests/demo_joint_positions_keyboard.py) (read the notes in the file before running the demo. start this demo by running `rosrun franka_interface joint_position_keyboard.py`).

See [documentation][fri-doc] for all available methods and functionalities.

See the [tests](https://github.com/justagist/franka_ros_interface/tree/master/franka_interface/tests) and [scripts](https://github.com/justagist/franka_ros_interface/tree/master/franka_interface/scripts) directories in `franka_interface` for few more usage examples.

**More usage examples can be found in the [PandaRobot](https://github.com/justagist/panda_robot) package.**

#### Some useful ROS topics

##### Published Topics

| ROS Topic | Data |
| ------ | ------ |
| */franka_ros_interface/custom_franka_state_controller/robot_state* | gravity, coriolis, jacobian, cartesian velocity, etc. |
| */franka_ros_interface/custom_franka_state_controller/tip_state* | end-effector pose, wrench, etc. |
| */franka_ros_interface/joint_states* | joint positions, velocities, efforts |
| */franka_ros_interface/franka_gripper/joint_states* | joint positions, velocities, efforts of gripper joints |

##### Subscribed Topics

| ROS Topic | Data |
| ------ | ------ |
| */franka_ros_interface/motion_controller/arm/joint_commands* | command the robot using the currently active controller |
| */franka_ros_interface/franka_gripper/[move/grasp/stop/homing]* | (action msg) command the joints of the gripper |

Other topics for changing the controller gains (also dynamically configurable), command timeout, etc. are also available.

#### ROS Services

Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient. Other services for changing coordinate frames, adding gripper load configuration, etc. are also available.

#### Python API

Most of the above services and topics are wrapped using simple Python classes or utility functions, providing more control and simplicity. This includes direct control of the robot and gripper using the provided custom low-level controllers, MoveIt, and JointTrajectoryAction. Refer README files in individual subpackages.

[Documentation][fri-doc]

More usage examples in the [PandaRobot](https://github.com/justagist/panda_robot) package.

## Related Packages

- [*panda_simulator*][ps-repo] : A Gazebo simulator for the Franka Emika Panda robot with ROS interface, providing exposed controllers and real-time robot state feedback similar to the real robot when using the *franka_ros_interface* package. Provides almost complete real-to-sim transfer of code.
- [*PandaRobot*](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the robot integrated with its gripper control, controller manager, coordinate frames manager, etc. with safety checks and other helper utilities. It also provides the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl). It is built over Franka ROS Interface and provides a more intuitive and unified single-class interface.
- [*franka_panda_description*][fpd-repo] : Robot description package modified from [*franka_ros*][franka-ros] package to include dynamics parameters for the robot arm (as estimated in [this paper](https://hal.inria.fr/hal-02265293/document)). Also includes transmission and control definitions required for the [*panda_simulator*][ps-repo] package.

   [ps-repo]: <https://github.com/justagist/panda_simulator>
   [fri-repo]: <https://github.com/justagist/franka_ros_interface>
   [fpd-repo]: <https://github.com/justagist/franka_panda_description>
   [fri-doc]: <https://justagist.github.io/franka_ros_interface>
   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://github.com/frankaemika/franka_ros>

## License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2021, Saif Sidhik

If you use this software for research, please considering citing using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747412.svg)](https://doi.org/10.5281/zenodo.3747412).
