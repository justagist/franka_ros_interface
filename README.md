# Franka ROS Interface [![Build Status](https://travis-ci.org/justagist/franka_ros_interface.svg?branch=master)](https://travis-ci.org/justagist/franka_ros_interface)

A ROS API for the Franka Emika Panda robot, extending the [*franka_ros* package][franka-ros] to expose more information about the robot, and providing low level control of the robot using ROS and Python API.  

Provides controlling and managing the Franka Emika Panda robot (real and simulated). Contains exposed controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager, coordinate frames, etc. Provides almost complete sim-to-real / real-to-sim transfer of code with the [*panda_simulator*][ps-repo] package. 

## Features
  - Low-level *controllers* (joint position, velocity, torque, impedance) available that can be controlled through ROS topics (including position control for gripper).
  - Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.
  - Python API to monitor and control the robot using any of the available controllers.
  - Python API for managing controllers, coordinate frames, controlling and monitoring the gripper.
  - The [*panda_simulator*][ps-repo] package (which is Gazebo-based simulator for the robot) can also be controlled using this package (ROS and Python interface), providing almost complete sim-to-real transfer of code.
  
  ### Continuous Integration Builds
  
  ROS Kinetic / Melodic: [![Build Status](https://travis-ci.org/justagist/franka_ros_interface.svg?branch=master)](https://travis-ci.org/justagist/franka_ros_interface)
 
  ### Dependencies

 - *libfranka* (`sudo apt install ros-$ROS_DISTRO-libfranka` or [install from source][libfranka-doc])
 - *franka-ros* (`sudo apt install ros-$ROS_DISTRO-franka-ros` or [install from source][libfranka-doc])
 
### Installation
Once the above dependencies are installed, the package can be installed using catkin_make:

    $ cd <catkin_ws>
    $ git clone https://github.com/justagist/panda_simulator src/panda_simulator
    $ catkin_make
    $ source devel/setup.bash
 
 After building the package:
 
 - Copy/move the *franka.sh* file to the root of the catkin_ws
    `$ cp franka_ros_interface/franka.sh ./`
 - Change the values in the copied file (described in the file).
 
 ### The *franka.sh* environments
 Once the values are correctly modified, different environments can be set for controlling the robot by sourcing this file.

- For instance, running `./franka.sh master` would start an environment assuming that the computer is directly connected to the robot (requires Real-Time kernel setup as described in the [FCI documentation](https://frankaemika.github.io/docs/installation_linux.html). 
- On the other hand, `./franka.sh slave` would start an environment assuming that the robot is not connected directly to the computer, but to another computer in the network (whose IP must be specified in *franka.sh*). This way, if the 'master' is connected to the robot and running the driver node (see below), the 'slave' can control the robot (**no need for Real Time kernel!**) as long as they are in the same network.
- Simulation environment can be started by running `./franka.sh sim` (only required when using [*panda_simulator*][ps-repo] package).
 
### Usage

The 'driver' node can be started by running (can only be used if run in 'master' environment):
    
    $ roslaunch franka_interface interface.launch # (use argument load_gripper:=false for starting without gripper)
    
This exposes a variety of ROS topics and services for communicating with and controlling the robot. This can be accessed and modified using ROS topics and services (see below too find out about some of the available topics and services), or using the provided Python API (see *franka_interface* and *franka_tools* directories).

#### Some useful ROS topics

##### Published Topics:
| ROS Topic | Data |
| ------ | ------ |
| */franka_ros_interface/custom_franka_state_controller/robot_state* | gravity, coriolis, jacobian, cartesian velocity, etc. |
| */franka_ros_interface/custom_franka_state_controller/tip_state* | end-effector pose, wrench, etc. |
| */franka_ros_interface/joint_states* | joint positions, velocities, efforts |
| */franka_ros_interface/franka_gripper/joint_states* | joint positions, velocities, efforts of gripper joints |

##### Subscribed Topics:
| ROS Topic | Data |
| ------ | ------ |
| */franka_ros_interface/motion_controller/arm/joint_commands* | command the robot using the currently active controller |
| */franka_ros_interface/franka_gripper/[move/grasp/stop/homing]* | (action msg) command the joints of the gripper |

Other topics for changing the controller gains (also dynamically configurable), command timeout, etc. are also available. 

#### ROS Services:
Controller manager service can be used to switch between all available controllers (joint position, velocity, effort). Gripper joints can be controlled using the ROS ActionClient. Other services for changing coordinate frames, adding gripper load configuration, etc. are also available.

#### Python API

Most of the above services and topics are wrapped using simple Python classes or utility functions, providing more control and simplicity. Refer induvidual Python files in *franka_interface* and *franka_tools* directories for more details.

#### Related Packages

- [*panda_simulator*][ps-repo] : A Gazebo simulator for the Franka Emika Panda robot with ROS interface, providing exposed controllers and real-time robot state feedback similar to the real robot when using the *franka_ros_interface* package. Provides almost complete real-to-sim transfer of code.
- [*panda_robot*](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the robot integrated with its gripper control, controller manager, coordinate frames manager, etc. with safety checks and other helper utilities. It also provides the kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).


#### License

Apache 2.0

   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://frankaemika.github.io/docs/franka_ros.html>
   [ps-repo]: <https://github.com/justagist/panda_simulator>
   
   
   
