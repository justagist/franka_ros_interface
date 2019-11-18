/***************************************************************************
* Adapted from arm_controller_interface.cpp (sawyer_simulator package)

*
* @package: franka_interface
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019, Saif Sidhik
* Copyright (c) 2013-2018, Rethink Robotics Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

#include <franka_interface/motion_controller_interface.h>
#include <controller_manager_msgs/SwitchController.h>

namespace franka_interface {

void MotionControllerInterface::init(ros::NodeHandle& nh,
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager) {
  current_mode_ = -1;

  if (!nh.getParam("/controllers_config/position_controller", position_controller_name_)) {
        position_controller_name_ = "position_joint_position_controller";
    }
  if (!nh.getParam("/controllers_config/torque_controller", torque_controller_name_)) {
        torque_controller_name_ = "effort_joint_torque_controller";
    }
  if (!nh.getParam("/controllers_config/impedance_controller", impedance_controller_name_)) {
        impedance_controller_name_ = "effort_joint_impedance_controller";
    }
  if (!nh.getParam("/controllers_config/velocity_controller", velocity_controller_name_)) {
        velocity_controller_name_ = "velocity_joint_velocity_controller";
    }

  controller_manager_ = controller_manager;
  joint_command_sub_ = nh.subscribe("/franka_ros_interface/motion_controller/arm/joint_commands", 1,
                       &MotionControllerInterface::jointCommandCallback, this);

  // Command Timeout
  joint_command_timeout_sub_ = nh.subscribe("/franka_ros_interface/motion_controller/arm/joint_command_timeout", 1,
                       &MotionControllerInterface::jointCommandTimeoutCallback, this);
  double command_timeout_default;
  nh.param<double>("command_timeout", command_timeout_default, 0.2);
  auto p_cmd_timeout_length = std::make_shared<ros::Duration>(std::min(1.0,
                                std::max(0.0, command_timeout_default)));
  box_timeout_length_.set(p_cmd_timeout_length);

  ROS_INFO_STREAM("MotionControllerInterface Initialised");

  // Update at 100Hz
  cmd_timeout_timer_ = nh.createTimer(100, &MotionControllerInterface::commandTimeoutCheck, this);
}

void MotionControllerInterface::commandTimeoutCheck(const ros::TimerEvent& e) {
  // lock out other thread(s) which are getting called back via ros.
  std::lock_guard<std::mutex> guard(mtx_);
  // Check Command Timeout
  std::shared_ptr<const ros::Duration>  p_timeout_length;
  box_timeout_length_.get(p_timeout_length);
  std::shared_ptr<const ros::Time>  p_cmd_msg_time;
  box_cmd_timeout_.get(p_cmd_msg_time);
  bool command_timeout = (p_cmd_msg_time && p_timeout_length &&
      ((ros::Time::now() - *p_cmd_msg_time.get()) > (*p_timeout_length.get())));
  if(command_timeout && current_mode_ != franka_core_msgs::JointCommand::POSITION_MODE) {
    // Timeout violated, force robot back to Position Mode
    switchControllers(franka_core_msgs::JointCommand::POSITION_MODE);
    ROS_WARN("MotionControllerInterface: Command timeout violated: Switched to Position control mode.");
  }
}

void MotionControllerInterface::jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM("MotionControllerInterface: Joint command timeout: " << msg.data);
  auto p_cmd_timeout_length = std::make_shared<ros::Duration>(
                                std::min(1.0, std::max(0.0, double(msg.data))));
  box_timeout_length_.set(p_cmd_timeout_length);
}

bool MotionControllerInterface::switchControllers(int control_mode) {
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;

  if(current_mode_ != control_mode)
  {
    switch (control_mode)
    {
      case franka_core_msgs::JointCommand::POSITION_MODE:
        start_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(impedance_controller_name_);
        stop_controllers.push_back(torque_controller_name_);
        stop_controllers.push_back(velocity_controller_name_);
        break;
      case franka_core_msgs::JointCommand::IMPEDANCE_MODE:
        start_controllers.push_back(impedance_controller_name_);
        stop_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(torque_controller_name_);
        stop_controllers.push_back(velocity_controller_name_);
        break;
      case franka_core_msgs::JointCommand::TORQUE_MODE:
        start_controllers.push_back(torque_controller_name_);
        stop_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(impedance_controller_name_);
        stop_controllers.push_back(velocity_controller_name_);
        break;
      case franka_core_msgs::JointCommand::VELOCITY_MODE:
        start_controllers.push_back(velocity_controller_name_);
        stop_controllers.push_back(position_controller_name_);
        stop_controllers.push_back(impedance_controller_name_);
        stop_controllers.push_back(torque_controller_name_);
        break;        
      default:
        ROS_ERROR_STREAM_NAMED("MotionControllerInterface", "Unknown JointCommand mode "
                                << control_mode << ". Ignoring command.");
        return false;
    }
    if (!controller_manager_->switchController(start_controllers, stop_controllers,
                              controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    {
      ROS_ERROR_STREAM_NAMED("MotionControllerInterface", "Failed to switch controllers");
      return false;
    }
    current_mode_ = control_mode;
    ROS_INFO_STREAM("MotionControllerInterface: Controller " << start_controllers[0]
                            << " started; Controllers " << stop_controllers[0] +
                            " and " + stop_controllers[1] << " stopped.");
  }
  return true;
}

void MotionControllerInterface::jointCommandCallback(const franka_core_msgs::JointCommandConstPtr& msg) {
  // lock out other thread(s) which are getting called back via ros.
  std::lock_guard<std::mutex> guard(mtx_);
  if(switchControllers(msg->mode)) {
    auto p_cmd_msg_time = std::make_shared<ros::Time>(ros::Time::now());
    box_cmd_timeout_.set(p_cmd_msg_time);
  }
}

}
