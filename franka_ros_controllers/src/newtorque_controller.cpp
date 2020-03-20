// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_ros_controllers/newtorque_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_ros_controllers {

bool NewTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;

  torque_params_ = node_handle.subscribe(
    "/torque_target", 20, &NewTorqueController::torqueParamCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());

  ROS_WARN(
      "TorqueController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("TorqueController: Could not read parameter arm_id");
    return false;
  }
  /*if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "TorqueController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }*/



  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names) || joint_limits_.joint_names.size() != 7) {
    ROS_ERROR(
        "TorqueController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }


  std::map<std::string, double> torque_limit_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_effort_limit", torque_limit_map))
  {
    ROS_ERROR("TorqueController: Failed to find joint effort limits on the param server. Aborting controller init");
    return false;
  }


  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (torque_limit_map.find(joint_limits_.joint_names[i]) != torque_limit_map.end()) {
        joint_limits_.effort.push_back(torque_limit_map[joint_limits_.joint_names[i]]);
    } else {
        ROS_ERROR("TorqueController: Unable to find torque limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("TorqueController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TorqueController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("TorqueController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TorqueController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("TorqueController: Error getting effort joint interface from hardware");
    return false;
  }
  ROS_WARN("TorqueController: foop");
  /*for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      ROS_WARN("TorqueController: fooiiip");
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("TorqueController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }*/

  ROS_WARN("TorqueController: foo");
  // Initialize
  target_torque_.setZero();
  desired_torque_.setZero();
  return true;
}

void NewTorqueController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();
}

void NewTorqueController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::VectorXd tau_cmd(7);

  tau_cmd << saturateTorqueRate(desired_torque_, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  // Update signals changed online through dynamic reconfigure
  desired_torque_ = filter_gain_ * target_torque_ + (1 - filter_gain_) * desired_torque_;
}

void NewTorqueController::torqueParamCallback(
     const franka_core_msgs::TorqueCmdConstPtr& msg) {

  if (msg->torque.size() != 7) {
      ROS_ERROR_STREAM("TorqueController: Published Commands are not of size 7");
  }
  else if (checkTorqueLimits(msg->torque)) {
      ROS_ERROR_STREAM("TorqueController: Commanded positions or velicities are beyond allowed position limits.");
  }
  else {
      for (size_t i = 0; i < 7; ++i) {
          target_torque_(i) = msg->torque[i];
      }
      //target_torque_ = msg->torque;
      //std::copy_n(msg->torque.begin(), 7, target_torque_);
  }
}

bool NewTorqueController::checkTorqueLimits(std::vector<double> torques)
{
  for (size_t i = 0;  i < 7; ++i){
    if (!(abs(torques[i]) < joint_limits_.effort[i])){
      return true;
    }
  }

  return false;
}

Eigen::Matrix<double, 7, 1> NewTorqueController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::NewTorqueController,
                       controller_interface::ControllerBase)
