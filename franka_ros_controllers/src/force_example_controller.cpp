// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_ros_controllers/force_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_ros_controllers {

bool ForceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  std::map<std::string, double> torque_limit_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_effort_limit", torque_limit_map)) {
    ROS_ERROR("EffortJointTorqueController: Failed to find joint effort limits on the param server. Aborting controller init");
    return false;
  }

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (torque_limit_map.find(joint_limits_.joint_names[i]) != torque_limit_map.end()) {
        joint_limits_.effort.push_back(torque_limit_map[joint_limits_.joint_names[i]]);
      } else {
        ROS_ERROR("EffortJointTorqueController: Unable to find torque limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }

  //O
    double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("ForceExampleController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);


  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/joint_commands", 20, &ForceExampleController::jointCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  publisher_controller_states_.init(node_handle, "/franka_ros_interface/motion_controller/arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.controller_name = "force_example_controller";
    publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

  }
  //O

  /*
  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::desired_mass_paramConfig>>(

      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceExampleController::desiredMassParamCallback, this, _1, _2));
  */
  return true;
}

void ForceExampleController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();

  //O
  std::fill(jnt_cmd_.begin(), jnt_cmd_.end(), 0);
  prev_jnt_cmd_ = jnt_cmd_;
  //O
}

void ForceExampleController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(2) = desired_mass_ * -9.81;
  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  // FF + PI control (PI gains are initially all 0)
  tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  
  // Doesnt error if this is commented. 
  /*if (trigger_publish_() && publisher_controller_states_.trylock()) {
      for (size_t i = 0; i < 7; ++i){

        //publisher_controller_states_.msg_.joint_controller_states[i].set_point = jnt_cmd_[i];
        publisher_controller_states_.msg_.joint_controller_states[i].process_value = tau_cmd[i];
        publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();
        publisher_controller_states_.msg_.joint_controller_states[i].command = tau_cmd[i];
        publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;
      }
      publisher_controller_states_.unlockAndPublish();
  }*/
  
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    prev_jnt_cmd_[i] = tau_cmd[i];
  }

  // Update signals changed online through dynamic reconfigure
  target_mass_ = 0.1;
  target_k_p_ = 0.0;
  target_k_i_ = 0.0; 

  desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
  k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
  k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
}

/*
void ForceExampleController::desiredMassParamCallback(
    franka_ros_controllers::desired_mass_paramConfig& config,
    uint32_t)  {
  target_mass_ = 2.0; //config.desired_mass;
  target_k_p_ = 0.0; //config.k_p;
  target_k_i_ = 0.0; //config.k_i;
}
*/

bool ForceExampleController::checkTorqueLimits(std::vector<double> torques)
{
  //TODO incoporate this
  for (size_t i = 0;  i < 7; ++i){
    if (!(abs(torques[i]) < joint_limits_.effort[i])){
      return true;
    }
  }

  return false;
}

Eigen::Matrix<double, 7, 1> ForceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}


void ForceExampleController::jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

  if (msg->mode == franka_core_msgs::JointCommand::TORQUE_MODE){
    if (msg->effort.size() != 7) {
      ROS_ERROR_STREAM(
          "ForceExampleController: Published Commands are not of size 7");
      std::fill(jnt_cmd_.begin(), jnt_cmd_.end(), 0);
      jnt_cmd_= prev_jnt_cmd_;
    }
    else if (checkTorqueLimits(msg->effort)) {
         ROS_ERROR_STREAM(
            "ForceExampleController: Commanded torques are beyond allowed torque limits.");
        std::fill(jnt_cmd_.begin(), jnt_cmd_.end(), 0);
        jnt_cmd_= prev_jnt_cmd_;
    }
    else {
      std::copy_n(msg->effort.begin(), 7, jnt_cmd_.begin());

    }
  }
  // else ROS_ERROR_STREAM("EffortJointTorqueController: Published Command msg are not of JointCommand::TORQUE_MODE! Dropping message");
}


}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::ForceExampleController,
                       controller_interface::ControllerBase)
