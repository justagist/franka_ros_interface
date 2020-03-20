#include <franka_ros_controllers/ji_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_ros_controllers {

bool JIController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("/robot_config/arm_id", arm_id)) {
    ROS_ERROR("JIController: Could not read parameter arm_id");
    return false;
  }

  franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("JIController: Could not get Franka state interface from hardware");
    return false;
  }
  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names) || joint_limits_.joint_names.size() != 7) {
    ROS_ERROR(
        "JIController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JIController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JIController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  k_gains_target_.clear();
  d_gains_target_.clear();

  // k_gains_target_.resize(k_gains_.size());
  // d_gains_target_.resize(d_gains_.size());

  std::map<std::string, double> pos_limit_lower_map;
  std::map<std::string, double> pos_limit_upper_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/lower", pos_limit_lower_map) ) {
  ROS_ERROR(
      "JIController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/upper", pos_limit_upper_map) ) {
  ROS_ERROR(
      "JIController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }


  std::map<std::string, double> velocity_limit_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_velocity_limit", velocity_limit_map))
  {
    ROS_ERROR("JIController: Failed to find joint velocity limits on the param server. Aborting controller init");
    return false;
  }

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (pos_limit_lower_map.find(joint_limits_.joint_names[i]) != pos_limit_lower_map.end())
      {
        joint_limits_.position_lower.push_back(pos_limit_lower_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("JIController: Unable to find lower position limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
    if (pos_limit_upper_map.find(joint_limits_.joint_names[i]) != pos_limit_upper_map.end())
      {
        joint_limits_.position_upper.push_back(pos_limit_upper_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("JIController: Unable to find upper position limit  values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
    if (velocity_limit_map.find(joint_limits_.joint_names[i]) != velocity_limit_map.end())
      {
        joint_limits_.velocity.push_back(velocity_limit_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("JIController: Unable to find velocity limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
    // ROS_INFO_STREAM("K: "<<k_gains_target_[i]);
  }  

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("JIController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JIController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JIController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JIController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("JIController: Exception getting franka state handle: " << ex.what());
    return false;
  }


  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JIController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_limits_.joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JIController: Exception getting joint handles: " << ex.what());
      return false;
    }

    k_gains_target_.push_back(k_gains_[i]);
    d_gains_target_.push_back(d_gains_[i]);
  }


  /*dynamic_reconfigure_controller_gains_node_ =
      ros::NodeHandle("/franka_ros_interface/effort_joint_impedance_controller/arm/controller_parameters_config");

  dynamic_server_controller_config_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::joint_controller_paramsConfig>>(

      dynamic_reconfigure_controller_gains_node_);
  dynamic_server_controller_config_->setCallback(
      boost::bind(&JIController::controllerConfigCallback, this, _1, _2));*/

  desired_joints_subscriber_ = node_handle.subscribe(
      "ji_position_velocity", 20, &JIController::jointCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  /*publisher_controller_states_.init(node_handle, "/franka_ros_interface/motion_controller/arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.controller_name = "effort_joint_impedance_controller";
    publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

  }*/

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  for (size_t i = 0; i < 7; ++i) { // this has to be done again; apparently when the dyn callback is initialised everything is set to zeros again!?
    k_gains_target_[i] = k_gains_[i];
    d_gains_target_[i] = d_gains_[i];
  }

  return true;
}

void JIController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = franka_state_handle_->getRobotState();

  for (size_t i = 0; i < 7; ++i) {
    initial_pos_[i] = robot_state.q[i];
  }
  prev_pos_ = initial_pos_;
  pos_d_target_ = initial_pos_;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  dq_d_ = dq_filtered_;
}

void JIController::update(const ros::Time& time,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = franka_state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated{};
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (pos_d_target_[i] - robot_state.q[i]) +
                          d_gains_[i] * (dq_d_[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);

    prev_pos_[i] = robot_state.q[i];
    k_gains_[i] = filter_params_ * k_gains_target_[i] + (1.0 - filter_params_) * k_gains_[i];
    d_gains_[i] = filter_params_ * d_gains_target_[i] + (1.0 - filter_params_) * d_gains_[i];
  }

}

bool JIController::checkPositionLimits(std::vector<double> positions)
{
  for (size_t i = 0;  i < 7; ++i){
    if (!((positions[i] <= joint_limits_.position_upper[i]) && (positions[i] >= joint_limits_.position_lower[i]))){
      return true;
    }
  }

  return false;
}

bool JIController::checkVelocityLimits(std::vector<double> velocities)
{
  // bool retval = true;
  for (size_t i = 0;  i < 7; ++i){
    if (!(abs(velocities[i]) <= joint_limits_.velocity[i])){
      return true;
    }
  }

  return false;
}

std::array<double, 7> JIController::saturateTorqueRate(
  const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void JIController::jointCmdCallback(const franka_core_msgs::JICmdConstPtr& msg) {

    if (msg->position.size() != 7) {
      ROS_ERROR_STREAM(
          "JIController: Published Commands are not of size 7");
      pos_d_target_ = prev_pos_;
    }
    else if (checkPositionLimits(msg->position) || checkVelocityLimits(msg->velocity)) {
         ROS_ERROR_STREAM(
            "JIController: Commanded positions or velicities are beyond allowed position limits.");
        pos_d_target_ = prev_pos_;

    }
    else {
      std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());
      std::copy_n(msg->velocity.begin(), 7, dq_d_.begin()); // if velocity is not there, the controller fails!!
    }
 
  // else ROS_ERROR_STREAM("JIController: Published Command msg are not of JointCommand::IMPEDANCE_MODE! Dropping message");
}

void JIController::controllerConfigCallback(
    franka_ros_controllers::joint_controller_paramsConfig& config,
    uint32_t /*level*/) {
    ROS_DEBUG_STREAM("JIController: Updating Config");
    //TODO swap out for cart impedance style setting
    k_gains_target_[0] = config.groups.controller_gains.j1_k;
    k_gains_target_[1] = config.groups.controller_gains.j2_k;
    k_gains_target_[2] = config.groups.controller_gains.j3_k;
    k_gains_target_[3] = config.groups.controller_gains.j4_k;
    k_gains_target_[4] = config.groups.controller_gains.j5_k;
    k_gains_target_[5] = config.groups.controller_gains.j6_k;
    k_gains_target_[6] = config.groups.controller_gains.j7_k;

    d_gains_target_[0] = config.groups.controller_gains.j1_d;
    d_gains_target_[1] = config.groups.controller_gains.j2_d;
    d_gains_target_[2] = config.groups.controller_gains.j3_d;
    d_gains_target_[3] = config.groups.controller_gains.j4_d;
    d_gains_target_[4] = config.groups.controller_gains.j5_d;
    d_gains_target_[5] = config.groups.controller_gains.j6_d;
    d_gains_target_[6] = config.groups.controller_gains.j7_d;

}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::JIController,
                       controller_interface::ControllerBase)
