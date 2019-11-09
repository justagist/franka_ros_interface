#include <franka_ros_controllers/position_joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_ros_controllers {

bool PositionJointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joints_subscriber_ = node_handle.subscribe(
      "arm/joint_commands", 20, &PositionJointPositionController::jointPosCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PositionJointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PositionJointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PositionJointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  if (!node_handle.getParam("joint_position_limits_lower", joint_position_limits_lower_) ) {
  ROS_ERROR(
      "PositionJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  if (!node_handle.getParam("joint_position_limits_upper", joint_position_limits_upper_) ) {
  ROS_ERROR(
      "PositionJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PositionJointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("PositionJointPositionController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);

  dynamic_reconfigure_joint_controller_params_node_ =
      ros::NodeHandle("position_joint_position_controller/arm/controller_parameters_config");

  dynamic_server_joint_controller_params_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::joint_position_controller_paramsConfig>>(
      dynamic_reconfigure_joint_controller_params_node_);

  dynamic_server_joint_controller_params_->setCallback(
      boost::bind(&PositionJointPositionController::jointControllerParamCallback, this, _1, _2));

  publisher_controller_states_.init(node_handle, "arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.names.resize(joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_names.size());

  }

  return true;
}

void PositionJointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }
  pos_d_ = initial_pos_;
  prev_pos_ = initial_pos_;
  pos_d_target_ = initial_pos_;
}

void PositionJointPositionController::update(const ros::Time& time,
                                            const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(pos_d_[i]);
  }
  double filter_val = filter_joint_pos_ * filter_factor_;
  for (size_t i = 0; i < 7; ++i) {
    prev_pos_[i] = position_joint_handles_[i].getPosition();
    pos_d_[i] = filter_val * pos_d_target_[i] + (1.0 - filter_val) * pos_d_[i];
  }

  if (trigger_publish_() && publisher_controller_states_.trylock()) {
    for (size_t i = 0; i < 7; ++i){

      publisher_controller_states_.msg_.joint_controller_states[i].set_point = pos_d_target_[i];
      publisher_controller_states_.msg_.joint_controller_states[i].process_value = pos_d_[i];
      publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();

      publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;

    }

    publisher_controller_states_.unlockAndPublish();        
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  filter_joint_pos_ = param_change_filter_ * target_filter_joint_pos_ + (1.0 - param_change_filter_) * filter_joint_pos_;

}

bool PositionJointPositionController::checkPositionLimits(std::vector<double> positions)
{
  // bool retval = true;
  for (size_t i = 0;  i < 7; ++i){
    if (!((positions[i] <= joint_position_limits_upper_[i]) && (positions[i] >= joint_position_limits_lower_[i]))){
      return true;
    }
  }

  return false;
}

void PositionJointPositionController::jointPosCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

    if (msg->mode == franka_core_msgs::JointCommand::POSITION_MODE){
      if (msg->position.size() != 7) {
        ROS_ERROR_STREAM(
            "PositionJointPositionController: Published Commands are not of size 7");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;
      }
      else if (checkPositionLimits(msg->position)) {
         ROS_ERROR_STREAM(
            "PositionJointPositionController: Commanded positions are beyond allowed position limits.");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;

      }
      else
      {
        std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());
      }
        // std::cout << "Desired Joint Pos: " << pos_d_[0] << "  " << pos_d_[2] << std::endl;
      
    }
    else ROS_ERROR_STREAM("PositionJointPositionController: Published Command msg are not it JointCommand::POSITION_MODE! Dropping message");
}

void PositionJointPositionController::jointControllerParamCallback(franka_ros_controllers::joint_position_controller_paramsConfig& config,
                               uint32_t level){
  target_filter_joint_pos_ = config.position_joint_delta_filter;
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::PositionJointPositionController,
                       controller_interface::ControllerBase)
