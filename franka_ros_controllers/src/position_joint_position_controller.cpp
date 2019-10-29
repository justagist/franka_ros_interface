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
      "/franka_ros_controllers/joint_commands", 20, &PositionJointPositionController::jointPosCmdCallback, this,
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
  dynamic_reconfigure_joint_controller_params_node_ =
      ros::NodeHandle("dynamic_reconfigure_joint_controller_params_node");

  dynamic_server_joint_controller_params_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::jointControllerParamsConfig>>(
      dynamic_reconfigure_joint_controller_params_node_);

  dynamic_server_joint_controller_params_->setCallback(
      boost::bind(&PositionJointPositionController::jointControllerParamCallback, this, _1, _2));

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

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  filter_joint_pos_ = param_change_filter_ * target_filter_joint_pos_ + (1.0 - param_change_filter_) * filter_joint_pos_;

}

void PositionJointPositionController::jointPosCmdCallback(const sensor_msgs::JointStateConstPtr& msg) {

    if (msg->position.size() != 7) {
      ROS_ERROR_STREAM(
          "PositionJointPositionController: Published Commands are not of size 7");
      pos_d_ = prev_pos_;
      pos_d_target_ = prev_pos_;
    }
    else {
      std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());
      // std::cout << "Desired Joint Pos: " << pos_d_[0] << "  " << pos_d_[2] << std::endl;
    }
}

void PositionJointPositionController::jointControllerParamCallback(franka_ros_controllers::jointControllerParamsConfig& config,
                               uint32_t level){
  target_filter_joint_pos_ = config.position_joint_delta_filter;
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::PositionJointPositionController,
                       controller_interface::ControllerBase)
