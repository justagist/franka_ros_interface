#include <franka_control_ros/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_control_ros {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_control_ros/joint_position_commands", 20, &JointPositionController::jointPosCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }
  pos_d_ = initial_pos_;
  prev_pos_ = initial_pos_;
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(pos_d_[i]);
  }
  for (size_t i = 0; i < 7; ++i) {
    prev_pos_[i] = position_joint_handles_[i].getPosition();
  }


}

void JointPositionController::jointPosCmdCallback(const sensor_msgs::JointStateConstPtr& msg) {

    if (msg->position.size() != 7) {
      ROS_ERROR_STREAM(
          "JointPositionController: Published Commands are not of size 7");
      pos_d_ = prev_pos_;
    }
    else {
      std::copy_n(msg->position.begin(), 7, pos_d_.begin());
      std::cout << "Desired Joint Pos: " << pos_d_[0] << "  " << pos_d_[2] << std::endl;
    }
}

}  // namespace franka_control_ros

PLUGINLIB_EXPORT_CLASS(franka_control_ros::JointPositionController,
                       controller_interface::ControllerBase)
