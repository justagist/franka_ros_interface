
#pragma once

#include <array>
#include <string>
#include <vector>

#include <sensor_msgs/JointState.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_ros_controllers {

class PositionJointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::array<double, 7> initial_pos_{};
  std::array<double, 7> prev_pos_{};
  std::array<double, 7> pos_d_target_{};
  std::array<double, 7> pos_d_;

  // joint_cmd subscriber
  ros::Subscriber desired_joints_subscriber_;

  double filter_params_{0.005};


  void jointPosCmdCallback(const sensor_msgs::JointStateConstPtr& msg);
};

}  // namespace franka_ros_controllers
