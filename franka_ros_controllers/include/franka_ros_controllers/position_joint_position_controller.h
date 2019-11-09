
#pragma once

#include <array>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_ros_controllers/joint_position_controller_paramsConfig.h>

#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>

#include <mutex>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

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

  double filter_joint_pos_{0.3};
  double target_filter_joint_pos_{0.3};
  double filter_factor_{0.01};

  double param_change_filter_{0.005};

  // Dynamic reconfigure
  std::unique_ptr< dynamic_reconfigure::Server<franka_ros_controllers::joint_position_controller_paramsConfig> > dynamic_server_joint_controller_params_;
  ros::NodeHandle dynamic_reconfigure_joint_controller_params_node_;

  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;

  void jointControllerParamCallback(franka_ros_controllers::joint_position_controller_paramsConfig& config,
                               uint32_t level);
  void jointPosCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);
};

}  // namespace franka_ros_controllers
