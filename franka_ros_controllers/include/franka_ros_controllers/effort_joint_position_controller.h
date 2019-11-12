#pragma once

#include <memory>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_ros_controllers/joint_position_controller_paramsConfig.h>
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>

#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <mutex>

namespace franka_ros_controllers {

class EffortJointPositionController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaStateInterface,
                                            hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  static std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated);  // NOLINT (readability-identifier-naming)

  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};

  double filter_params_{0.005};
  std::vector<double> k_gains_;
  std::vector<double> k_gains_target_;
  std::vector<double> d_gains_;
  std::vector<double> d_gains_target_;
  double coriolis_factor_{1.0};
  std::array<double, 7> pos_d_;
  std::array<double, 7> initial_pos_;
  std::array<double, 7> prev_pos_;
  std::array<double, 7> pos_d_target_;
  std::array<double, 7> d_error_;
  std::array<double, 7> p_error_last_;

  // std::vector<double> joint_position_limits_lower_;
  // std::vector<double> joint_position_limits_upper_;
  // std::vector<double> joint_velocity_limits_;

  // std::map<std::string, double> acceleration_map_;;

  franka_hw::FrankaStateInterface* franka_state_interface_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_{};

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};

  ros::Subscriber desired_joints_subscriber_;
  std::unique_ptr< dynamic_reconfigure::Server<franka_ros_controllers::joint_position_controller_paramsConfig> > dynamic_server_controller_config_;
  ros::NodeHandle dynamic_reconfigure_controller_gains_node_;

  franka_core_msgs::JointLimits joint_limits_;

  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;

  bool checkPositionLimits(std::vector<double> positions);
  bool checkVelocityLimits(std::vector<double> positions);

  void controllerConfigCallback(franka_ros_controllers::joint_position_controller_paramsConfig& config,
                               uint32_t level);
  void jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);
};

}  // namespace franka_ros_controllers