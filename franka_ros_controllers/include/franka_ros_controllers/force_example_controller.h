// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>

//X
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>
#include <mutex>
//X

/* #include <franka_ros_controllers/desired_mass_paramConfig.h> */

namespace franka_ros_controllers {

class ForceExampleController : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface, //O not this
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> { //O not this
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  //O instead of eigen, used std
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double desired_mass_{2.0};
  double target_mass_{2.0};
  double k_p_{0.0};
  double k_i_{0.0};
  double target_k_p_{0.0};
  double target_k_i_{0.0};
  double filter_gain_{0.001};
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  static constexpr double kDeltaTauMax{1.0};

  //O
  std::array<double, 7> jnt_cmd_{};
  std::array<double, 7> prev_jnt_cmd_{};
  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  ros::Subscriber desired_joints_subscriber_;
  franka_core_msgs::JointLimits joint_limits_;
  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;
  bool checkTorqueLimits(std::vector<double> torques);
  void jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);
  //O

  // Dynamic reconfigure
  /*
  std::unique_ptr<dynamic_reconfigure::Server<franka_ros_controllers::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_ros_controllers::desired_mass_paramConfig& config,
                                uint32_t level);
  */
};

}  // namespace franka_ros_controllers
