#pragma once

#include <memory>
#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_ros_controllers {

class EffortJointImpedanceController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            franka_hw::FrankaStateInterface,
                                            hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double angle_{0.0};
  double vel_current_{0.0};

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> pos_d_;
  std::array<double, 7> initial_pos_;
  std::array<double, 7> prev_pos_;
  std::array<double, 7> pos_d_target_;
  std::array<double, 7> dq_d_;
  std::array<double, 7> dq_filtered_;

  franka_hw::FrankaStateInterface* franka_state_interface_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_{};

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};

  ros::Subscriber desired_joints_subscriber_;
  void jointCmdCallback(const sensor_msgs::JointStateConstPtr& msg);
};

}  // namespace franka_example_controllers