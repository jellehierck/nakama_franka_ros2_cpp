#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_TOPIC_JOINT_IMPEDANCE_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_TOPIC_JOINT_IMPEDANCE_CONTROLLER

#include <cmath>

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>

#include <cstddef>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_tools/realtime_buffer.h>

#include "nakama_franka_interfaces/msg/franka_joint_commands.hpp"  // IWYU pragma: keep

namespace nakama_franka_controllers {

class TopicJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  static constexpr const size_t N_JOINTS = 7;
  // TODO: Make configurable
  static constexpr const double DEFAULT_MOTION_DURATION_SEC = 10.0;
  // TODO: Make configurable?
  static constexpr const double VELOCITY_FILTER_ALPHA_GAIN = 0.99;

  using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

  TopicJointImpedanceController();

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  void update_joint_states();

  // Tools required to subscribe to and store commands
  rclcpp::Subscription<nakama_franka_interfaces::msg::FrankaJointCommands>::SharedPtr           joint_commands_sub_;
  realtime_tools::RealtimeBuffer<nakama_franka_interfaces::msg::FrankaJointCommands::SharedPtr> rt_command_buffer_;

  // TODO: Make goal configurable
  const Vector7d joint_positions_goal_{M_PI / 4.0, M_PI / 4, 0, -3 * M_PI / 4, 0, M_PI / 2, M_PI / 4};
  Vector7d       joint_positions_init_{0, 0, 0, 0, 0, 0, 0};
  Vector7d       joint_positions_curr_;
  Vector7d       joint_velocities_curr_;
  Vector7d       joint_velocities_curr_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;

  double elapsed_sec_ = 0.0;
  double motion_duration_sec_ = DEFAULT_MOTION_DURATION_SEC;

  rclcpp::Time start_time_;
};

}  // namespace nakama_franka_controllers

#endif  // NAKAMA_FRANKA_ROS2_CONTROLLERS_TOPIC_JOINT_IMPEDANCE_CONTROLLER
