#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_DIRECT_JOINT_POSITION_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_DIRECT_JOINT_POSITION_CONTROLLER

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nakama_franka_controllers {

class LocalJointPositionController : public controller_interface::ControllerInterface {
 public:
  static constexpr const int N_JOINTS = 7;
  static constexpr const double DEFAULT_MOTION_DURATION_SEC = 10.0;

  LocalJointPositionController();

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::array<double, N_JOINTS> initial_q_{0, 0, 0, 0, 0, 0, 0};
  const std::array<double, N_JOINTS> goal_q_{M_PI / 4.0, M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4};

  double elapsed_sec_ = 0.0;
  double motion_duration_sec_ = DEFAULT_MOTION_DURATION_SEC;

  bool initialization_flag_{true};

  rclcpp::Time start_time_;
};

}  // namespace nakama_franka_controllers

#endif  // NAKAMA_FRANKA_ROS2_CONTROLLERS_DIRECT_JOINT_POSITION_CONTROLLER
