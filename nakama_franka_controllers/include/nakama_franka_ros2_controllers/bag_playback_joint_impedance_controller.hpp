#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_PLAYBACK_JOINT_IMPEDANCE_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_PLAYBACK_JOINT_IMPEDANCE_CONTROLLER

#include <cmath>
#include <cstddef>
#include <memory>

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <realtime_tools/realtime_tools/realtime_buffer.h>

#include "nakama_franka_interfaces/msg/franka_joint_commands.hpp"  // IWYU pragma: keep

namespace nakama_franka_controllers {

class BagPlaybackJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  static constexpr const size_t N_JOINTS = 7;
  // TODO: Make configurable
  static constexpr const double DEFAULT_MOTION_DURATION_SEC = 10.0;
  // TODO: Make configurable?
  static constexpr const double VELOCITY_FILTER_ALPHA_GAIN = 0.99;

  using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

  BagPlaybackJointImpedanceController();

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  void update_joint_states();

  // Tools required to read and (de)serialize messages from a bag
  rosbag2_storage::StorageOptions                     bag_storage_options_;
  std::unique_ptr<rosbag2_cpp::Reader>                bag_reader_;
  rclcpp::Serialization<sensor_msgs::msg::JointState> joint_state_serialization_;

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

#endif  // NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_PLAYBACK_JOINT_IMPEDANCE_CONTROLLER
