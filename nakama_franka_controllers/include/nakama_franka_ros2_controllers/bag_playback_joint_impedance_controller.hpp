#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_PLAYBACK_JOINT_IMPEDANCE_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_PLAYBACK_JOINT_IMPEDANCE_CONTROLLER

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

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
#include <realtime_tools/realtime_tools/realtime_buffer.h>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <franka_msgs/msg/franka_robot_state.hpp>

namespace nakama_franka_controllers {

/**
 * Controls the robot to follow a path stored inside a rosbag, using joint impedance control.
 */
class BagPlaybackJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  /// Set the type of the recorded message
  using RecordedTopicMsg = franka_msgs::msg::FrankaRobotState;

  static constexpr const size_t N_JOINTS = 7;
  using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

  // TODO: Make configurable
  static constexpr const double DEFAULT_MOTION_DURATION_SEC = 10.0;
  // TODO: Make configurable?
  static constexpr const double VELOCITY_FILTER_ALPHA_GAIN = 0.99;

  /**
   * Specify which hardware command interfaces are used.
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * Specify which hardware state interfaces are used.
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * Initialize all parameters.
   */
  CallbackReturn on_init() override;

  /**
   * Store parameter values and create the bag reader.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Destroy all managed objects created during the configure stage.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Open the bag, check that the bag contents are valid, and initialize control variables.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Close the bag.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Perform the controller update step. This will read the next message on the specified topic from the bag and
   * calculate the joint command to send.
   *
   * @param time Current time.
   * @param period Time since the last controller update step.
   */
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  /**
   * Read and store the joint states from the state interfaces.
   */
  void update_joint_states();

  // Tools required to read and (de)serialize messages from a bag
  std::string                             bag_joint_state_topic_;
  rosbag2_storage::StorageOptions         bag_storage_options_;
  std::unique_ptr<rosbag2_cpp::Reader>    bag_reader_;
  rclcpp::Serialization<RecordedTopicMsg> recorded_topic_msg_serialization_;

  // TODO: Make goal configurable
  const Vector7d joint_positions_goal_{M_PI / 4.0, M_PI / 4, 0, -3 * M_PI / 4, 0, M_PI / 2, M_PI / 4};
  Vector7d       joint_positions_init_;
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
