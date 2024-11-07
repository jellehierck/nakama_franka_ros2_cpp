#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_RECORDER_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_RECORDER_CONTROLLER

#include <atomic>
#include <cmath>
#include <cstddef>
#include <memory>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <string>

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <realtime_tools/realtime_tools/realtime_buffer.h>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <franka_semantic_components/franka_robot_state.hpp>

#include <nakama_franka_interfaces/srv/start_bag_recording.hpp>
#include <nakama_franka_interfaces/srv/stop_bag_recording.hpp>

namespace nakama_franka_controllers {

class BagRecorderController : public controller_interface::ControllerInterface {
 public:
  /// Set the type of the recorded message
  using RecordedTopicMsg = sensor_msgs::msg::JointState;
  // TODO: Semantic component disabled because it does not work with fake hardware
  // using RecordedTopicMsg = franka_msgs::msg::FrankaRobotState;

  /// Default recording topic name to which the recorded messages are stored
  static constexpr const char *DEFAULT_RECORD_TOPIC_NAME = "/robot_state";

  /// Service name for starting a recording. Use tilde to prefix with node name, see
  /// https://design.ros2.org/articles/topic_and_service_names.html
  static constexpr const char *START_RECORDING_SERVICE_NAME = "~/start_recording";

  /// Service name for stopping a recording. Use tilde to prefix with node name, see
  /// https://design.ros2.org/articles/topic_and_service_names.html
  static constexpr const char *STOP_RECORDING_SERVICE_NAME = "~/stop_recording";

  static constexpr const size_t N_JOINTS = 7;
  using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

  BagRecorderController();

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
   * Store parameter values, create the bag writer and services.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Destroy all managed objects created during the configure stage.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Bind to the hardware state interfaces.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Close the bag if it was still running and release the hardware state interfaces.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Perform the controller update step. This will read the robot state and store it to the bag.
   *
   * @param time Current time.
   * @param period Time since the last controller update step.
   */
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  /**
   * Service callback when the start recording service is called.
   * @param request Request information.
   * @param response Response.
   */
  void start_recording(
    nakama_franka_interfaces::srv::StartBagRecording::Request::ConstSharedPtr request,
    nakama_franka_interfaces::srv::StartBagRecording::Response::SharedPtr     response
  );

  /**
   * Service callback when the stop recording service is called.
   * @param request Empty.
   * @param response Empty.
   */
  void stop_recording(
    nakama_franka_interfaces::srv::StopBagRecording::Request::ConstSharedPtr request,
    nakama_franka_interfaces::srv::StopBagRecording::Response::SharedPtr     response
  );

  /**
   * Force stop the recording without checking if it was still running.
   */
  void force_stop_recording();

  // TODO: Semantic component disabled because it does not work with fake hardware
  // std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_semantic_component_;

  std::string                          bag_record_topic_name_;
  rosbag2_storage::StorageOptions      bag_storage_options_;
  std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;

  rclcpp::Service<nakama_franka_interfaces::srv::StartBagRecording>::SharedPtr start_recording_srv_;
  rclcpp::Service<nakama_franka_interfaces::srv::StopBagRecording>::SharedPtr  stop_recording_srv_;

  std::atomic_bool recording_started_{false};

  rclcpp::Time start_time_;
};

}  // namespace nakama_franka_controllers

#endif  // NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_RECORDER_CONTROLLER
