#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_RECORDER_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_BAG_RECORDER_CONTROLLER

#include <atomic>
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
  static constexpr const size_t N_JOINTS = 7;
  using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

  /// Service name for starting a recording. Use tilde to prefix with node name, see
  /// https://design.ros2.org/articles/topic_and_service_names.html
  static constexpr const char *START_RECORDING_SERVICE_NAME = "~/start_recording";

  /// Service name for stopping a recording. Use tilde to prefix with node name, see
  /// https://design.ros2.org/articles/topic_and_service_names.html
  static constexpr const char *STOP_RECORDING_SERVICE_NAME = "~/stop_recording";

  /// Set the type of the recorded message
  using RecordedTopicMsg = franka_msgs::msg::FrankaRobotState;

  /// Parameter name for the topic to which the recorded messages are stored
  static constexpr const char *RECORD_TOPIC_PARAMETER_NAME = "record_topic";

  /// Default recording topic name to which the recorded messages are stored
  static constexpr const char *DEFAULT_RECORD_TOPIC_NAME = "/robot_state";

  BagRecorderController();

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  void start_recording(
    nakama_franka_interfaces::srv::StartBagRecording::Request::ConstSharedPtr request,
    nakama_franka_interfaces::srv::StartBagRecording::Response::SharedPtr     response
  );

  void stop_recording(
    nakama_franka_interfaces::srv::StopBagRecording::Request::ConstSharedPtr request,
    nakama_franka_interfaces::srv::StopBagRecording::Response::SharedPtr     response
  );

  void force_stop_recording();

  std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_semantic_component_;

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
