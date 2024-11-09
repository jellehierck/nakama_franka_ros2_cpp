#ifndef NAKAMA_FRANKA_BAG_JOINT_STATE_BAG_RECORDER
#define NAKAMA_FRANKA_BAG_JOINT_STATE_BAG_RECORDER

#include <atomic>
#include <cmath>
#include <memory>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <nakama_franka_interfaces/srv/start_bag_recording.hpp>
#include <nakama_franka_interfaces/srv/stop_bag_recording.hpp>

namespace nakama_bag {

class JointStateBagRecorder : public rclcpp::Node {
 public:
  /// Set the type of the recorded message
  using RecordedTopicMsg = sensor_msgs::msg::JointState;

  /// Default recording topic name to which the recorded messages are stored
  static constexpr const char *DEFAULT_RECORD_TOPIC = "/franka/joint_state";

  /// Service name for starting a recording. Use tilde to prefix with node name, see
  /// https://design.ros2.org/articles/topic_and_service_names.html
  static constexpr const char *START_RECORDING_SERVICE_NAME = "~/start_recording";

  /// Service name for stopping a recording. Use tilde to prefix with node name, see
  /// https://design.ros2.org/articles/topic_and_service_names.html
  static constexpr const char *STOP_RECORDING_SERVICE_NAME = "~/stop_recording";

  JointStateBagRecorder();

  /**
   * Callback on receiving a joint state message, which will record it to the bag.
   *
   * @note This method should never run in parallel with other methods which access the writer, e.g. using callback
   * groups.
   *
   * @param msg Received message.
   */
  void record_joint_state_msg_to_bag(sensor_msgs::msg::JointState::ConstSharedPtr msg);

 private:
  /**
   * Service callback when the start recording service is called.
   *
   * @note This method should never run in parallel with other methods which access the writer, e.g. using callback
   * groups.
   *
   * @param request Request information.
   * @param response Response.
   */
  void start_recording(
    nakama_franka_interfaces::srv::StartBagRecording::Request::ConstSharedPtr request,
    nakama_franka_interfaces::srv::StartBagRecording::Response::SharedPtr     response
  );

  /**
   * Service callback when the stop recording service is called.
   *
   * @note This method should never run in parallel with other methods which access the writer, e.g. using callback
   * groups.
   *
   * @param request Empty.
   * @param response Empty.
   */
  void stop_recording(
    nakama_franka_interfaces::srv::StopBagRecording::Request::ConstSharedPtr request,
    nakama_franka_interfaces::srv::StopBagRecording::Response::SharedPtr     response
  );

  std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;

  rclcpp::CallbackGroup ::SharedPtr                                            recording_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr                joint_states_sub_;
  rclcpp::Service<nakama_franka_interfaces::srv::StartBagRecording>::SharedPtr start_recording_srv_;
  rclcpp::Service<nakama_franka_interfaces::srv::StopBagRecording>::SharedPtr  stop_recording_srv_;

  std::atomic_bool recording_started_{false};
};

}  // namespace nakama_bag
#endif  // NAKAMA_FRANKA_BAG_JOINT_STATE_BAG_RECORDER
