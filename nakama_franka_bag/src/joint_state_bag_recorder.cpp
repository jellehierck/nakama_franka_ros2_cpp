#include "nakama_franka_bag/joint_state_bag_recorder.hpp"

#include <atomic>
#include <cmath>
#include <ctime>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <rosbag2_cpp/writer.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <nakama_franka_interfaces/srv/start_bag_recording.hpp>
#include <nakama_franka_interfaces/srv/stop_bag_recording.hpp>

nakama_bag::JointStateBagRecorder::JointStateBagRecorder() : Node("joint_state_bag_recorder") {
  // Declare parameters
  auto record_topic_param_descriptor = rcl_interfaces::msg::ParameterDescriptor();
  record_topic_param_descriptor.description = "Topic name to record the joint states to in the bag.";
  this->declare_parameter<std::string>("record_topic", DEFAULT_RECORD_TOPIC, record_topic_param_descriptor);

  // Register the services
  start_recording_srv_ = this->create_service<nakama_franka_interfaces::srv::StartBagRecording>(
    START_RECORDING_SERVICE_NAME,
    std::bind(&JointStateBagRecorder::start_recording, this, std::placeholders::_1, std::placeholders::_2)
  );

  stop_recording_srv_ = this->create_service<nakama_franka_interfaces::srv::StopBagRecording>(
    STOP_RECORDING_SERVICE_NAME,
    std::bind(&JointStateBagRecorder::stop_recording, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Subscriptions to topics which are recorded
  // TODO: Add more subscriptions for more franka state data? If needed.
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/franka/joint_states", rclcpp::SystemDefaultsQoS(),
    std::bind(&nakama_bag::JointStateBagRecorder::record_joint_state_msg_to_bag, this, std::placeholders::_1)
  );

  // Initialize the writer
  bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
}

void nakama_bag::JointStateBagRecorder::record_joint_state_msg_to_bag(sensor_msgs::msg::JointState::ConstSharedPtr msg
) {
  // Acquire the mutex to prevent access to the writer during the execution of this function
  std::scoped_lock lock(this->writer_mutex_);

  // If the writer is not open yet, do nothing
  if (!recording_started_.load(std::memory_order_relaxed)) {
    return;
  }

  // Write message to the bag using the time that was provided in the message itself
  bag_writer_->write<RecordedTopicMsg>(*msg, this->get_parameter("record_topic").as_string(), msg->header.stamp);
}

void nakama_bag::JointStateBagRecorder::start_recording(
  nakama_franka_interfaces::srv::StartBagRecording::Request::ConstSharedPtr request,
  nakama_franka_interfaces::srv::StartBagRecording::Response::SharedPtr     response
) {
  // Acquire the mutex to prevent access to the writer during the execution of this function
  std::scoped_lock lock(this->writer_mutex_);

  // Check if the writer is already open: if so, we cannot open it again
  if (recording_started_.load(std::memory_order_relaxed)) {
    response->started = false;
    RCLCPP_ERROR(this->get_logger(), "Could not start recording, already recording!");
    return;
  }

  // Create the file path
  std::stringstream bagfile_uri_ss;
  bagfile_uri_ss << request->bag_path;
  bagfile_uri_ss << request->bag_name;

  if (request->append_timestamp) {
    const std::time_t curr_time = std::lround(this->now().seconds());
    bagfile_uri_ss << std::put_time(std::localtime(&curr_time), "_%FT%T");
  }

  const std::string bagfile_uri = bagfile_uri_ss.str();

  // Open the writer path
  try {
    bag_writer_->open(bagfile_uri);
  } catch (const std::runtime_error &error) {
    response->started = false;
    RCLCPP_ERROR(
      this->get_logger(), "Could not open bag writer with path %s, error: %s", bagfile_uri.c_str(), error.what()
    );
    return;
  }

  // Bagfile is opened succesfully
  recording_started_.store(true);
  response->started = true;
  response->opened_bag_uri = bagfile_uri;
  RCLCPP_INFO(this->get_logger(), "Started recording bag to path %s", bagfile_uri.c_str());
}

void nakama_bag::JointStateBagRecorder::stop_recording(
  nakama_franka_interfaces::srv::StopBagRecording::Request::ConstSharedPtr /*request*/,
  nakama_franka_interfaces::srv::StopBagRecording::Response::SharedPtr /*response*/
) {
  // Acquire the mutex to prevent access to the writer during the execution of this function
  std::scoped_lock lock(this->writer_mutex_);

  // Check if the writer is already open: if so, we cannot open it again
  if (!recording_started_.load(std::memory_order_relaxed)) {
    RCLCPP_WARN(this->get_logger(), "Could not stop recording, it is not started!");
    return;
  }

  // Force close the writer
  bag_writer_->close();
  recording_started_.store(false);
  RCLCPP_INFO(this->get_logger(), "Stopped recording");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nakama_bag::JointStateBagRecorder>());
  rclcpp::shutdown();
  return 0;
}