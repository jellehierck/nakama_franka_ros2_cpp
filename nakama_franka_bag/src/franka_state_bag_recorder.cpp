#include <atomic>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <functional>
#include <iomanip>
#include <memory>
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

#include <rosbag2_cpp/writer.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <nakama_franka_interfaces/srv/start_bag_recording.hpp>
#include <nakama_franka_interfaces/srv/stop_bag_recording.hpp>

class FrankaStateBagRecorder : public rclcpp::Node {
 public:
  FrankaStateBagRecorder() : Node("franka_state_bag_recorder") {
    // Initialize the writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Subscriptions to topics which are recorded
    // TODO: Add more subscriptions for more franka state data? If needed.
    // TODO: Make the input subscription a parameter or use remaps
    measured_joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/franka_robot_state_broadcaster/measured_joint_states", rclcpp::SystemDefaultsQoS(),
      std::bind(&FrankaStateBagRecorder::measured_joint_states_callback, this, std::placeholders::_1)
    );

    // Register the services
    // TODO: Namespace the services (or maybe in launch file?)
    start_recording_srv_ = create_service<nakama_franka_interfaces::srv::StartBagRecording>(
      "start_bag_recording",
      std::bind(&FrankaStateBagRecorder::start_recording, this, std::placeholders::_1, std::placeholders::_2)
    );

    stop_recording_srv_ = create_service<nakama_franka_interfaces::srv::StopBagRecording>(
      "stop_bag_recording",
      std::bind(&FrankaStateBagRecorder::stop_recording, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Ready to start recording");
  }

 private:
  void measured_joint_states_callback(sensor_msgs::msg::JointState::ConstSharedPtr msg) const {
    // If the writer is not open yet, do nothing
    if (!recording_started_.load(std::memory_order_relaxed)) {
      return;
    }

    rclcpp::Time time_stamp = this->now();

    // TODO: Make the stored topic name a parameter
    writer_->write<sensor_msgs::msg::JointState>(*msg, "/joint_state_commands", msg->header.stamp);
  }

  void start_recording(
    const std::shared_ptr<nakama_franka_interfaces::srv::StartBagRecording::Request> request,
    std::shared_ptr<nakama_franka_interfaces::srv::StartBagRecording::Response>      response
  ) {
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
      writer_->open(bagfile_uri);
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
    RCLCPP_INFO(this->get_logger(), "Started recording bag to path %s", bagfile_uri.c_str());
  }

  void stop_recording(
    const std::shared_ptr<nakama_franka_interfaces::srv::StopBagRecording::Request> /*request*/,
    std::shared_ptr<nakama_franka_interfaces::srv::StopBagRecording::Response> /*response*/
  ) {
    // Check if the writer is already open: if so, we cannot open it again
    if (!recording_started_.load(std::memory_order_relaxed)) {
      RCLCPP_WARN(this->get_logger(), "Could not stop recording, it is not started!");
      return;
    }

    // Close the writer
    writer_->close();

    // Bagfile is closed succesfully
    recording_started_.store(false);
    RCLCPP_INFO(this->get_logger(), "Stopped recording");
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_states_sub_;

  rclcpp::Service<nakama_franka_interfaces::srv::StartBagRecording>::SharedPtr start_recording_srv_;
  rclcpp::Service<nakama_franka_interfaces::srv::StopBagRecording>::SharedPtr  stop_recording_srv_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  std::atomic_bool recording_started_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaStateBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
