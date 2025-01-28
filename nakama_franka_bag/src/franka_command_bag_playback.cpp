#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <functional>
#include <iomanip>
#include <memory>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/timer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>

#include "nakama_franka_interfaces/srv/start_bag_recording.hpp"
#include "nakama_franka_interfaces/srv/stop_bag_recording.hpp"
#include "nakama_franka_interfaces/msg/franka_joint_commands.hpp"
#include "nakama_franka_interfaces/msg/detail/franka_joint_commands__struct.hpp"
#include "nakama_franka_interfaces/srv/detail/start_bag_recording__struct.hpp"
#include "nakama_franka_interfaces/srv/detail/stop_bag_recording__struct.hpp"

class FrankaCommandBagPlayback : public rclcpp::Node {
 public:
  FrankaCommandBagPlayback() : Node("franka_state_bag_recorder") {
    // Initialize the reader
    // TODO: Move to service instead
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "/home/jelle/thesis/nakama_ws/bags/recording";
    bag_reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    bag_reader_->open(storage_options);

    // Subscriptions to topics which are recorded
    // TODO: Add more subscriptions for more franka state data? If needed.
    // TODO: Make the input subscription a parameter or use remaps
    cmd_publisher_ = this->create_publisher<nakama_franka_interfaces::msg::FrankaJointCommands>("/turtle1/pose", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&FrankaCommandBagPlayback::read_iteration, this)
    );

    // Register the services
    // start_recording_srv_ = create_service<nakama_franka_interfaces::srv::StartBagRecording>(
    //   "start_bag_recording",
    //   std::bind(&FrankaCommandBagPlayback::start_recording, this, std::placeholders::_1, std::placeholders::_2)
    // );

    // stop_recording_srv_ = create_service<nakama_franka_interfaces::srv::StopBagRecording>(
    //   "stop_bag_recording",
    //   std::bind(&FrankaCommandBagPlayback::stop_recording, this, std::placeholders::_1, std::placeholders::_2)
    // );

    RCLCPP_INFO(this->get_logger(), "Ready to start recording");
  }

 private:
  void read_iteration() {
    while (bag_reader_->has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr bag_msg_serialized = bag_reader_->read_next();

      // If the message is not from the topic we want to publish, skip it
      // TODO: Make topic name configurable
      if (bag_msg_serialized->topic_name != "/turtle1/pose") {
        continue;
      }

      // TODO: Verify that message on this topic is of the correct type (sensor_msgs::msg::JointState)
      rclcpp::SerializedMessage serialized_msg(*bag_msg_serialized->serialized_data);
      auto                      joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
      joint_state_serialization_.deserialize_message(&serialized_msg, joint_state_msg.get());

      nakama_franka_interfaces::msg::FrankaJointCommands cmd_msg;
      // TODO: Set the cmd_msg contents from the joint_state_msg

      cmd_publisher_->publish(cmd_msg);

      break;  // Stop the while loop
    }
  }

  //   void start_recording(
  //     const std::shared_ptr<nakama_franka_interfaces::srv::StartBagRecording::Request> request,
  //     std::shared_ptr<nakama_franka_interfaces::srv::StartBagRecording::Response>      response
  //   ) {
  //     // Check if the writer is already open: if so, we cannot open it again
  //     if (recording_started_.load(std::memory_order_relaxed)) {
  //       response->started = false;
  //       RCLCPP_ERROR(this->get_logger(), "Could not start recording, already recording!");
  //       return;
  //     }

  //     // Create the file path
  //     std::stringstream bagfile_uri_ss;
  //     bagfile_uri_ss << request->bag_path;
  //     bagfile_uri_ss << request->bag_name;

  //     if (request->append_timestamp) {
  //       const std::time_t curr_time = std::lround(this->now().seconds());
  //       bagfile_uri_ss << std::put_time(std::localtime(&curr_time), "_%FT%T");
  //     }

  //     const std::string bagfile_uri = bagfile_uri_ss.str();

  //     // Open the writer path
  //     try {
  //       writer_->open(bagfile_uri);
  //     } catch (const std::runtime_error &error) {
  //       response->started = false;
  //       RCLCPP_ERROR(
  //         this->get_logger(), "Could not open bag writer with path %s, error: %s", bagfile_uri.c_str(), error.what()
  //       );
  //       return;
  //     }

  //     // Bagfile is opened succesfully
  //     recording_started_.store(true);
  //     response->started = true;
  //     RCLCPP_INFO(this->get_logger(), "Started recording bag to path %s", bagfile_uri.c_str());
  //   }

  //   void stop_recording(
  //     const std::shared_ptr<nakama_franka_interfaces::srv::StopBagRecording::Request> /*request*/,
  //     std::shared_ptr<nakama_franka_interfaces::srv::StopBagRecording::Response> /*response*/
  //   ) {
  //     // Check if the writer is already open: if so, we cannot open it again
  //     if (!recording_started_.load(std::memory_order_relaxed)) {
  //       RCLCPP_WARN(this->get_logger(), "Could not stop recording, it is not started!");
  //       return;
  //     }

  //     // Close the writer
  //     writer_->close();

  //     // Bagfile is closed succesfully
  //     recording_started_.store(false);
  //     RCLCPP_INFO(this->get_logger(), "Stopped recording");
  //   }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_states_sub_;

  rclcpp::Service<nakama_franka_interfaces::srv::StartBagRecording>::SharedPtr start_recording_srv_;
  rclcpp::Service<nakama_franka_interfaces::srv::StopBagRecording>::SharedPtr  stop_recording_srv_;

  // TODO: Replace timer with cactus-rt loop
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<nakama_franka_interfaces::msg::FrankaJointCommands>::SharedPtr cmd_publisher_;
  std::unique_ptr<rosbag2_cpp::Reader>                                             bag_reader_;
  rclcpp::Serialization<sensor_msgs::msg::JointState>                              joint_state_serialization_;

  std::atomic_bool recording_started_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaCommandBagPlayback>());
  rclcpp::shutdown();
  return 0;
}
