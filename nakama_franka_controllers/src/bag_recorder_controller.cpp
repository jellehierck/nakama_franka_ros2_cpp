#include "nakama_franka_ros2_controllers/bag_recorder_controller.hpp"

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <exception>
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
#include <rclcpp/duration.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <controller_interface/controller_interface_base.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <nakama_franka_interfaces/srv/start_bag_recording.hpp>
#include <nakama_franka_interfaces/srv/stop_bag_recording.hpp>
#include <vector>

nakama_franka_controllers::BagRecorderController::BagRecorderController() = default;

controller_interface::InterfaceConfiguration
nakama_franka_controllers::BagRecorderController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::BagRecorderController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // TODO: Semantic component disabled because it does not work with fake hardware
  // state_interfaces_config.names = franka_robot_semantic_component_->get_state_interface_names();

  for (size_t i = 1; i <= N_JOINTS; i++) {
    state_interfaces_config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagRecorderController::on_init() {
  try {
    auto_declare<std::string>("record_topic", DEFAULT_RECORD_TOPIC_NAME);
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagRecorderController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Get and store the topic name
  bag_record_topic_name_ = get_node()->get_parameter("record_topic").as_string();

  // Initialize the writer
  bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();

  // Register the services
  start_recording_srv_ = this->get_node()->create_service<nakama_franka_interfaces::srv::StartBagRecording>(
    START_RECORDING_SERVICE_NAME, std::bind(
                                    &nakama_franka_controllers::BagRecorderController::start_recording, this,
                                    std::placeholders::_1, std::placeholders::_2
                                  )
  );

  stop_recording_srv_ = this->get_node()->create_service<nakama_franka_interfaces::srv::StopBagRecording>(
    STOP_RECORDING_SERVICE_NAME, std::bind(
                                   &nakama_franka_controllers::BagRecorderController::stop_recording, this,
                                   std::placeholders::_1, std::placeholders::_2
                                 )
  );

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagRecorderController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // Destroy all managed objects which were initialized in the configure stage
  bag_writer_ = nullptr;
  start_recording_srv_ = nullptr;
  stop_recording_srv_ = nullptr;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagRecorderController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  // TODO: Semantic component disabled because it does not work with fake hardware
  // franka_robot_semantic_component_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagRecorderController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/
) {
  if (recording_started_.load(std::memory_order_relaxed)) {
    RCLCPP_WARN(
      get_node()->get_logger(), "Deactivate called, but the recording was still running! Stopping the recording now"
    );
    force_stop_recording();
  }

  // TODO: Semantic component disabled because it does not work with fake hardware
  // franka_robot_semantic_component_->release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type nakama_franka_controllers::BagRecorderController::update(
  const rclcpp::Time &time, const rclcpp::Duration & /*period*/
) {
  // TODO: Semantic component disabled because it does not work with fake hardware
  // // TODO: Possibly initialize this message in header file to prevent reallocation in every control update step
  // franka_msgs::msg::FrankaRobotState robot_state_msg;
  // franka_robot_semantic_component_->get_values_as_message(robot_state_msg);
  // // TODO: Convert robot state to other message format to only extract relevant information
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.position = std::vector<double>(N_JOINTS, 0.0);
  for (size_t i = 0; i < N_JOINTS; i++) {
    const auto &position_interface = state_interfaces_.at(i);

    joint_state_msg.position[i] = position_interface.get_value();
  }

  try {
    // Only write to the bag if the recording is started
    if (recording_started_.load(std::memory_order_relaxed)) {
      // TODO: Semantic component disabled because it does not work with fake hardware
      // bag_writer_->write<RecordedTopicMsg>(robot_state_msg, bag_record_topic_name_, time);
      bag_writer_->write<RecordedTopicMsg>(joint_state_msg, bag_record_topic_name_, time);
    }

  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not write message to rosbag, error: %s", error.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void nakama_franka_controllers::BagRecorderController::start_recording(
  nakama_franka_interfaces::srv::StartBagRecording::Request::ConstSharedPtr request,
  nakama_franka_interfaces::srv::StartBagRecording::Response::SharedPtr     response
) {
  // Check if the writer is already open: if so, we cannot open it again
  if (recording_started_.load(std::memory_order_relaxed)) {
    response->started = false;
    RCLCPP_ERROR(get_node()->get_logger(), "Could not start recording, already recording!");
    return;
  }

  // Create the file path
  std::stringstream bagfile_uri_ss;
  bagfile_uri_ss << request->bag_path;
  bagfile_uri_ss << request->bag_name;

  if (request->append_timestamp) {
    const std::time_t curr_time = std::lround(get_node()->now().seconds());
    bagfile_uri_ss << std::put_time(std::localtime(&curr_time), "_%FT%T");
  }

  const std::string bagfile_uri = bagfile_uri_ss.str();

  // Open the writer path
  try {
    bag_writer_->open(bagfile_uri);
  } catch (const std::runtime_error &error) {
    response->started = false;
    RCLCPP_ERROR(
      get_node()->get_logger(), "Could not open bag writer with path %s, error: %s", bagfile_uri.c_str(), error.what()
    );
    return;
  }

  // Bagfile is opened succesfully
  recording_started_.store(true);
  response->started = true;
  response->opened_bag_uri = bagfile_uri;
  RCLCPP_INFO(get_node()->get_logger(), "Started recording bag to path %s", bagfile_uri.c_str());
}

void nakama_franka_controllers::BagRecorderController::stop_recording(
  nakama_franka_interfaces::srv::StopBagRecording::Request::ConstSharedPtr /*request*/,
  nakama_franka_interfaces::srv::StopBagRecording::Response::SharedPtr /*response*/
) {
  // Check if the writer is already open: if so, we cannot open it again
  if (!recording_started_.load(std::memory_order_relaxed)) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not stop recording, it is not started!");
    return;
  }

  // Force close the writer
  force_stop_recording();
  RCLCPP_INFO(get_node()->get_logger(), "Stopped recording");
}

void nakama_franka_controllers::BagRecorderController::force_stop_recording() {
  // Close the writer
  bag_writer_->close();
  recording_started_.store(false);
}

// Export the controller as a pluginlib plugin

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nakama_franka_controllers::BagRecorderController, controller_interface::ControllerInterface)