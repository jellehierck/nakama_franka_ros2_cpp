#include "nakama_franka_ros2_controllers/bag_playback_joint_impedance_controller.hpp"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller_interface_base.hpp>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

controller_interface::InterfaceConfiguration
nakama_franka_controllers::BagPlaybackJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::BagPlaybackJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagPlaybackJointImpedanceController::on_init() {
  try {
    auto_declare<double>("motion_duration", DEFAULT_MOTION_DURATION_SEC);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::string>("bag_uri", "");
    auto_declare<std::string>("bag_topic", DEFAULT_RECORD_TOPIC);
    auto_declare<bool>("process_finished", false);
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::BagPlaybackJointImpedanceController::
  on_configure(                     // NOLINT(readability-function-cognitive-complexity)
    const rclcpp_lifecycle::State & /*previous_state*/
  ) {
  // Get and store motion duration
  motion_duration_sec_ = get_node()->get_parameter("motion_duration").as_double();

  // Get k-gains from parameter
  auto k_gains_param = get_node()->get_parameter("k_gains").as_double_array();
  if (k_gains_param.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains_param.size() != N_JOINTS) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "k_gains should be of size %zu but is of size %ld", N_JOINTS, k_gains_param.size()
    );
    return CallbackReturn::FAILURE;
  }

  // Get d-gains from parameter
  auto d_gains_param = get_node()->get_parameter("d_gains").as_double_array();
  if (d_gains_param.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains_param.size() != N_JOINTS) {
    RCLCPP_FATAL(
      get_node()->get_logger(), "d_gains should be of size %zu but is of size %ld", N_JOINTS, d_gains_param.size()
    );
    return CallbackReturn::FAILURE;
  }

  // Store gains
  for (size_t i = 0; i < N_JOINTS; i++) {
    d_gains_(static_cast<int>(i)) = d_gains_param.at(i);
    k_gains_(static_cast<int>(i)) = k_gains_param.at(i);
  }

  // Create bag reader object
  bag_reader_ = std::make_unique<rosbag2_cpp::Reader>();

  // Get the topic name which should be read from the bag
  bag_joint_state_topic_ = get_node()->get_parameter("bag_topic").as_string();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::BagPlaybackJointImpedanceController::on_cleanup(const rclcpp_lifecycle::State
                                                                             & /*previous_state*/) {
  // Destroy all managed objects which were initialized in the configure stage
  bag_reader_ = nullptr;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::BagPlaybackJointImpedanceController::on_activate(const rclcpp_lifecycle::State
                                                                              & /*previous_state*/) {
  // Get the bag uri parameter and create bag reader object
  auto bag_storage_options = rosbag2_storage::StorageOptions();
  bag_storage_options.uri = get_node()->get_parameter("bag_uri").as_string();

  // Open the bag
  try {
    bag_reader_->open(bag_storage_options);
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Could not open bag %s, error: %s", bag_storage_options.uri.c_str(), error.what()
    );
    return CallbackReturn::ERROR;
  }

  // Check that the bag topic set as parameter is present in the bag
  // TODO: Maybe make topic type configurable? Or create a templated class which can be templated to deserialize msgs
  auto bag_topics_metadata = bag_reader_->get_all_topics_and_types();
  auto is_bag_topic = [this](rosbag2_storage::TopicMetadata metadata) {
    return metadata.name == this->bag_joint_state_topic_ &&
           metadata.type == rosidl_generator_traits::name<RecordedTopicMsg>();
  };
  if (auto find_it = std::find_if(begin(bag_topics_metadata), end(bag_topics_metadata), is_bag_topic);
      find_it == std::end(bag_topics_metadata)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Could not find topic %s with type %s in bag %s", this->bag_joint_state_topic_.c_str(),
      rosidl_generator_traits::name<RecordedTopicMsg>(), bag_storage_options.uri.c_str()
    );
    return CallbackReturn::ERROR;
  }

  // Initialize controller variables
  update_joint_states();
  joint_velocities_curr_filtered_.setZero();
  elapsed_sec_ = 0.0;

  // Indicate that reading is started
  is_reading_.store(true);
  this->get_node()->set_parameter({"process_finished", false});

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::BagPlaybackJointImpedanceController::on_deactivate(const rclcpp_lifecycle::State
                                                                                & /*previous_state*/) {
  // Close the bag reader
  bag_reader_->close();

  return CallbackReturn::SUCCESS;
}

void nakama_franka_controllers::BagPlaybackJointImpedanceController::update_joint_states() {
  for (size_t i = 0; i < N_JOINTS; i++) {
    const auto &position_interface = state_interfaces_.at(2 * i);
    const auto &velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_curr_(static_cast<int>(i)) = position_interface.get_value();
    joint_velocities_curr_(static_cast<int>(i)) = velocity_interface.get_value();
  }
}

std::optional<nakama_franka_controllers::BagPlaybackJointImpedanceController::Vector7d>
nakama_franka_controllers::BagPlaybackJointImpedanceController::read_next_desired_joint_positions_from_bag() {
  Vector7d joint_positions_desired;

  // Loop through the next bag messages until a message on the desired topic is found
  // TODO: Check if this is okay inside the real-time loop, or if we should use some real-time friendly buffer
  while (bag_reader_->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr bag_msg_serialized = bag_reader_->read_next();

    // If the message is not from the topic we want to publish, shortcut the loop and try the next message
    if (bag_msg_serialized->topic_name == bag_joint_state_topic_) {
      rclcpp::SerializedMessage serialized_msg(*bag_msg_serialized->serialized_data);
      auto                      deserialized_msg = std::make_shared<RecordedTopicMsg>();

      // Deserialize the message and check that deserialization is successful
      try {
        recorded_topic_msg_serialization_.deserialize_message(&serialized_msg, deserialized_msg.get());
      } catch (const std::exception &error) {
        // An error occurred, return an empty object to indicate failure
        RCLCPP_ERROR(get_node()->get_logger(), "Error while deserializing bag message, error: %s", error.what());
        return std::nullopt;
      }

      // TODO: check if command is stale

      // Store the joint positions in the bag message as desired joint positions
      for (size_t i = 0; i < N_JOINTS; i++) {
        // The joint names are likely out of order in the stored message. Therefore, we need to find the index of each
        // joint name to get the correct value

        // The following lambda will allow us to find the correct joint name (joint names are numbered 1-7)
        const std::string expected_joint_name = "fr3_joint" + std::to_string(i + 1);
        auto              is_joint_name = [expected_joint_name](std::string msg_joint_name) -> bool {
          return msg_joint_name == expected_joint_name;
        };

        // Find the iterator for the joint name in the received message
        auto found_name_iterator =
          std::find_if(deserialized_msg->name.begin(), deserialized_msg->name.end(), is_joint_name);

        // Determine the index of the found joint name
        long int found_name_index = std::distance(deserialized_msg->name.begin(), found_name_iterator);

        // Use the obtained index to find the corresponding joint position
        joint_positions_desired(static_cast<int>(i)) = deserialized_msg->position.at(found_name_index);
      }

      // We have found the next desired joint position, stop the loop
      return joint_positions_desired;
    }
  }

  // We reached the end of the bag without finding a message. This is probably because all messages are read
  return std::nullopt;
}

controller_interface::return_type nakama_franka_controllers::BagPlaybackJointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
) {
  // Get the joint positions and velocities
  update_joint_states();

  // If not currently reading, we simply send zero torques and exit early
  if (!is_reading_.load(std::memory_order_relaxed)) {
    for (size_t i = 0; i < N_JOINTS; i++) {
      command_interfaces_[i].set_value(0.0);
      return controller_interface::return_type::OK;
    }
  }

  // Read the next comand from the bag
  auto joint_positions_desired_optional = read_next_desired_joint_positions_from_bag();

  // Check if a command was obtained from the bag
  if (!joint_positions_desired_optional.has_value()) {
    // No command was obtained, stop the reading new commands
    // TODO: The reason for no command could also be an error, but we do not propagate that error

    // Set the flags which indicate whether the reading is done
    is_reading_.store(false);
    this->get_node()->set_parameter({"process_finished", true});

    // Send zero torques command
    for (size_t i = 0; i < N_JOINTS; i++) {
      command_interfaces_[i].set_value(0.0);
      return controller_interface::return_type::OK;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Last command is read, the controller is done reading");

    return controller_interface::return_type::OK;
  }

  // If a value was obtaind from the bad, obtain it and calculate the torque command
  auto joint_positions_desired = joint_positions_desired_optional.value();

  // Filter the joint velocities
  joint_velocities_curr_filtered_ = (1 - VELOCITY_FILTER_ALPHA_GAIN) * joint_velocities_curr_filtered_ +
                                    VELOCITY_FILTER_ALPHA_GAIN * joint_velocities_curr_;

  // Calculate the torque command and set the command interface
  Vector7d joint_torques_command = k_gains_.cwiseProduct(joint_positions_desired - joint_positions_curr_) +
                                   d_gains_.cwiseProduct(-joint_velocities_curr_filtered_);
  for (size_t i = 0; i < N_JOINTS; i++) {
    command_interfaces_[i].set_value(joint_torques_command(static_cast<int>(i)));
  }

  return controller_interface::return_type::OK;
}

// Export the controller as a pluginlib plugin

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  nakama_franka_controllers::BagPlaybackJointImpedanceController, controller_interface::ControllerInterface
)