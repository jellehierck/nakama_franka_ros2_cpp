#include "nakama_franka_ros2_controllers/bag_playback_joint_impedance_controller.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <memory>
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
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

nakama_franka_controllers::BagPlaybackJointImpedanceController::BagPlaybackJointImpedanceController() = default;

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
    auto_declare<std::string>("bag_topic", "");
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

  // Get the bag uri parameter and create bag reader object
  bag_storage_options_.uri = get_node()->get_parameter("bag_uri").as_string();
  bag_reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(bag_storage_options_);

  // Get the topic name which should be read from the bag
  bag_joint_state_topic_ = get_node()->get_parameter("bag_topic").as_string();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::BagPlaybackJointImpedanceController::on_activate(const rclcpp_lifecycle::State
                                                                              & /*previous_state*/) {
  // Open the bag
  try {
    bag_reader_->open(bag_storage_options_);
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Could not open bag %s, error: %s", bag_storage_options_.uri.c_str(), error.what()
    );
    return CallbackReturn::ERROR;
  }

  // Check that the bag topic set as parameter is present in the bag
  // TODO: Maybe make topic type configurable? Or create a templated class which can be templated to deserialize msgs
  auto bag_topics_metadata = bag_reader_->get_all_topics_and_types();
  auto is_bag_topic = [this](rosbag2_storage::TopicMetadata metadata) {
    return metadata.name == this->bag_joint_state_topic_ &&
           metadata.type == rosidl_generator_traits::name<sensor_msgs::msg::JointState>();
  };

  if (auto find_it = std::find_if(begin(bag_topics_metadata), end(bag_topics_metadata), is_bag_topic);
      find_it == std::end(bag_topics_metadata)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Could not find topic %s with type %s in bag %s", this->bag_joint_state_topic_.c_str(),
      rosidl_generator_traits::name<sensor_msgs::msg::JointState>(), bag_storage_options_.uri.c_str()
    );
    return CallbackReturn::ERROR;
  }

  //
  update_joint_states();
  joint_velocities_curr_filtered_.setZero();
  joint_positions_init_ = joint_positions_curr_;

  elapsed_sec_ = 0.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::BagPlaybackJointImpedanceController::on_deactivate(const rclcpp_lifecycle::State
                                                                                & /*previous_state*/) {
  // Close the bag reader
  bag_reader_->close();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::BagPlaybackJointImpedanceController::on_cleanup(const rclcpp_lifecycle::State
                                                                             & /*previous_state*/) {
  // Destroy the bag reader
  bag_reader_ = nullptr;

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

controller_interface::return_type nakama_franka_controllers::BagPlaybackJointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration &period
) {
  // Get the joint positions and velocities
  update_joint_states();

  // Update the time
  elapsed_sec_ += period.seconds();

  // Create vector for desired joint positions which will be read from the bag, and initialize to zeros
  Vector7d joint_positions_desired;
  joint_positions_desired.setZero();

  // Read the next comand from the bag
  // TODO: Check if this is possible inside the real-time loop, or if we should use some real-time friendly buffer
  while (bag_reader_->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr bag_msg_serialized = bag_reader_->read_next();

    // If the message is not from the topic we want to publish, shortcut the loop and try the next message
    // TODO: Make topic name configurable
    if (bag_msg_serialized->topic_name == bag_joint_state_topic_) {
      rclcpp::SerializedMessage serialized_msg(*bag_msg_serialized->serialized_data);
      auto                      joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

      // Deserialize the message and check that deserialization is successful
      try {
        joint_state_serialization_.deserialize_message(&serialized_msg, joint_state_msg.get());
      } catch (const std::exception &error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error while deserializing bag message, error: %s", error.what());
        return controller_interface::return_type::ERROR;
      }

      // TODO: check if command is stale

      // Store the joint positions in the bag message as desired joint positions
      for (size_t i = 0; i < N_JOINTS; i++) {
        joint_positions_desired(static_cast<int>(i)) = joint_state_msg->position.at(i);
      }

      break;  // We have found the next desired joint position, stop the loop
    }
  }

  // Check that a new command was set after reading the bag
  if (joint_positions_desired.isZero()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not read next command from the bag");
    return controller_interface::return_type::ERROR;
  }

  // Only command the first joint to move
  for (size_t i = 0; i < N_JOINTS; i++) {
    if (i != 0) {
      joint_positions_desired(static_cast<int>(i)) = joint_positions_init_(static_cast<int>(i));
    }
  }

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