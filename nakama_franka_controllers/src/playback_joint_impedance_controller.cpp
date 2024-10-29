#include "nakama_franka_ros2_controllers/playback_joint_impedance_controller.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <controller_interface/controller_interface_base.hpp>

nakama_franka_controllers::PlaybackJointImpedanceController::PlaybackJointImpedanceController() = default;

controller_interface::InterfaceConfiguration
nakama_franka_controllers::PlaybackJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::PlaybackJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn nakama_franka_controllers::PlaybackJointImpedanceController::on_init() {
  try {
    auto_declare<double>("motion_duration", DEFAULT_MOTION_DURATION_SEC);
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::PlaybackJointImpedanceController::
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

  // Start command subscription
  joint_commands_sub_ = get_node()->create_subscription<nakama_franka_interfaces::msg::FrankaJointCommands>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const nakama_franka_interfaces::msg::FrankaJointCommands::SharedPtr msg) {
      rt_command_buffer_.writeFromNonRT(msg);
    }
  );

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::PlaybackJointImpedanceController::on_activate(const rclcpp_lifecycle::State
                                                                           & /*previous_state*/) {
  // reset command buffer if a command came through callback when controller was inactive
  rt_command_buffer_ =
    realtime_tools::RealtimeBuffer<nakama_franka_interfaces::msg::FrankaJointCommands::SharedPtr>(nullptr);

  update_joint_states();
  joint_velocities_curr_filtered_.setZero();
  joint_positions_init_ = joint_positions_curr_;

  elapsed_sec_ = 0.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::PlaybackJointImpedanceController::on_deactivate(const rclcpp_lifecycle::State
                                                                             & /*previous_state*/) {
  // reset command buffer
  rt_command_buffer_ =
    realtime_tools::RealtimeBuffer<nakama_franka_interfaces::msg::FrankaJointCommands::SharedPtr>(nullptr);
  return CallbackReturn::SUCCESS;
}

void nakama_franka_controllers::PlaybackJointImpedanceController::update_joint_states() {
  for (size_t i = 0; i < N_JOINTS; i++) {
    const auto &position_interface = state_interfaces_.at(2 * i);
    const auto &velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_curr_(static_cast<int>(i)) = position_interface.get_value();
    joint_velocities_curr_(static_cast<int>(i)) = velocity_interface.get_value();
  }
}

controller_interface::return_type nakama_franka_controllers::PlaybackJointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration &period
) {
  // Get the joint positions and velocities
  update_joint_states();

  // Update the time
  elapsed_sec_ += period.seconds();

  // Read the next comand from the real-time buffer
  // TODO: only obtain the latest command
  // TODO: check if command is stale
  auto     joint_commands = rt_command_buffer_.readFromRT();
  Vector7d joint_positions_desired;

  // Check if a new command is ready
  if (!joint_commands || !(*joint_commands)) {
    std::chrono::milliseconds warning_throttle_time = std::chrono::milliseconds(5000);
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), warning_throttle_time.count(),
      "At least one command was not ready on real-time buffer in the last %ld ms", warning_throttle_time.count()
    );

    // No command on the buffer, set the desired position as the current position
    // TODO: possibly wait for new command?
    joint_positions_desired = joint_positions_curr_;

  } else {
    // Convert the command to desired joint positions
    for (size_t i = 0; i < N_JOINTS; i++) {
      if (i != 0) {
        joint_positions_desired(static_cast<int>(i)) = (*joint_commands)->commands.at(i);
      }
    }
    // TODO: check if command will result in high motion, e.g. when position is far from current position, and handle
    // that case instead of resulting in reflex error
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
  nakama_franka_controllers::PlaybackJointImpedanceController, controller_interface::ControllerInterface
)