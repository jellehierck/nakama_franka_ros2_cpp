#include "nakama_franka_ros2_controllers/local_joint_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>
#include <cstdio>

nakama_franka_controllers::LocalJointPositionController::LocalJointPositionController() {}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::LocalJointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= N_JOINTS; ++i) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::LocalJointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= N_JOINTS; ++i) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
  }

  return config;
}

controller_interface::CallbackReturn nakama_franka_controllers::LocalJointPositionController::on_init() {
  try {
    auto_declare<double>("motion_duration", DEFAULT_MOTION_DURATION_SEC);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::LocalJointPositionController::
  on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  motion_duration_sec_ = get_node()->get_parameter("motion_duration").as_double();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn nakama_franka_controllers::LocalJointPositionController::
  on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_sec_ = 0.0;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type nakama_franka_controllers::LocalJointPositionController::update(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period
) {
  // In the first iteration, store the current pose as the initial pose
  if (initialization_flag_) {
    for (int i = 0; i < N_JOINTS; i++) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
    }
    initialization_flag_ = false;
  }

  elapsed_sec_ += period.seconds();
  double motion_duration_fraction = std::min(elapsed_sec_ / motion_duration_sec_, 1.0);
  double motion_fraction = 0.5 * (1 + std::cos(M_PI * (1 + motion_duration_fraction)));

  for (int i = 0; i < N_JOINTS; ++i) {
    if (i == 0) {
      // Only command the first joint to move
      command_interfaces_[i].set_value(initial_q_.at(i) * (1 - motion_fraction) - goal_q_.at(i) * motion_fraction);
    } else {
      command_interfaces_[i].set_value(initial_q_.at(i));
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn nakama_franka_controllers::LocalJointPositionController::
  on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_sec_ = 0.0;
  return CallbackReturn::SUCCESS;
}

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
  nakama_franka_controllers::LocalJointPositionController, controller_interface::ControllerInterface
)