#include "nakama_franka_ros2_controllers/local_joint_impedance_controller.hpp"

#include <algorithm>
#include <cassert>
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

nakama_franka_controllers::LocalJointImpedanceController::LocalJointImpedanceController() = default;

controller_interface::InterfaceConfiguration
nakama_franka_controllers::LocalJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::LocalJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn nakama_franka_controllers::LocalJointImpedanceController::on_init() {
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

controller_interface::CallbackReturn nakama_franka_controllers::LocalJointImpedanceController::
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

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::LocalJointImpedanceController::on_activate(const rclcpp_lifecycle::State
                                                                        & /*previous_state*/) {
  update_joint_states();
  joint_velocities_curr_filtered_.setZero();
  joint_positions_init_ = joint_positions_curr_;

  elapsed_sec_ = 0.0;

  return CallbackReturn::SUCCESS;
}

void nakama_franka_controllers::LocalJointImpedanceController::update_joint_states() {
  for (size_t i = 0; i < N_JOINTS; i++) {
    const auto &position_interface = state_interfaces_.at(2 * i);
    const auto &velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_curr_(static_cast<int>(i)) = position_interface.get_value();
    joint_velocities_curr_(static_cast<int>(i)) = velocity_interface.get_value();
  }
}

controller_interface::return_type nakama_franka_controllers::LocalJointImpedanceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration &period
) {
  // Get the joint positions and velocities
  update_joint_states();

  // Update the time
  elapsed_sec_ += period.seconds();

  // Calculate how far along the motion path should be to calculate the next desired joint position
  double   motion_duration_fraction = std::min(elapsed_sec_ / motion_duration_sec_, 1.0);
  double   motion_fraction = 0.5 * (1 + std::cos(M_PI * (1 + motion_duration_fraction)));
  Vector7d joint_positions_desired =
    joint_positions_init_ * (1 - motion_fraction) - -joint_positions_goal_ * motion_fraction;

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
  nakama_franka_controllers::LocalJointImpedanceController, controller_interface::ControllerInterface
)