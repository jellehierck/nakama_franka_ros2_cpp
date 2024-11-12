#include "nakama_franka_ros2_controllers/move_to_pose_controller.hpp"

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp_action/server.hpp>

#include <controller_interface/controller_interface_base.hpp>

#include "nakama_franka_ros2_controllers/motion_generator.hpp"

controller_interface::InterfaceConfiguration
nakama_franka_controllers::MoveToPoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
nakama_franka_controllers::MoveToPoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 1; i <= N_JOINTS; i++) {
    config.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    config.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn nakama_franka_controllers::MoveToPoseController::on_init() {
  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception &e) {
    (void)fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::MoveToPoseController::on_configure(  // NOLINT(readability-function-cognitive-complexity)
  const rclcpp_lifecycle::State &                               /*previous_state*/
) {
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

  // Set up move to pose action server
  move_to_pose_action_server_ = rclcpp_action::create_server<FrankaMoveToPose>(
    get_node(), "~/move_to_pose",
    std::bind(&MoveToPoseController::handle_new_move_to_pose_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MoveToPoseController::handle_move_to_pose_cancel, this, std::placeholders::_1),
    std::bind(&MoveToPoseController::handle_move_to_pose_accepted, this, std::placeholders::_1)
  );

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::MoveToPoseController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::MoveToPoseController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Reset joint states to initialize the arrays
  reset_joint_states();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
nakama_franka_controllers::MoveToPoseController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void nakama_franka_controllers::MoveToPoseController::reset_joint_states() {
  joint_positions_curr_.setZero();
  joint_velocities_curr_.setZero();
  joint_velocities_curr_filtered_.setZero();
}

void nakama_franka_controllers::MoveToPoseController::update_curr_joint_states() {
  for (size_t i = 0; i < N_JOINTS; i++) {
    const auto &position_interface = state_interfaces_.at(2 * i);
    const auto &velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    joint_positions_curr_(static_cast<int>(i)) = position_interface.get_value();
    joint_velocities_curr_(static_cast<int>(i)) = velocity_interface.get_value();
  }
}

controller_interface::return_type nakama_franka_controllers::MoveToPoseController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
) {
  // Get the joint positions and velocities
  update_curr_joint_states();

  // Initialize the joint torque command outputs
  Vector7d joint_torques_command;

  // Check if currently not executing a move to pose action. If so, send zero torques to the joints and exit early
  if (move_to_pose_goal_handle_ == nullptr || !move_to_pose_goal_handle_->is_executing()) {
    joint_torques_command.setZero();
    for (size_t i = 0; i < N_JOINTS; i++) {
      command_interfaces_[i].set_value(joint_torques_command(static_cast<int>(i)));
    }
    return controller_interface::return_type::OK;
  }

  // We are currently executing the move to pose action

  // Calculate the next desired joint position
  auto     trajectory_time = this->get_node()->now() - start_time_;
  auto     motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  Vector7d joint_positions_desired = motion_generator_output.first;
  bool     motion_finished = motion_generator_output.second;

  // Filter the joint velocities
  joint_velocities_curr_filtered_ = (1 - VELOCITY_FILTER_ALPHA_GAIN) * joint_velocities_curr_filtered_ +
                                    VELOCITY_FILTER_ALPHA_GAIN * joint_velocities_curr_;

  // Calculate the torque command and set the command interface
  joint_torques_command = k_gains_.cwiseProduct(joint_positions_desired - joint_positions_curr_) +
                          d_gains_.cwiseProduct(-joint_velocities_curr_filtered_);
  for (size_t i = 0; i < N_JOINTS; i++) {
    command_interfaces_[i].set_value(joint_torques_command(static_cast<int>(i)));
  }

  // Update update action feedback
  move_to_pose_feedback_->curr_joint_positions = vector_to_array(joint_positions_curr_);
  // TODO: Make publish real-time friendly (but that is quite hard and there are little examples)
  move_to_pose_goal_handle_->publish_feedback(move_to_pose_feedback_);

  // Check if the motion is finished
  if (motion_finished) {
    // The motion is finished, stop the action server and clean up the managed objects
    move_to_pose_result_->final_joint_positions = vector_to_array(joint_positions_curr_);
    move_to_pose_goal_handle_->succeed(move_to_pose_result_);
    move_to_pose_goal_handle_ = nullptr;
    RCLCPP_INFO(get_node()->get_logger(), "Goal pose reached!");
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse nakama_franka_controllers::MoveToPoseController::handle_new_move_to_pose_goal(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const FrankaMoveToPose::Goal> goal
) {
  // Check if a motion is currently running
  if (motion_started_.load(std::memory_order_relaxed)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot move to a new pose, another motion is running!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check if the speed factor is valid
  if (!(goal->speed_factor > FrankaMoveToPose::Goal::SPEED_FACTOR_MIN ||
        goal->speed_factor <= FrankaMoveToPose::Goal::SPEED_FACTOR_MAX)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Cannot move to a new pose, the speed factor %f is outside the allowed range (%f, %f]",
      goal->speed_factor, FrankaMoveToPose::Goal::SPEED_FACTOR_MIN, FrankaMoveToPose::Goal::SPEED_FACTOR_MAX
    );
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "New pose accepted!");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void nakama_franka_controllers::MoveToPoseController::handle_move_to_pose_accepted(
  std::shared_ptr<FrankaMoveToPoseGoalHandle> goal_handle
) {
  // Create the motion generated with the desired goal position and speed
  Vector7d joint_positions_goal = array_to_vector(goal_handle->get_goal()->goal_joint_positions);
  motion_generator_ = std::make_unique<MotionGenerator>(
    goal_handle->get_goal()->speed_factor, joint_positions_curr_, joint_positions_goal
  );

  // Initialize variables needed for motion generation and control
  start_time_ = this->get_node()->now();
  joint_velocities_curr_filtered_.setZero();

  // Store the new goal handle
  move_to_pose_goal_handle_ = goal_handle;

  // Reset the feedback and result messages which are sent by the action server
  move_to_pose_feedback_ = std::make_shared<FrankaMoveToPose::Feedback>();
  move_to_pose_result_ = std::make_shared<FrankaMoveToPose::Result>();

  // Indicate the motion is started
  motion_started_.store(true);
  RCLCPP_INFO(get_node()->get_logger(), "Starting move to pose...");
}

rclcpp_action::CancelResponse nakama_franka_controllers::MoveToPoseController::handle_move_to_pose_cancel(
  std::shared_ptr<FrankaMoveToPoseGoalHandle> /*goal_handle*/
) {
  // We cannot cancel a moving command as this will most likely make the robot go into a Reflex error
  return rclcpp_action::CancelResponse::REJECT;
}

// Export the controller as a pluginlib plugin

#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nakama_franka_controllers::MoveToPoseController, controller_interface::ControllerInterface)