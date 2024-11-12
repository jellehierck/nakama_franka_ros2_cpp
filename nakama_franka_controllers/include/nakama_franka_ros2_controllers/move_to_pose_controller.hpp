#ifndef NAKAMA_FRANKA_ROS2_CONTROLLERS_MOVE_TO_POSE_IMPEDANCE_CONTROLLER
#define NAKAMA_FRANKA_ROS2_CONTROLLERS_MOVE_TO_POSE_IMPEDANCE_CONTROLLER

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <memory>

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_action/server.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>

#include <nakama_franka_interfaces/action/franka_move_to_pose.hpp>

#include "nakama_franka_ros2_controllers/motion_generator.hpp"

namespace nakama_franka_controllers {

/**
 * Controls the robot to follow a path stored inside a rosbag, using joint impedance control.
 */
class MoveToPoseController : public controller_interface::ControllerInterface {
 public:
  static constexpr const size_t N_JOINTS = 7;
  using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

  // TODO: Make configurable?
  static constexpr const double VELOCITY_FILTER_ALPHA_GAIN = 0.99;

  /**
   * Specify which hardware command interfaces are used.
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * Specify which hardware state interfaces are used.
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * Initialize all parameters.
   */
  CallbackReturn on_init() override;

  /**
   * TODO:
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * TODO:
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * TODO:
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * TODO:
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * Perform the controller update step.
   *
   * @param time Current time.
   * @param period Time since the last controller update step.
   */
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  using FrankaMoveToPose = nakama_franka_interfaces::action::FrankaMoveToPose;
  using FrankaMoveToPoseGoalHandle = rclcpp_action::ServerGoalHandle<FrankaMoveToPose>;

  /**
   * Read and store the current joint states from the state interfaces.
   */
  void update_curr_joint_states();

  /**
   * Reset the stored joint states to zero.
   */
  void reset_joint_states();

  rclcpp_action::GoalResponse handle_new_move_to_pose_goal(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FrankaMoveToPose::Goal> goal
  );

  rclcpp_action::CancelResponse handle_move_to_pose_cancel(std::shared_ptr<FrankaMoveToPoseGoalHandle> goal_handle);

  void handle_move_to_pose_accepted(std::shared_ptr<FrankaMoveToPoseGoalHandle> goal_handle);

  /**
   * Helper function to map an array of joint values to a vector.
   * @param arr Array of joint values.
   * @return `Vector7d` A vector with the exact values in the array.
   */
  static Vector7d array_to_vector(std::array<double, N_JOINTS> arr) {
    Vector7d vec;
    for (size_t i = 0; i < N_JOINTS; i++) {
      vec(static_cast<int>(i)) = arr.at(i);
    }
    return vec;
  }

  /**
   * Helper function to map a vector of joint values to an array.
   * @param vec Vector of joint valules.
   * @return `std::array<double, N_JOINTS>` An arrat with the exact values in the vector.
   */
  static std::array<double, N_JOINTS> vector_to_array(Vector7d vec) {
    std::array<double, N_JOINTS> arr;
    for (size_t i = 0; i < N_JOINTS; i++) {
      arr[i] = vec(static_cast<int>(i));
    }
    return arr;
  }

  rclcpp_action::Server<FrankaMoveToPose>::SharedPtr move_to_pose_action_server_;
  std::shared_ptr<FrankaMoveToPoseGoalHandle>        move_to_pose_goal_handle_;
  std::shared_ptr<FrankaMoveToPose::Feedback>        move_to_pose_feedback_;
  std::shared_ptr<FrankaMoveToPose::Result>          move_to_pose_result_;

  std::unique_ptr<MotionGenerator> motion_generator_;
  rclcpp::Time                     start_time_;
  std::atomic_bool                 motion_started_{false};

  Vector7d joint_positions_curr_;
  Vector7d joint_velocities_curr_;
  Vector7d joint_velocities_curr_filtered_;

  Vector7d k_gains_;
  Vector7d d_gains_;
};

}  // namespace nakama_franka_controllers

#endif  // NAKAMA_FRANKA_ROS2_CONTROLLERS_MOVE_TO_POSE_IMPEDANCE_CONTROLLER
