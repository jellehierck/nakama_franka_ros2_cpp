#include <gtest/gtest.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadLocalJointPositionController, load_controller) {
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(std::make_unique<hardware_interface::ResourceManager>(
                                               ros2_control_test_assets::minimal_robot_urdf),
                                           executor, "test_controller_manager");

  auto response =
      cm.load_controller("test_local_joint_position_controller",
                         "nakama_franka_controllers/LocalJointPositionController");

  ASSERT_NE(response, nullptr);

  rclcpp::shutdown();
}
