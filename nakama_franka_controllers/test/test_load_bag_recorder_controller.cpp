#include <memory>

#include <gtest/gtest.h>

#include <rclcpp/executor.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

TEST(TestLoadControllers, test_load_bag_recorder_controller) {
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager controller_manager(
    std::make_unique<hardware_interface::ResourceManager>(ros2_control_test_assets::minimal_robot_urdf), executor,
    "test_controller_manager"
  );

  auto response = controller_manager.load_controller(
    "test_bag_recorder_controller", "nakama_franka_controllers/BagRecorderController"
  );

  ASSERT_NE(response, nullptr);

  rclcpp::shutdown();
}
