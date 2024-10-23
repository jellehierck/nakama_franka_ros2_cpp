#include <cstdio>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

class FrankaStateBagRecorder : public rclcpp::Node {

public:
  FrankaStateBagRecorder() : Node("franka_state_bag_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // TODO: Trigger opening the bag with a service
    // TODO: Make the bag name a parameter / part of the service
    writer_->open("my_bag");

    // TODO: Make the input subscription a parameter or use remaps
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/franka_robot_state_broadcaster/desired_joint_states",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&FrankaStateBagRecorder::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  void topic_callback(std::shared_ptr<sensor_msgs::msg::JointState> msg) const {
    rclcpp::Time time_stamp = this->now();

    // TODO: Make the stored topic name a parameter
    writer_->write<sensor_msgs::msg::JointState>(*msg, "/joint_state_commands",
                                                 msg->header.stamp);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrankaStateBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
