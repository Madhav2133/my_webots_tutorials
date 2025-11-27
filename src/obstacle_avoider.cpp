#include "walker/obstacle_avoider.hpp"

namespace walker {

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      std::bind(&ObstacleAvoider::leftSensorCallback, this,
                std::placeholders::_1));

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      std::bind(&ObstacleAvoider::rightSensorCallback, this,
                std::placeholders::_1));
}

void ObstacleAvoider::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value_ = msg->range;
  processSensorData();
}

void ObstacleAvoider::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value_ = msg->range;
  processSensorData();
}

void ObstacleAvoider::processSensorData() {
  // Use WalkerLogic to compute the command (delegates to current state)
  auto command = walker_logic_.computeVelocityCommand(
      left_sensor_value_, right_sensor_value_);

  // Log state changes for debugging
  static const char* last_state = "";
  const char* current_state = walker_logic_.getCurrentStateName();
  if (current_state != last_state) {
    RCLCPP_INFO(this->get_logger(), "State: %s", current_state);
    last_state = current_state;
  }

  // Publish the command
  auto command_message = std::make_unique<geometry_msgs::msg::Twist>(command);
  publisher_->publish(std::move(command_message));
}

} // namespace walker

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<walker::ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}
