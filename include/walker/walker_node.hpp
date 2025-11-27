#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "walker/motor_controller.hpp"

namespace walker_robot {
class walker_robot : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

  /**
   * @brief Get the motor controller instance (for testing)
   * @return Reference to the motor controller
   */
  MotorController& getMotorController() { return motor_controller_; }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  MotorController motor_controller_;
  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
};
} // namespace walker_robot
#endif
