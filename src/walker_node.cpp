#include "walker/walker_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

namespace walker_robot {
void walker_robot::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&walker_robot::cmdVelCallback, this, std::placeholders::_1));
}

void walker_robot::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void walker_robot::step() {
  // Use MotorController to compute wheel velocities
  auto [left_velocity, right_velocity] = 
      motor_controller_.computeWheelVelocities(cmd_vel_msg);

  wb_motor_set_velocity(left_motor, left_velocity);
  wb_motor_set_velocity(right_motor, right_velocity);
}
} // namespace walker_robot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(walker_robot::walker_robot,
                       webots_ros2_driver::PluginInterface)
