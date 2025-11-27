#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "geometry_msgs/msg/twist.hpp"
#include <utility>

namespace walker_robot {

/**
 * @brief Motor control logic for differential drive robot (testable without Webots)
 * 
 * Converts Twist commands (linear/angular velocities) to individual wheel velocities
 * for a differential drive robot.
 */
class MotorController {
public:
  /**
   * @brief Constructor
   * @param half_distance_between_wheels Half the distance between wheels in meters (default: 0.045)
   * @param wheel_radius Wheel radius in meters (default: 0.025)
   */
  explicit MotorController(double half_distance_between_wheels = 0.045,
                          double wheel_radius = 0.025);

  /**
   * @brief Compute wheel velocities from Twist command
   * @param twist Twist message with linear.x (forward speed) and angular.z (angular speed)
   * @return Pair of (left_wheel_velocity, right_wheel_velocity) in rad/s
   */
  std::pair<double, double> computeWheelVelocities(const geometry_msgs::msg::Twist& twist) const;

  /**
   * @brief Compute wheel velocities from individual components
   * @param forward_speed Linear forward velocity in m/s
   * @param angular_speed Angular velocity in rad/s
   * @return Pair of (left_wheel_velocity, right_wheel_velocity) in rad/s
   */
  std::pair<double, double> computeWheelVelocities(double forward_speed, double angular_speed) const;

  /**
   * @brief Get half distance between wheels
   */
  double getHalfDistanceBetweenWheels() const { return half_distance_between_wheels_; }

  /**
   * @brief Get wheel radius
   */
  double getWheelRadius() const { return wheel_radius_; }

private:
  double half_distance_between_wheels_;
  double wheel_radius_;
};

// Inline implementations
inline MotorController::MotorController(double half_distance_between_wheels, double wheel_radius)
    : half_distance_between_wheels_(half_distance_between_wheels),
      wheel_radius_(wheel_radius) {
}

inline std::pair<double, double> MotorController::computeWheelVelocities(
    const geometry_msgs::msg::Twist& twist) const {
  return computeWheelVelocities(twist.linear.x, twist.angular.z);
}

inline std::pair<double, double> MotorController::computeWheelVelocities(
    double forward_speed, double angular_speed) const {
  double left_velocity = (forward_speed - angular_speed * half_distance_between_wheels_) / wheel_radius_;
  double right_velocity = (forward_speed + angular_speed * half_distance_between_wheels_) / wheel_radius_;
  return std::make_pair(left_velocity, right_velocity);
}

} // namespace walker_robot

#endif // MOTOR_CONTROLLER_HPP

