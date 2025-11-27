#ifndef WALKER_LOGIC_HPP
#define WALKER_LOGIC_HPP

#include "geometry_msgs/msg/twist.hpp"
#include <memory>

namespace walker {

// Forward declaration for WalkerLogic (needed by states)
class WalkerLogic;

/**
 * @brief Abstract base class for walker states (State Pattern)
 * 
 * Each concrete state handles its own behavior and can transition to other states.
 * Based on the State design pattern from Game Programming Patterns.
 */
class WalkerState {
public:
  virtual ~WalkerState() = default;

  /**
   * @brief Handle sensor input and compute velocity command
   * @param context The walker logic context
   * @param left_sensor_value Left sensor range reading
   * @param right_sensor_value Right sensor range reading
   * @return Twist message with linear and angular velocities
   */
  virtual geometry_msgs::msg::Twist handleInput(
      WalkerLogic& context,
      double left_sensor_value,
      double right_sensor_value) = 0;

  /**
   * @brief Get the name of the state (for debugging/testing)
   */
  virtual const char* getName() const = 0;
};

/**
 * @brief State for moving forward (no obstacle detected)
 * 
 * In this state, the robot moves forward. If an obstacle is detected,
 * it transitions to RotatingState.
 */
class MovingForwardState : public WalkerState {
public:
  explicit MovingForwardState(double forward_speed, double max_range)
      : forward_speed_(forward_speed), max_range_(max_range) {
  }

  geometry_msgs::msg::Twist handleInput(
      WalkerLogic& context,
      double left_sensor_value,
      double right_sensor_value) override;

  const char* getName() const override { return "MovingForward"; }

private:
  double forward_speed_;
  double max_range_;
  
  bool isObstacleDetected(double left_sensor_value, double right_sensor_value) const {
    const double threshold = 0.9 * max_range_;
    return (left_sensor_value < threshold || right_sensor_value < threshold);
  }
};

/**
 * @brief State for rotating in place (obstacle detected)
 * 
 * In this state, the robot rotates in place. The rotation direction
 * alternates between clockwise and counterclockwise. When the path
 * is clear, it transitions back to MovingForwardState.
 */
class RotatingState : public WalkerState {
public:
  explicit RotatingState(double rotation_speed, double max_range, bool turn_right)
      : rotation_speed_(rotation_speed), max_range_(max_range), turn_right_(turn_right) {
  }

  geometry_msgs::msg::Twist handleInput(
      WalkerLogic& context,
      double left_sensor_value,
      double right_sensor_value) override;

  const char* getName() const override { return "Rotating"; }

  /**
   * @brief Get the current rotation direction
   * @return true if rotating clockwise (right), false for counterclockwise (left)
   */
  bool isTurningRight() const { return turn_right_; }

  /**
   * @brief Toggle rotation direction for next rotation
   */
  void toggleDirection() { turn_right_ = !turn_right_; }

private:
  double rotation_speed_;
  double max_range_;
  bool turn_right_;  // true = clockwise (right), false = counterclockwise (left)
  
  bool isPathClear(double left_sensor_value, double right_sensor_value) const {
    const double threshold = 0.9 * max_range_;
    return (left_sensor_value >= threshold && right_sensor_value >= threshold);
  }
};

/**
 * @brief Context class for the State pattern (testable without ROS2)
 * 
 * This class implements the Roomba-like walker algorithm using the State design pattern.
 * It maintains a reference to the current state and delegates behavior to it.
 * Based on the State pattern from Game Programming Patterns.
 */
class WalkerLogic {
public:
  /**
   * @brief Constructor
   * @param max_range Maximum sensor range for obstacle detection (default: 0.15)
   * @param forward_speed Forward velocity when no obstacle (default: 0.1)
   * @param rotation_speed Angular velocity when rotating (default: 2.0)
   */
  explicit WalkerLogic(double max_range = 0.15, 
                       double forward_speed = 0.1, 
                       double rotation_speed = 2.0);

  /**
   * @brief Destructor
   */
  ~WalkerLogic();

  /**
   * @brief Compute velocity command based on sensor readings
   * Delegates to the current state.
   * @param left_sensor_value Left sensor range reading
   * @param right_sensor_value Right sensor range reading
   * @return Twist message with linear and angular velocities
   */
  geometry_msgs::msg::Twist computeVelocityCommand(double left_sensor_value, 
                                                    double right_sensor_value);

  /**
   * @brief Change the current state (called by states during transitions)
   * @param new_state Pointer to the new state (must be one of our owned states)
   */
  void changeState(WalkerState* new_state);

  /**
   * @brief Get the current state name (for testing/debugging)
   */
  const char* getCurrentStateName() const;

  /**
   * @brief Get maximum sensor range threshold
   */
  double getMaxRange() const { return max_range_; }

  /**
   * @brief Get forward speed
   */
  double getForwardSpeed() const { return forward_speed_; }

  /**
   * @brief Get rotation speed
   */
  double getRotationSpeed() const { return rotation_speed_; }

  /**
   * @brief Get the rotating state (for testing)
   */
  RotatingState* getRotatingState() { return rotating_state_.get(); }

  /**
   * @brief Get the moving forward state (for testing)
   */
  MovingForwardState* getMovingForwardState() { return moving_forward_state_.get(); }

private:
  friend class MovingForwardState;
  friend class RotatingState;

  double max_range_;
  double forward_speed_;
  double rotation_speed_;
  
  // State instances (owned by WalkerLogic)
  std::unique_ptr<MovingForwardState> moving_forward_state_;
  std::unique_ptr<RotatingState> rotating_state_;
  
  // Current state pointer (State pattern - points to one of the above states)
  WalkerState* current_state_;
  
  // Track rotation direction for alternating
  bool turn_right_next_ = true;
  
  /**
   * @brief Get the next rotation direction and toggle it
   */
  bool getAndToggleRotationDirection();
};

} // namespace walker

#endif // WALKER_LOGIC_HPP
