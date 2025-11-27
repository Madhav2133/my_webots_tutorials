#include "walker/walker_logic.hpp"

// Implement state methods here to avoid circular dependency
namespace walker {

geometry_msgs::msg::Twist MovingForwardState::handleInput(
    WalkerLogic& context,
    double left_sensor_value,
    double right_sensor_value) {
  
  geometry_msgs::msg::Twist command;
  
  if (isObstacleDetected(left_sensor_value, right_sensor_value)) {
    // Obstacle detected - transition to rotating state
    bool turn_right = context.getAndToggleRotationDirection();
    auto* rotating_state = context.getRotatingState();
    rotating_state->toggleDirection();
    if (rotating_state->isTurningRight() != turn_right) {
      rotating_state->toggleDirection();  // Ensure correct direction
    }
    context.changeState(rotating_state);
    
    // Return rotation command
    command.linear.x = 0.0;
    command.angular.z = turn_right ? -context.getRotationSpeed() : context.getRotationSpeed();
  } else {
    // No obstacle - continue moving forward
    command.linear.x = forward_speed_;
    command.angular.z = 0.0;
  }
  
  return command;
}

geometry_msgs::msg::Twist RotatingState::handleInput(
    WalkerLogic& context,
    double left_sensor_value,
    double right_sensor_value) {
  
  geometry_msgs::msg::Twist command;
  
  if (isPathClear(left_sensor_value, right_sensor_value)) {
    // Path is clear - transition back to moving forward state
    context.changeState(context.getMovingForwardState());
    
    // Return forward command
    command.linear.x = context.getForwardSpeed();
    command.angular.z = 0.0;
  } else {
    // Still blocked - continue rotating
    command.linear.x = 0.0;
    command.angular.z = turn_right_ ? -rotation_speed_ : rotation_speed_;
  }
  
  return command;
}

}

namespace walker {

WalkerLogic::WalkerLogic(double max_range, double forward_speed, double rotation_speed)
    : max_range_(max_range), 
      forward_speed_(forward_speed), 
      rotation_speed_(rotation_speed),
      current_state_(nullptr),
      turn_right_next_(true) {
  
  // Create state instances (reused during transitions)
  moving_forward_state_ = std::make_unique<MovingForwardState>(forward_speed_, max_range_);
  rotating_state_ = std::make_unique<RotatingState>(rotation_speed_, max_range_, turn_right_next_);
  
  // Start in moving forward state
  current_state_ = moving_forward_state_.get();
}

WalkerLogic::~WalkerLogic() = default;

geometry_msgs::msg::Twist WalkerLogic::computeVelocityCommand(
    double left_sensor_value, 
    double right_sensor_value) {
  
  // Delegate to current state
  return current_state_->handleInput(*this, left_sensor_value, right_sensor_value);
}

void WalkerLogic::changeState(WalkerState* new_state) {
  current_state_ = new_state;
}

const char* WalkerLogic::getCurrentStateName() const {
  return current_state_ ? current_state_->getName() : "Unknown";
}

bool WalkerLogic::getAndToggleRotationDirection() {
  bool current = turn_right_next_;
  turn_right_next_ = !turn_right_next_;
  return current;
}

} // namespace walker

