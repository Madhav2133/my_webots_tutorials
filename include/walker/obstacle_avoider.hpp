#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "walker/walker_logic.hpp"

namespace walker {

/**
 * @brief ROS2 node that implements obstacle avoidance using WalkerLogic
 */
class ObstacleAvoider : public rclcpp::Node {
public:
  explicit ObstacleAvoider();

  /**
   * @brief Get the walker logic instance (for testing)
   * @return Reference to the walker logic
   */
  WalkerLogic& getWalkerLogic() { return walker_logic_; }

private:
  void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void processSensorData();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

  WalkerLogic walker_logic_;
  double left_sensor_value_{0.0};
  double right_sensor_value_{0.0};
};

} // namespace walker
