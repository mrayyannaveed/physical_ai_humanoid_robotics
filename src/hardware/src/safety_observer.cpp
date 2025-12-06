#include "rclcpp/rclcpp.hpp"
#include "common/contracts/ContactState.msg" // Placeholder for custom message
#include "std_msgs/msg/bool.hpp" // For safety status

class SafetyObserver : public rclcpp::Node
{
public:
  SafetyObserver() : Node("safety_observer")
  {
    safety_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>("safety_status", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&SafetyObserver::publish_safety_status, this));
    RCLCPP_INFO(this->get_logger(), "Safety Observer Node Started");
  }

private:
  void publish_safety_status()
  {
    auto message = std_msgs::msg::Bool();
    message.data = true; // Placeholder: Assume safe
    safety_status_publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyObserver>());
  rclcpp::shutdown();
  return 0;
}
