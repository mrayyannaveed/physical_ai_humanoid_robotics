#include "rclcpp/rclcpp.hpp"
#include "common/contracts/ContactState.msg" // Placeholder for custom message

class TorqueController : public rclcpp::Node
{
public:
  TorqueController() : Node("torque_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Torque Controller Node Started");
  }

private:
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TorqueController>());
  rclcpp::shutdown();
  return 0;
}
