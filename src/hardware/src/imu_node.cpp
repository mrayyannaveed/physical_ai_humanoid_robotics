#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class IMUNode : public rclcpp::Node
{
public:
  IMUNode() : Node("imu_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&IMUNode::publish_imu_data, this));
    RCLCPP_INFO(this->get_logger(), "IMU Node Started");
  }

private:
  void publish_imu_data()
  {
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->now();
    message.header.frame_id = "imu_link";
    // Populate with dummy data for now
    message.orientation.w = 1.0;
    publisher_->publish(message);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
