#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/rgb/image_raw", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CameraNode::publish_camera_data, this));
    RCLCPP_INFO(this->get_logger(), "Camera Node Started");
  }

private:
  void publish_camera_data()
  {
    auto message = sensor_msgs::msg::Image();
    message.header.stamp = this->now();
    message.header.frame_id = "camera_link";
    message.height = 480;
    message.width = 640;
    message.encoding = "rgb8";
    message.is_bigendian = 0;
    message.step = message.width * 3; // 3 bytes per pixel for rgb8
    message.data.resize(message.height * message.step);
    // Populate with dummy data for now
    publisher_->publish(message);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
