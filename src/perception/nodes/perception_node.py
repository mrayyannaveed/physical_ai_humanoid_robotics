import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Placeholder for perception input

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/rgb/image_raw',
            self.listener_callback,
            10)
        self.get_logger().info('Perception Node Started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image data (header: {msg.header.stamp})')
        # Placeholder for AI inference
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
