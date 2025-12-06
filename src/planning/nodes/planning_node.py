import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # Placeholder for control output

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_cmd_vel)
        self.get_logger().info('Planning Node Started')

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.1 # Placeholder for policy decision
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
