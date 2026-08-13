import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
