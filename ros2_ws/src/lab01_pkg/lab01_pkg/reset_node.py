import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

class ResetNode(Node):
    def __init__(self):
        super().__init__('ResetNode')
        self.subscription = self.create_subscription(
            Pose2D,
            'pose',
            self.listener_callback,
            10)
        self.counterx = 0
        self.countery = 0
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Bool, 'reset', 10)
        self.get_logger().info('ResetNode has been started.')

    def listener_callback(self, msg):
        # Counts the position and suggests resets if out of bounds
        self.get_logger().info(f'Received pose: {msg}')
        self.counterx += msg.x
        self.countery += msg.y
        if self.counterx > 6 or self.countery > 6:
            self.get_logger().info('Position out of bounds, RESET to (0,0)')
            self.counterx = 0 #because the position will be reset to 0
            self.countery = 0
            self.publisher.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)

    reset_node = ResetNode()

    rclpy.spin(reset_node)

    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()