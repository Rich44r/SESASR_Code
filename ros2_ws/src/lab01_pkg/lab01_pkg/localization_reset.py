import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Pose2D


class localization(Node):
    def __init__(self):
        super().__init__('localization_reset')
        
        self.X = 0 #posizione X del robot
        self.Y = 0 #posizione Y del robot

        self.subscription_cmd_vel = self.create_subscription(Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription_reset = self.create_subscription(Bool,
            'reset',
            self.callback_reset,
            10)
        self.subscription_cmd_vel  # prevent unused variable warning
        self.subscription_reset  
        self.publisher_ = self.create_publisher(Pose2D, 'pose', 10)
        self.get_logger().info('Localization node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Twist: linear.x={msg.linear.x}, linear.y={msg.linear.y}')
        self.X += msg.linear.x
        self.Y += msg.linear.y
        
        self.get_logger().info(f'Position: X={self.X}, Y={self.Y}')
        self.publisher_.publish(Pose2D(x=self.X, y=self.Y))

    def callback_reset(self, msg):
        if msg.data:#if a reset msg is received
            self.X = 0.0
            self.Y = 0.0
            self.publisher_.publish(Pose2D(x=self.X, y=self.Y))
            self.get_logger().info('Reset message received: POSITION RESET TO (0,0)')
        


def main(args=None):
    rclpy.init(args=args)

    localization_node = localization()

    rclpy.spin(localization_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()