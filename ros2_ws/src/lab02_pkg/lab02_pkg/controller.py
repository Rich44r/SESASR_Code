import rclpy
import tf_transformations
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


LINEAR_VELOCITY = 0.20  # m/s
ANGULAR_VELOCITY = 1.5  # rad/s

class ControllerNode(Node):
    def __init__(self):
        super().__init__('Controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription_scan  
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription_odom  
        self.get_logger().info('Controller node has been started.')

    def laser_callback(self, msg):

        pass
        

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()