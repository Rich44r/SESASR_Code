import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


LINEAR_VELOCITY = 0.20  # m/s
ANGULAR_VELOCITY = 1.5  # rad/s
MINIMUM_DISTANCE = 0.5  # m
RANGE_MAX = 3.5  # m

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
        self.subscriber_ground = self.create_subscription(
            Odometry,
            'ground_truth',
            self.ground_callback,
            10)
        self.subscriber_ground
        self.get_logger().info('Controller node has been started.')

    def laser_callback(self, msg):
        distance_front = []
        distance_left = []
        distance_right = []
        send = Twist()

        #front interval of 60 degrees
        for i in range(0,30):
            if not math.isinf(msg.ranges[i]):
                distance_front.append(msg.ranges[i])
            else:
                distance_front.append(RANGE_MAX)  #assume no obstacle in that direction
        for i in range(330,360):
            if not math.isinf(msg.ranges[i]):
                distance_front.append(msg.ranges[i])
            else:
                distance_front.append(RANGE_MAX)  #assume no obstacle in that direction

        if min(distance_front)>MINIMUM_DISTANCE:
            #self.get_logger().info('moving forward')
            send.linear.x = LINEAR_VELOCITY
            distance_front.clear()
        else:
            send.linear.x = 0.0
            #decide to turn left or right
            for i in range(30,90):
                if not math.isinf(msg.ranges[i]):
                    distance_left.append(msg.ranges[i])
                else:
                    distance_left.append(RANGE_MAX)  #assume no obstacle in that direction
            for i in range(270,330):
                if not math.isinf(msg.ranges[i]):
                    distance_right.append(msg.ranges[i])
                else:
                    distance_right.append(RANGE_MAX)  #assume no obstacle in that direction
            if max(distance_left)>max(distance_right):
                #self.get_logger().info('turning right')
                send.angular.z = ANGULAR_VELOCITY
            else:
                send.angular.z = -ANGULAR_VELOCITY
                #self.get_logger().info('turning left')
                distance_left.clear()
                distance_right.clear()

        self.publisher.publish(send)
                

        

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.get_logger().info(
            f'Position -> x: {position.x:.2f}, y: {position.y:.2f}, Yaw: {yaw:.2f} rad')
        
    def ground_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.get_logger().info(
            f'Ground Truth -> x: {position.x:.2f}, y: {position.y:.2f}, Yaw: {yaw:.2f} rad')


        
        

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()