import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


LINEAR_VELOCITY = 0.20  # m/s
ANGULAR_VELOCITY = 1.5  # rad/s
MINIMUM_DISTANCE = 0.5  # m
RANGE_MAX = 3.5  # m

class ControllerNode(Node):
    def __init__(self):
        super().__init__('Controller')
        
        self.state = "forward" #go forward by default
        self.start_yaw = None  # To track the starting yaw for rotations

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data)
        self.subscription_scan  
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data)
        self.subscription_odom  
        self.subscriber_ground = self.create_subscription(
            Odometry,
            'ground_truth',
            self.ground_callback,
            qos_profile_sensor_data)
        self.subscriber_ground
        self.get_logger().info('Controller node has been started.')


    def laser_callback(self, msg):
        self.get_logger().info(f'dimensione : {len(msg.ranges)}')
        distance_front = []
        distance_left = []
        distance_right = []
        send = Twist()
        #list of real angles (angles avrà la stessa dimensione di ranges)
        angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
        #interval definitions
        min1_front, max1_front = math.radians(0), math.radians(30)
        min2_front, max2_front = math.radians(330), math.radians(360)
        min_left, max_left = math.radians(30), math.radians(90)
        min_right, max_right = math.radians(270), math.radians(330)

        #d : distance, a : angle
        distance_front = [d if not math.isinf(d) else RANGE_MAX
                          for d,a in zip(msg.ranges, angles) if min1_front<=a<= max1_front or min2_front<=a<=max2_front]
        distance_left = [d if not math.isinf(d) else RANGE_MAX
                         for d,a in zip(msg.ranges, angles) if min_left <= a<= max_left]
        distance_right = [d if not math.isinf(d) else RANGE_MAX
                         for d,a in zip(msg.ranges, angles) if min_right <= a<= max_right]
        
        if min(distance_front)>MINIMUM_DISTANCE:
            #self.get_logger().info('moving forward')
            self.state = "forward"
            distance_front.clear()
        else:
            send.linear.x = 0.0
            #decide if turn left or right
            if max(distance_left)>max(distance_right):
                self.state = "left"
                distance_left.clear()
            else:
                self.state = "right"
                distance_right.clear()

        self.publisher.publish(send)
                

        

    def odom_callback(self, msg):
        send = Twist()
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
            f'Odom -> x: {position.x:.2f}, y: {position.y:.2f}, Yaw: {yaw:.2f} rad')
        
        if self.state == "forward":
            send.linear.x = LINEAR_VELOCITY

        if self.state == "left":
            # Save the starting yaw the first time
            if self.start_yaw is None:
                self.start_yaw = yaw
            # Calculate the angle difference
            diff = self.normalize_angle(yaw - self.start_yaw)
            if abs(diff) < math.pi/2 - 0.16:
                send.angular.z = ANGULAR_VELOCITY
                self.publisher.publish(send)
            else:
                self.state = "forward"
                send.angular.z = 0.0
                self.publisher.publish(send)
                self.start_yaw = None  # Reset for the next rotation

        elif self.state == "right":
            # Save the starting yaw the first time
            if self.start_yaw is None:
                self.start_yaw = yaw
            # Calculate the angle difference
            diff = self.normalize_angle(yaw - self.start_yaw)
            if abs(diff) < math.pi/2 - 0.16:
                send.angular.z = -ANGULAR_VELOCITY
                self.publisher.publish(send)
            else:
                self.state = "forward"
                send.angular.z = 0.0
                self.publisher.publish(send)
                self.start_yaw = None  # Reset for the next rotation


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


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle



def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()