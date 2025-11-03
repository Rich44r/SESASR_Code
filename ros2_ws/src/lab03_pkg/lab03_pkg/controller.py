import rclpy
import tf_transformations
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


LINEAR_VELOCITY = 0.20  # m/s
ANGULAR_VELOCITY = 0.4     # rad/s
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

    def get_index(self, target_angle, angle_increment):
        target_rad = math.radians(target_angle)
        index = target_rad/angle_increment
        return int(index)


    def laser_callback(self, msg):
        #self.get_logger().info(f'dimensione : {len(msg.ranges)}')
        distance_front = []
        distance_left = []
        distance_right = []
        send = Twist()

        #calculate indices for front region
        min1_front = self.get_index(0,msg.angle_increment)
        max1_front = self.get_index(30,msg.angle_increment)
        min2_front = self.get_index(330,msg.angle_increment)
        max2_front = len(msg.ranges)-1 #last index
        #front interval of 60 degrees
        for i in range(min1_front,max1_front):
            if not math.isinf(msg.ranges[i]):
                distance_front.append(msg.ranges[i])
            else:
                distance_front.append(RANGE_MAX)  #assume no obstacle in that direction
        for i in range(min2_front,max2_front):
            if not math.isinf(msg.ranges[i]):
                distance_front.append(msg.ranges[i])
            else:
                distance_front.append(RANGE_MAX)  #assume no obstacle in that direction

        
        
        if min(distance_front)>MINIMUM_DISTANCE and self.state!="turning":
            #self.get_logger().info('moving forward')
            self.state = "forward"
            send.linear.x = LINEAR_VELOCITY
            distance_front.clear()
        else:
            send.linear.x = 0.0
            #calculate left and right indices
            min_left = self.get_index(30,msg.angle_increment)
            max_left = self.get_index(90,msg.angle_increment)
            min_right = self.get_index(270,msg.angle_increment)
            max_right = self.get_index(330,msg.angle_increment)

            #decide to turn left or right
            for i in range(min_left,max_left):
                if not math.isinf(msg.ranges[i]):
                    distance_left.append(msg.ranges[i])
                else:
                    distance_left.append(RANGE_MAX)  #assume no obstacle in that direction
            for i in range(min_right,max_right):
                if not math.isinf(msg.ranges[i]):
                    distance_right.append(msg.ranges[i])
                else:
                    distance_right.append(RANGE_MAX)  #assume no obstacle in that direction
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
        

            

        if self.state == "left":
            # Save the starting yaw the first time
            if self.start_yaw is None:
                self.start_yaw = yaw
            # Calculate the angle difference
            diff = self.normalize_angle(yaw - self.start_yaw)
            if abs(diff) < math.pi/2 :
                self.state = "turning"
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
            if abs(diff) < math.pi/2:
                self.state = "turning"
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