import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

Max_Vel = 0.22  # m/s
Max_angVel = 1.5 #rad/s

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        #readin Lidar
        self.subscription = self.create_subscription(     #IN a subscription, any time a msg is published on the topic the callback function is executed
            LaserScan,
            'scan',
            self.LidarScanner,
            10)

        self.velocity = Twist()
   
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def LidarScanner(self,scan):
        self.velocity.angular.z = 0.0    
        self.velocity.linear.x=0.0
        
        if scan.ranges[0] < 0.75:

            if scan.ranges[89] > scan.ranges[269]:
                self.velocity.angular.z = Max_angVel-0.5
            else:
                self.velocity.angular.z = -Max_angVel-0.5
        else: 
            self.velocity.linear.x = Max_Vel
            
        if scan.ranges[29] < 0.6:
            self.velocity.angular.z = -Max_angVel-0.5
        elif scan.ranges[329] < 0.6: 
            self.velocity.angular.z = Max_angVel-0.5
            
        self.publisher_.publish(self.velocity)
        self.get_logger().info(f'Publishing Twist: linear.x={self.velocity.linear.x} angular.z={self.velocity.angular.z}')




        
def main(args=None):
    rclpy.init(args=args) #initializa ros python executer 

    controller_node = ControllerNode()  

    rclpy.spin(controller_node)   #the executro (rcply) has to spin the node (this is the way ros work)

    # Destroy the node explicitly
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()