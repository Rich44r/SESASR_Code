import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist

ROBOT_SPEED = 1.0  # m/s

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_reset')
        
        timer_period = 1.0  # seconds
        self.N = 1 #durata di ogni fase del movimento in secondi
        self.dir = 0 #direction of the robot : 0=X+, 1=Y+, 2=X-, 3=Y-
        self.it = 0 #iteration counter

        self.timer = self.create_timer(timer_period, self.timer_callback)  # timer impostato a 1 secondo
        self.subscription_reset = self.create_subscription(
            Bool,
            'reset',
            self.reset_callback,
            10)
        self.subscription_reset  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # crea emittente di messaggi di tipo Twist sul topic cmd_vel
        self.get_logger().info('Controller node has been started.')
        
    def reset_callback(self,msg):
        if msg.data: #if the reset message is true
            self.get_logger().info('Reset message received: RESTARTING THE CONTROL SEQUENCE')
            self.N = 1
            self.dir = 0
            self.it = 0


    def timer_callback(self):
        msg = Twist()  # crea un messaggio di tipo Twist ad ogni scadenza del timer

        if self.it == self.N:
            self.dir += 1
            self.it = 0

        match self.dir:
            case 0:  # X+
                msg.linear.x = ROBOT_SPEED
                msg.linear.y = 0.0
            case 1:  # Y+
                msg.linear.x = 0.0
                msg.linear.y = ROBOT_SPEED
            case 2:  # X-
                msg.linear.x = -ROBOT_SPEED
                msg.linear.y = 0.0
            case 3:  # Y-
                msg.linear.x = 0.0
                msg.linear.y = -ROBOT_SPEED
            case _:  # default
                msg.linear.x = 0.0
                msg.linear.y = 0.0


        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Twist: Time = {self.N} linear.x={msg.linear.x}, linear.y={msg.linear.y}')

        self.it += 1

        if self.it == self.N and self.dir == 3:
            self.N += 1  # increase duration after completing a full square
            self.it = 0
            self.dir = 0  # reset direction to start a new square




def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()