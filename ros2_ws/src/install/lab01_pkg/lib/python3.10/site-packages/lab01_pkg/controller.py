import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

ROBOT_SPEED = 1.0  # m/s

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # crea emittente di messaggi di tipo Twist sul topic cmd_vel
        timer_period = 1.0  # seconds
        self.N = 1 #durata di ogni fase del movimento in secondi
        self.counter = 0 #contatore per tenere traccia del tempo trascorso in ogni fase
        self.state = 1 #stato del robot : 1=X+, 2=Y+, 3=X-, 4=Y-
        self.timer = self.create_timer(timer_period, self.timer_callback)  # timer impostato a 1 secondo
        self.get_logger().info('Controller node has been started.')

    def timer_callback(self):
        twist = Twist()  # crea un messaggio di tipo Twist ad ogni scadenza del timer

        if self.state == 1:  # Move forward in x direction
            twist.linear.x = ROBOT_SPEED
            twist.linear.y = 0.0
        elif self.state == 2:  # Move forward in y direction
            twist.linear.x = 0.0
            twist.linear.y = ROBOT_SPEED
        elif self.state == 3:  # Move backward in x direction
            twist.linear.x = -ROBOT_SPEED
            twist.linear.y = 0.0
        elif self.state == 4:  # Move backward in y direction
            twist.linear.x = 0.0
            twist.linear.y = -ROBOT_SPEED
        
        #controllo del tempo trascorso in ogni fase
        self.counter += 1
        if self.counter >= self.N:
            self.counter = 0
            self.state += 1
            if self.state > 4: # reset dello stato (finito il ciclo di movimento)
                self.state = 1

        self.N += 1
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing Twist: Time = {self.N} linear.x={twist.linear.x}, linear.y={twist.linear.y}')



def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()