import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from ekf import RobotEKF
import utils
import yaml

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray
from std_msgs.msg import Bool


class EKF_node(Node):
    def __init__(self):
        super().__init__('EKF_node')


        # general noise parameters
        std_lin_vel = 0.1  # [m/s]
        std_ang_vel = np.deg2rad(1.0)  # [rad/s]
        self.sigma_u = np.array([std_lin_vel, std_ang_vel])
        self.sigma_u_odom = 0

        # Define noise params and Q for landmark sensor model
        std_range = 0.1  # [m]
        std_bearing = np.deg2rad(1.0)  # [rad]
        self.sigma_z = np.array([std_range, std_bearing])
        self.Q_landm = np.diag([std_range**2, std_bearing**2])
        # Define H Jacobian function
        _, self.eval_Ht = utils.landmark_sm_simpy()

        #initialize EKF
        eval_gux = utils.sample_velocity_motion_model
        _, eval_Gt, eval_Vt = utils.velocity_mm_simpy()
        self.ekf = RobotEKF(dim_x=3, dim_u=2, eval_gux=eval_gux, eval_Gt=eval_Gt, eval_Vt=eval_Vt)
        self.ekf.mu = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.ekf.Sigma = np.diag([0.1, 0.1, 0.1])
        self.ekf.Mt = np.diag([std_lin_vel**2, std_ang_vel**2])

        # Initialize state variables
        self.x=0.0
        self.y=0.0
        self.theta_z = 0.0
        self.v = 0.0
        self.w = 0.0
        #flag to check if odometry has been received
        self.ekf_ready = False

        #lettura landmark nel file yaml
        self.filename = "../../turtlebot3_perception/turtlebot3_perception/config/landmarks.yaml"

        with open(self.filename, 'r') as file:
            data = yaml.safe_load(file)

        self.landmarks_matrix = np.column_stack((  data['landmarks']['x'],data['landmarks']['y']))



        # Create a timer to call the EKF update at a fixed rate
        self.timer = self.create_timer(1/20, self.ekf_callback)  # 20 Hz

        #read odometry and refresh the latest odometry 
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        #read landmarks
        self.subscription = self.create_subscription(
            LandmarkArray,
            'landmarks',
            self.landmarks_callback,
            10)
        
        #publish the estimated pose
        self.publisher_ = self.create_publisher(
            Odometry, 
            'ekf', 
            10)


        self.last_odom = None # ultima lettura di odometria

        self.subscription  # prevent unused variable warning
        self.get_logger().info('EKF node has been started.')



    def odom_callback(self, msg):
        self.last_odom = msg
        self.get_logger().info(f'Received Odometry: position.x={msg.pose.pose.position.x}, position.y={msg.pose.pose.position.y}')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        self.theta_z = euler[2]  # Yaw angle
        self.get_logger().info(f'Extracted Yaw (theta_z)={self.theta_z}')

        # extract linear and angular velocities
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        #enable EKF after first odometry reception
        self.ekf_ready = True
        self.get_logger().info(f'Extracted velocities: v={self.v}, w={self.w}')

    def landmarks_callback(self, msg):
        if not self.ekf_ready:
            self.get_logger().info('EKF not ready, odometry data not yet received.')
            return

        landmarks_measured = msg
        self.get_logger().info(f'Received Landmarks: number of landmarks={len(msg.landmarks)}')
        #Process each landmark measurement
        for lmark in landmarks_measured: 
            #take measurement vector
            z = np.array([lmark.range, lmark.bearing])
            id_seen = lmark.id
            #perform EKF update for each landmark
            self.ekf.update(
                        z,
                        eval_hx=utils.eval_hx_landm,
                        eval_Ht=utils.eval_Ht_landm,
                        Qt=self.Q_landm,
                        Ht_args=(*self.ekf.mu, *self.landmarks_matrix),  # the Ht function requires a flattened array of parameters
                        hx_args=(self.ekf.mu, lmark, self.sigma_z),
                        residual=utils.residual,
                        angle_idx=id_seen,
                    )
        


    def ekf_callback(self):
        if not self.ekf_ready:
            return  # Skip EKF update if not ready
        self.ekf.predict(u=[self.v,self.w], sigma_u=self.sigma_u, g_extra_args=(1/20,))

        
        
        


def main(args=None):
    rclpy.init(args=args)

    EKF_node = EKF_node()

    rclpy.spin(EKF_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    EKF_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()