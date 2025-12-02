import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from lab04_pkg.ekf import RobotEKF
import lab04_pkg.utils as utils
import yaml
import angles

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
        self.eval_hx_landm = utils.landmark_range_bearing_model

        #initialize EKF
        eval_gux = utils.sample_velocity_motion_model
        _, eval_Gt, eval_Vt = utils.velocity_mm_simpy()
        self.ekf = RobotEKF(dim_x=3, dim_u=2, eval_gux=eval_gux, eval_Gt=eval_Gt, eval_Vt=eval_Vt)
        self.ekf.mu = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.ekf.Sigma = np.diag([0.0, 0.0, 0.0])
        self.ekf.Mt = np.diag([std_lin_vel**2, std_ang_vel**2])

        # Initialize command variables
        self.v = 0.0
        self.w = 0.0
        #flag to check if odometry has been received
        self.ekf_ready = False

        #lettura landmark nel file yaml
        self.filename = "/home/luke_skywalker/ros2_ws/src/turtlebot3_perception/turtlebot3_perception/config/landmarks.yaml"

        with open(self.filename, 'r') as file:
            data = yaml.safe_load(file)

        landmarks_matrix = np.column_stack((  data['landmarks']['x'],data['landmarks']['y']))
        id_list = [11, 12, 13, 21, 22, 23, 31, 32, 33]
        landmarks_matrix = np.column_stack((  data['landmarks']['x'],data['landmarks']['y']))

        #creation of a dictionary
        self.landmarks_coordinate = {}
        for id, landmark in zip(id_list, landmarks_matrix):
            self.landmarks_coordinate[id] = landmark

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
        #self.get_logger().info(f'Received Odometry: position.x={msg.pose.pose.position.x}, position.y={msg.pose.pose.position.y}')

        # extract linear and angular velocities
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        #enable EKF after first odometry reception
        self.ekf_ready = True
        #self.get_logger().info(f'Extracted velocities: v={self.v}, w={self.w}')

    def landmarks_callback(self, msg):
        if not self.ekf_ready:
            #self.get_logger().info('EKF not ready, odometry data not yet received.')
            return

        landmarks_measured = msg
        self.get_logger().info(f'Received Landmarks: number of landmarks={len(msg.landmarks)}')
        #Process each landmark measurement
        for lmark in landmarks_measured.landmarks: 
            #take measurement vector
            z = np.array([lmark.range, lmark.bearing])
            self.get_logger().info(f'ID seen = {lmark.id}')
            id_seen = lmark.id
            #perform EKF update for each landmark
            self.ekf.update(
                        z,
                        eval_hx = self.eval_hx_landm,
                        eval_Ht = self.eval_Ht,
                        Qt = self.Q_landm,
                        Ht_args = (*self.ekf.mu, *self.landmarks_coordinate[id_seen]),  # the Ht function requires a flattened array of parameters
                        hx_args = (self.ekf.mu, self.landmarks_coordinate[id_seen], self.sigma_z),
                        residual = utils.residual,
                        angle_idx = -1,
                    )
            #angle normalization between -pi pi
            self.ekf.mu[2] = angles.normalize_angle(self.ekf.mu[2])
        
        # After processing all landmarks, publish the estimated pose
        ekf_msg = Odometry()
        #filling the header
        ekf_msg.header.stamp = self.get_clock().now().to_msg()
        ekf_msg.header.frame_id = 'odom'
        #filling the pose
        ekf_msg.pose.pose.position.x = self.ekf.mu[0]
        ekf_msg.pose.pose.position.y = self.ekf.mu[1]
        #filling the orientation with quaternions (self.ekf.mu[2] is Yaw)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.ekf.mu[2])
        ekf_msg.pose.pose.orientation.x = quat[0]
        ekf_msg.pose.pose.orientation.y = quat[1]
        ekf_msg.pose.pose.orientation.z = quat[2]
        ekf_msg.pose.pose.orientation.w = quat[3]
        self.publisher_.publish(ekf_msg)
        self.get_logger().info(f'Published EKF Pose: x={self.ekf.mu[0]}, y={self.ekf.mu[1]}, theta={self.ekf.mu[2]}')

    def ekf_callback(self):
        if not self.ekf_ready:
            return  # Skip EKF update if not ready
        self.ekf.predict(u=np.array([self.v,self.w]), sigma_u=self.sigma_u, g_extra_args=(1/20,))

        
        
        


def main(args=None):
    rclpy.init(args=args)

    node = EKF_node()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()