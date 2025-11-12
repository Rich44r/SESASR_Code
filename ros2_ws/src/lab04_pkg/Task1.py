import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from numpy.linalg import inv
import tf_transformations
from landmark_msgs.msg import LandmarkArray
import yaml

class RobotEKF:
    def __init__(
        self,
        dim_x=2, #v,w
        dim_u=3, #x,y,theta
        eval_gux=None,
        eval_Gt=None,
        eval_Vt=None,
    ):
        """
        Initializes the extended Kalman filter creating the necessary matrices
        """
        self.mu = np.zeros((dim_x))  # mean state estimate
        self.Sigma = np.eye(dim_x)  # covariance state estimate
        self.Mt = np.eye(dim_u)  # process noise
        self.dt = 0.05  #1/f=dt with f=50Hz
        self.sigma_u = np.array([0.1, 0.1])  # esempio di deviazioni standard del comando [v, w]

        self.eval_gux = eval_gux
        self.eval_Gt = eval_Gt
        self.eval_Vt = eval_Vt

        self._I = np.eye(dim_x)  # identity matrix used for computations

        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10) # Subscriber
      
        self.landmark_sub = self.create_subscription(LandmarkArray, '/landmarks', self.landmark_callback, 10)

        # Publisher
        self.ekf_pub = self.create_publisher(Odometry, '/ekf', 10)

        # Timer per la predizione
        self.timer = self.create_timer(self.dt, self.predict_step)

        #lettura landmark nel file yaml
        self.filename = "/home/federichina/turtlebot3_perception/turtlebot3_perception/config/landmarks.yaml"

        with open(self.filename, 'r') as file:
            data = yaml.safe_load(file)

        self.landmarks_matrix = np.column_stack((  data['landmarks']['x'],data['landmarks']['y']))


        

    def odom_callback(self, msg: Odometry):
        # Convert quaternion to yaw
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        self.get_logger().info(f'Odom -> x: {position.x:.2f}, y: {position.y:.2f}, Yaw: {yaw:.2f} rad')

        self.last_odom = self.current_odom
        self.current_odom = np.array([position.x, position.y, yaw])

        dx = self.current_odom - self.last_odom
        v = np.linalg.norm(dx[:2]) / self.dt    #norma è sqrt(vx^2+vy^2) 
        #[:2] non compreso quindi non usa yaw
        w = (dx[2]) / self.dt
        u = np.array([v, w])

        self.predict(u, self.sigma_u, g_extra_args=(self.dt,))
    
    

    def predict(self, u, sigma_u, g_extra_args=()):
        """
        Update the state prediction using the control input u and compute the relative uncertainty ellipse
        Parameters
        ----------

        u : np.array
            command for this step.

        sigma_u : np.array
            std dev for each component of the command signal

        extra_args : tuple
            any additional required parameter: dt

        Modified variables:
            self.mu: the state prediction
            self.Sigma: the covariance matrix of the state prediction
        """
        # Update the state prediction evaluating the motion model
    


        self.mu = self.eval_gux(self.mu, u, sigma_u, *g_extra_args)
        
        args = (*self.mu, *u)
        # Update the covariance matrix of the state prediction,
        # you need to evaluate the Jacobians Gt and Vt

        Gt = self.eval_Gt(*args, *g_extra_args)
        Vt = self.eval_Vt(*args, *g_extra_args)
        self.Sigma = Gt @ self.Sigma @ Gt.T + Vt @ self.Mt @ Vt.T

#da qui è da fare :)
    def update(self, z, eval_hx, eval_Ht, Qt, Ht_args=(), hx_args=(),  residual=np.subtract, **kwargs):
        """Performs the update innovation of the extended Kalman filter.

        Parameters
        ----------

        z : np.array
            measurement for this step.

        lmark : [x, y] list-like
            Landmark location in cartesian coordinates.

        residual : function (z, z2), optional
            Optional function that computes the residual (difference) between
            the two measurement vectors. If you do not provide this, then the
            built in minus operator will be used. You will normally want to use
            the built in unless your residual computation is nonlinear (for
            example, if they are angles)
        """

        # Convert the measurement to a vector if necessary. Needed for the residual computation
        if np.isscalar(z):
            z = np.asarray([z], float)
            Qt = np.atleast_2d(Qt).astype(float)
            Ht = np.atleast_2d(Ht).astype(float)

        # Compute the Kalman gain, you need to evaluate the Jacobian Ht
        Ht = eval_Ht(*Ht_args)
        SigmaHT = self.Sigma @ Ht.T
        self.S = Ht @ SigmaHT + Qt
        self.K = SigmaHT @ inv(self.S)

        # Evaluate the expected measurement and compute the residual, then update the state prediction
        z_hat = eval_hx(*hx_args)
        if np.isscalar(z_hat):
            z_hat = np.asarray([z_hat], float)

        # if the z measurement include an angle update, we need to specify the positional index to normalize the residual
        y = residual(z, z_hat, **kwargs)
        self.mu = self.mu + self.K @ y

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
        # Note that I is the identity matrix.
        I_KH = self._I - self.K @ Ht
        self.Sigma = I_KH @ self.Sigma @ I_KH.T + self.K @ Qt @ self.K.T

        self.publish_ekf()

def main(args=None):
        rclpy.init(args=args)
        node = RobotEKF()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


