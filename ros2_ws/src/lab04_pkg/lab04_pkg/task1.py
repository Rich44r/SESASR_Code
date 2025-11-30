from rosbag2_reader_py import Rosbag2Reader
import numpy as np
from scipy.interpolate import interp1d
from rclpy.time import Time
import matplotlib.pyplot as plt
import lab04_pkg.utils as utils
import tf_transformations

def convert_quat(msg):
    orientation = msg
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    euler = tf_transformations.euler_from_quaternion(quaternion)

    return (euler[2])

#open rosbag
path = "/home/luke_skywalker/ros2_ws/rosbag2_2025_11_29-17_02_05"

reader = Rosbag2Reader(path)
topics = reader.all_topics
topics

time_gt = []
data_gt = []
time_odom = []
data_odom = []
time_ekf = []
data_ekf = []

#topics we are interested in
reader.set_filter(["/ground_truth","/ekf", "/odom"])

for topic_name, msg, t in reader:
    if topic_name == "/ground_truth":
        yaw = convert_quat(msg.pose.pose.orientation)
        time_gt.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
    elif topic_name == "/ekf":
        yaw = convert_quat(msg.pose.pose.orientation)
        time_ekf.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_ekf.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
    elif topic_name == "/odom":
        yaw = convert_quat(msg.pose.pose.orientation)
        time_odom.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_odom.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

time_gt = np.array(time_gt)
data_gt = np.array(data_gt)
time_ekf= np.array(time_ekf)
data_ekf = np.array(data_ekf)
time_odom = np.array(time_odom)
data_odom = np.array(data_odom)

print(f"Ground truth points: {len(data_gt)}")
print(f"EKF points: {len(data_ekf)}")
print(f"Odometry points: {len(data_odom)}")

#state plot vs time


#trajectories on 2D plane
plt.figure(figsize=(10,6))
plt.plot(data_gt[:,0], data_gt[:,1], label = 'GroundTruth')
plt.plot(data_ekf[:,0], data_ekf[:,1], label = 'EKF', linestyle = ':')
plt.plot(data_odom[:,0], data_odom[:,1], label = 'Odometry')

plt.legend()
plt.title('Trajectories')
plt.grid(True)
plt.show()

#Interpolation 
gt_interpol = interp1d(time_gt, data_gt, axis=0, fill_value="extrapolate", kind="nearest")
data_int_gt = gt_interpol(time_odom)
print(f"Interpolated ground truth points: {len(data_int_gt)}")

gt_interpol = interp1d(time_ekf, data_ekf, axis=0, fill_value="extrapolate", kind="nearest")
data_int_ekf = gt_interpol(time_odom)
print(f"Interpolated ekf points: {len(data_int_ekf)}")

#RMSE and MAE for ekf data
print("---EKF metrics---")
#actual - predicted
RMSE_ekf = utils.rmse(data_int_gt, data_int_ekf)
print(f'RMSE of ekf data: {RMSE_ekf}')

#error: actual - predicted
ekf_error = np.array(data_int_gt - data_int_ekf)
MAE_ekf = utils.mae(ekf_error)
print(f'MAE of ekf data: {MAE_ekf}')

#RMSE and MAE for odom data
print("---ODOM metrics---")
RMSE_odom = utils.rmse(data_int_gt, data_odom)
print(f'RMSE of odom data: {RMSE_odom}')

#error: actual - predicted
odom_error = np.array(data_int_gt - data_odom)
MAE_odom = utils.mae(odom_error)
print(f'MAE of odom data: {MAE_odom}')

#--plots for each state--

#x(t) state
plt.figure(figsize=(10,5))
plt.plot(time_odom, data_odom[:,0], label="odom")
plt.plot(time_gt, data_int_gt[:,0], label="ground_truth")
plt.plot(time_ekf, data_int_ekf[:,0], label="ekf")
plt.legend()
plt.title("X(t) – confronto tra i topic")
plt.grid(True)
plt.show()


plt.subplot(3,1,1)
plt.plot(time_gt, data_gt[:,0])
plt.title('X(t) /ground_truth')
plt.grid(True)


plt.subplot(3,1,2)
plt.plot(time_ekf, data_ekf[:,0])
plt.title('X(t) /ekf')
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_odom, data_odom[:,0])
plt.title('X(t) /odom')
plt.grid(True)
plt.show()

#y(t) state
plt.subplot(3,1,1)
plt.plot(time_gt, data_gt[:,1])
plt.title('Y(t) /ground_truth')
plt.grid(True)


plt.subplot(3,1,2)
plt.plot(time_ekf, data_ekf[:,1])
plt.title('Y(t) /ekf')
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_odom, data_odom[:,1])
plt.title('Y(t) /odom')
plt.grid(True)
plt.show()

#θ(t) state
plt.subplot(3,1,1)
plt.plot(time_gt, data_gt[:,2])
plt.title('θ(t) /ground_truth')
plt.grid(True)


plt.subplot(3,1,2)
plt.plot(time_ekf, data_ekf[:,2])
plt.title('θ(t) /ekf')
plt.grid(True)

plt.subplot(3,1,3)
plt.plot(time_odom, data_odom[:,2])
plt.title('θ(t) /odom')
plt.grid(True)
plt.show()