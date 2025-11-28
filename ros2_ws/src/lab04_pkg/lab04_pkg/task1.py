#Here is contained the part regarding the plots for task1
from rosbag2_reader_py import Rosbag2Reader
import numpy as np
from scipy.interpolate import interp1d
from rclpy.time import Time
import matplotlib.pyplot as plt
import lab04_pkg.utils as utils

#open rosbag
path = "/home/luke_skywalker/ros2_ws/rosbag2_2025_11_28-09_34_14"

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
reader.set_filter(["/ground_truth", "/ekf", "/odom"])

for topic_name, msg, t in reader:
    if topic_name == "/ground_truth":
        time_gt.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    elif topic_name == "/ekf":
        time_ekf.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_ekf.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    elif topic_name == "/odom":
        time_odom.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_odom.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

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
interpolation_odom = gt_interpol(time_odom)
print(f"Interpolated ground truth points: {len(interpolation_odom)}")

gt_interpol = interp1d(time_gt, data_gt, axis=0, fill_value="extrapolate", kind="nearest")
interpolation_ekf = gt_interpol(time_ekf)
print(f"Interpolated ground truth points: {len(interpolation_ekf)}")

#RMSE and MAE for ekf data
print("---EKF metrics---")
RMSE_ekf = utils.rmse(interpolation_ekf, data_ekf)
print(f'RMSE of ekf data: {RMSE_ekf}')

#error: actual - predicted
ekf_error = np.array(data_ekf - interpolation_ekf)
MAE_ekf = utils.mae(ekf_error)
print(f'MAE of ekf data: {MAE_ekf}')

#RMSE and MAE for odom data
print("---ODOM metrics---")
RMSE_odom = utils.rmse(interpolation_odom, data_odom)
print(f'RMSE of odom data: {RMSE_odom}')

#error: actual - predicted
odom_error = np.array(data_odom - interpolation_odom)
MAE_odom = utils.mae(odom_error)
print(f'MAE of odom data: {MAE_odom}')