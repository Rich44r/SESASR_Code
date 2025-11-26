#Here is contained the part regarding the plots for task1
from rosbag2_reader_py import Rosbag2Reader
import numpy as np
from scipy.interpolate import interp1d
from rclpy.time import Time

#open rosbag
path = "/home/luke_skywalker/ros2_ws/rosbag2_2025_11_22-11_01_01"

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
reader.set_filter(["/ground_truth", "/ekf"])

for topic_name, msg, t in reader:
    if topic_name == "/ground_truth":
        time_gt.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    elif topic_name == "/ekf":
        time_ekf.append(Time.from_msg(msg.header.stamp).nanoseconds)
        data_ekf.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

time_gt = np.array(time_gt)
data_gt = np.array(data_gt)
time_ekf= np.array(time_ekf)
data_ekf = np.array(data_ekf)

print(f"Ground truth points: {len(data_gt)}")
print(f"Odometry points: {len(data_ekf)}")

#state plot vs time


#trajectories on 2D plane
