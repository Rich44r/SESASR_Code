import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/luke_skywalker/ros2_ws/src/install/lab01_pkg'
