import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/belal/ros2_ws/src/assignment1_rt/install/assignment1_rt'
