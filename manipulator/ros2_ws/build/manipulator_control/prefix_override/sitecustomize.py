import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student35/Documents/Group5/ros2_ws/install/manipulator_control'
