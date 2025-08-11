import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pathonai/ros2_ws/src/opensource_dev/install/ThinkGrasp'
