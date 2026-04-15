import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/home/hywork/d1_nav_demo_ros2_ws/install/d1_nav_demo'
