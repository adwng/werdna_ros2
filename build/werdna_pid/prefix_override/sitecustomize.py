import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/andrew/werdna_ws/src/werdna_ros2/install/werdna_pid'
