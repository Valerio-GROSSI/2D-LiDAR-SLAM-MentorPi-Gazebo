import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/valerio/2D-LiDAR-SLAM-MentorPi-Gazebo/install/ros_robot_controller'
