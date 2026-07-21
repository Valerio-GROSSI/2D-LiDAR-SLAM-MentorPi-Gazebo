#!/usr/bin/env python3
# coding=utf8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import signal
import time
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

ODOM_POSE_SCOVARIANCE = list(map(float, [1e-3, 0, 0, 0, 0, 0,
                                         0, 1e-3, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float, [1e-9, 0, 0, 0, 0, 0, 
                                             0, 1e-3, 1e-9, 0, 0, 0,
                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,
                                             0, 0, 0, 0, 1e6, 0,
                                             0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = list(map(float, [1e-3, 0, 0, 0, 0, 0,
                                         0, 1e-3, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e3]))

ODOM_TWIST_COVARIANCE_STOP = list(map(float, [1e-9, 0, 0, 0, 0, 0, 
                                             0, 1e-3, 1e-9, 0, 0, 0,
                                             0, 0, 1e6, 0, 0, 0,
                                             0, 0, 0, 1e6, 0, 0,
                                             0, 0, 0, 0, 1e6, 0,
                                             0, 0, 0, 0, 0, 1e-9]))

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)

    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

class Controller(Node):
    def __init__(self, name):
        super().__init__(name)

        self.x = 0.0
        self.y = 0.0
        self.pose_yaw = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.last_time = 0.0
        self.current_time = 0.0
        signal.signal(signal.SIGINT, self.shutdown)

        # Declare parameters
        self.declare_parameter('pub_odom_topic', True)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')

        self.pub_odom_topic = self.get_parameter('pub_odom_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        self.clock = self.get_clock()
        if self.pub_odom_topic:
            self.tf_broadcaster = TransformBroadcaster(self)

            self.odom = Odometry()
            self.odom.header.frame_id = self.odom_frame_id
            self.odom.child_frame_id = self.base_frame_id

            self.odom.pose.covariance = ODOM_POSE_SCOVARIANCE
            self.odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)
            self.dt = 1.0/50.0

            threading.Thread(target=self.cal_odom_func, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z

    def cal_odom_func(self): 
        while True:
            self.current_time = time.time()
            if self.last_time is None:
                self.dt = 0.0
            else:
                self.dt = self.current_time - self.last_time
            
            self.odom.header.stamp = self.clock.now().to_msg()
            # delta_x = self.linear_x * self.dt * math.cos(self.pose_yaw)
            # delta_y = self.linear_y * self.dt * math.sin(self.pose_yaw)
            delta_x = (self.linear_x * math.cos(self.pose_yaw) - self.linear_y * math.sin(self.pose_yaw)) * self.dt
            delta_y = (self.linear_x * math.sin(self.pose_yaw) + self.linear_y * math.cos(self.pose_yaw)) * self.dt

            delta_yaw = self.angular_z * self.dt

            self.x += delta_x
            self.y += delta_y
            self.pose_yaw += delta_yaw

            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)

            self.odom.twist.twist.linear.x = self.linear_x
            self.odom.twist.twist.linear.y = self.linear_y
            self.odom.twist.twist.angular.z = self.angular_z

            if self.linear_x == 0.0 and self.linear_y == 0.0 and self.angular_z == 0.0:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
            else:
                self.odom.pose.covariance = ODOM_POSE_SCOVARIANCE
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
            
            self.odom_pub.publish(self.odom)

            transform = TransformStamped()

            transform.header.stamp = self.odom.header.stamp
            transform.header.frame_id = self.odom.header.frame_id
            transform.child_frame_id = self.odom.child_frame_id

            transform.transform.translation.x = self.odom.pose.pose.position.x
            transform.transform.translation.y = self.odom.pose.pose.position.y
            transform.transform.translation.z = self.odom.pose.pose.position.z
            
            transform.transform.rotation = self.odom.pose.pose.orientation

            self.tf_broadcaster.sendTransform(transform)

            self.last_time = self.current_time
            time.sleep(0.02)
        
    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Controller('odom_publisher_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()