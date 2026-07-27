#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped


class PoseToPoseStamped(Node):

    def __init__(self):
        super().__init__('pose_to_pose_stamped')

        self.declare_parameter('frame_id', 'map')

        self.frame_id = (self.get_parameter('frame_id').get_parameter_value().string_value)

        self.pose_sub = self.create_subscription(Pose, '/model/MentorPi/pose', self.pose_callback, 10)

        self.pose_stamped_pub = self.create_publisher(PoseStamped, '/model/MentorPi/pose_ref', 10,)

        self.get_logger().info(f'Conversion /model/MentorPi/pose -> 'f'/model/MentorPi/pose_ref, frame_id={self.frame_id}')

    def pose_callback(self, pose_msg):
        pose_stamped_msg = PoseStamped()

        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = self.frame_id
        pose_stamped_msg.pose = pose_msg

        self.pose_stamped_pub.publish(pose_stamped_msg)


def main(args=None):
    rclpy.init(args=args)

    node = PoseToPoseStamped()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
