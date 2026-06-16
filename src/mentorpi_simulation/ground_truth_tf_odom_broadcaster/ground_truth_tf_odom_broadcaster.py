#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthTfBroadcaster(Node):
    def __init__(self):
        super().__init__("ground_truth_tf_broadcaster")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Pose,
            "/model/MentorPi/pose",
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()

        # On suppose ici que le monde Gazebo "empty" est aligné avec odom
        tf.header.frame_id = "odom"

        # Si l'origine du modèle Gazebo correspond à base_footprint
        tf.child_frame_id = "base_footprint"

        tf.transform.translation.x = msg.position.x
        tf.transform.translation.y = msg.position.y
        tf.transform.translation.z = msg.position.z

        tf.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = GroundTruthTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()