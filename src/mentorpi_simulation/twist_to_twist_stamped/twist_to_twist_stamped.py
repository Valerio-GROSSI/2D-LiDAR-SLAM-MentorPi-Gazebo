#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped


class TwistToTwistStamped(Node):

    def __init__(self):
        super().__init__("twist_to_twist_stamped")

        # Paramètre pour le frame_id
        self.declare_parameter("frame_id", "base_link")

        self.frame_id = self.get_parameter("frame_id").value

        # Subscriber
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.callback,
            10,
        )

        # Publisher
        self.pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel_stamped",
            10,
        )

        self.get_logger().info("Twist -> TwistStamped converter started")

    def callback(self, msg: Twist):

        stamped = TwistStamped()

        # Horodatage actuel
        stamped.header.stamp = self.get_clock().now().to_msg()

        # Repère associé à la commande
        stamped.header.frame_id = self.frame_id

        # Copier la commande
        stamped.twist = msg

        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)

    node = TwistToTwistStamped()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()