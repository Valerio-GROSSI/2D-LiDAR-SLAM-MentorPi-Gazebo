#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node

from ros_robot_controller_msgs.msg import MotorsState, MotorState


class MotorCommandPublisher(Node):

    def __init__(self) -> None:
        super().__init__('motor_command_publisher')

        self.publisher = self.create_publisher(
            MotorsState,
            '/ros_robot_controller/set_motor',
            10,
        )

        self.timer = self.create_timer(
            0.01,  # 1 Hz
            self.publish_motor_command,
        )

    def publish_motor_command(self) -> None:
        msg = MotorsState()

        motor_1 = MotorState()
        motor_1.id = 1
        motor_1.rps = 0.5

        motor_2 = MotorState()
        motor_2.id = 2
        motor_2.rps = 0.5

        motor_3 = MotorState()
        motor_3.id = 3
        motor_3.rps = 0.5

        motor_4 = MotorState()
        motor_4.id = 4
        motor_4.rps = 0.5

        msg.data = [
            motor_1,
            motor_2,
            motor_3,
            motor_4,
        ]

        self.publisher.publish(msg)

        self.get_logger().info(
            'Motor command published: 0.5 RPS on motors 1 to 4'
        )

        # self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MotorCommandPublisher()

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