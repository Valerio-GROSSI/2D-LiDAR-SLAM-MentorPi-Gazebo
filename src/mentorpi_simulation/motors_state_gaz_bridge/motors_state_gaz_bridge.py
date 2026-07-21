#!/usr/bin/env python3

import math
from typing import Dict

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from ros_robot_controller_msgs.msg import MotorsState


class MotorsStateToWheelVelocity(Node):

    def __init__(self) -> None:
        super().__init__('motors_state_to_wheel_velocity')

        self.declare_parameter(
            'input_topic',
            '/ros_robot_controller/set_motor'
        )
        self.declare_parameter(
            'output_topic',
            '/wheel_velocity_controller/commands'
        )

        # Correspondance :
        # [wheel_lf, wheel_rf, wheel_lb, wheel_rb]
        self.declare_parameter('front_left_motor_id', 1)
        self.declare_parameter('front_right_motor_id', 2)
        self.declare_parameter('rear_left_motor_id', 3)
        self.declare_parameter('rear_right_motor_id', 4)

        # Permet d'inverser individuellement le sens d'une roue.
        self.declare_parameter('front_left_direction', 1.0)
        self.declare_parameter('front_right_direction', 1.0)
        self.declare_parameter('rear_left_direction', 1.0)
        self.declare_parameter('rear_right_direction', 1.0)

        self.declare_parameter('command_timeout', 0.5)

        input_topic = (
            self.get_parameter('input_topic')
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter('output_topic')
            .get_parameter_value()
            .string_value
        )

        self.motor_ids = [
            self.get_parameter('front_left_motor_id').value,
            self.get_parameter('front_right_motor_id').value,
            self.get_parameter('rear_left_motor_id').value,
            self.get_parameter('rear_right_motor_id').value,
        ]

        self.directions = [
            float(self.get_parameter('front_left_direction').value),
            float(self.get_parameter('front_right_direction').value),
            float(self.get_parameter('rear_left_direction').value),
            float(self.get_parameter('rear_right_direction').value),
        ]

        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )

        self.last_command_time = self.get_clock().now()
        self.timeout_active = False

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            output_topic,
            10
        )

        self.motor_subscription = self.create_subscription(
            MotorsState,
            input_topic,
            self.motor_callback,
            10
        )

        self.timeout_timer = self.create_timer(
            0.05,
            self.check_timeout
        )

        self.get_logger().info(
            f'Conversion {input_topic} -> {output_topic}'
        )
        self.get_logger().info(
            'Ordre de sortie : '
            '[wheel_lf_Joint, wheel_rf_Joint, '
            'wheel_lb_Joint, wheel_rb_Joint]'
        )
        self.get_logger().info(
            f'IDs moteurs correspondants : {self.motor_ids}'
        )

    def motor_callback(self, msg: MotorsState) -> None:
        rps_by_id: Dict[int, float] = {
            int(motor.id): float(motor.rps)
            for motor in msg.data
        }

        missing_ids = [
            motor_id
            for motor_id in self.motor_ids
            if motor_id not in rps_by_id
        ]

        if missing_ids:
            self.get_logger().warning(
                f'IDs moteurs absents du message : {missing_ids}'
            )
            return

        wheel_velocities = []

        for motor_id, direction in zip(
            self.motor_ids,
            self.directions
        ):
            rps = rps_by_id[motor_id]
            angular_velocity = direction * 2.0 * math.pi * rps
            wheel_velocities.append(angular_velocity)

        command = Float64MultiArray()
        command.data = wheel_velocities

        self.command_publisher.publish(command)

        self.last_command_time = self.get_clock().now()
        self.timeout_active = False

    def check_timeout(self) -> None:
        elapsed = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9

        if elapsed <= self.command_timeout:
            return

        if self.timeout_active:
            return

        stop_command = Float64MultiArray()
        stop_command.data = [0.0, 0.0, 0.0, 0.0]

        self.command_publisher.publish(stop_command)
        self.timeout_active = True

        self.get_logger().warning(
            'Timeout des commandes moteurs : arrêt des roues'
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MotorsStateToWheelVelocity()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_command = Float64MultiArray()
        stop_command.data = [0.0, 0.0, 0.0, 0.0]

        node.command_publisher.publish(stop_command)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()