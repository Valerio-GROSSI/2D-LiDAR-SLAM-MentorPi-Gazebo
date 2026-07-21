#!/usr/bin/env python3
# encoding: utf-8

"""
Low-level ROS 2 node for:
    - commanding the wheel motors;
    - reading wheel encoder measurements.
"""
import math
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.msg import MotorsState

class RosRobotMotorController(Node):
    def __init__(self) -> None:
        super().__init__('ros_robot_controller')

        self.declare_parameter('encoder_publish_rate', 50.0)
        self.declare_parameter('wheel_names', ['wheel_lf_joint', 'wheel_rf_joint', 'wheel_lr_joint', 'wheel_rr_joint'])

        self.encoder_publish_rate = float(self.get_parameter('encoder_publish_rate').value)
        self.wheel_names = list(self.get_parameter('wheel_names').value)

        if self.encoder_publish_rate <= 0.0:
            raise ValueError('encoder_publish_rate must be greater than zero')
        
        self.board = Board()
        self.board.enable_reception(True)
        self.running = True
        self.clock = self.get_clock()

        # Input: individual wheel speeds in rotations per second.
        self.motor_sub = self.create_subscription(
            MotorsState,
            '~/set_motor',
            self.set_motor_state,
            10, 
        )

        # Always start with stopped motors.
        self.stop_motors()

        # # Output: measured wheel positions and/or velocities.
        # self.encoder_pub = self.create_publisher(
        #     JointState,
        #     '~/wheel_states',
        #     10
        # )

        # self.encoder_thread = threading.Thread(
        #     target=self.encoder_loop,
        #     daemon=True,
        # )
        # self.encoder_thread.start()

        self.gravity = 9.80665

        self.imu_pub = self.create_publisher(
            Imu,
            '~/imu_raw',
            10,
        )

        self.imu_timer = self.create_timer(
            0.02,
            self.pub_imu_data,
        )

        self.get_logger().info('Motor controller, encoder and IMU publisher started')

    
    def stop_motors(self) -> None:
        """Stop all four drive motors."""
        self.board.set_motor_speed(
            [[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]]
        )

    def set_motor_state(self, msg: MotorsState) -> None:
        """Forward wheel-speed commands to the STM32 board."""
        commands = [[motor.id, motor.rps] for motor in msg.data]

        if not commands:
            self.get_logger().warning('Received an empty motor command')
            return
        
        self.board.set_motor_speed(commands)

    # def read_encoder_data(self) -> Optional[Any]:
    #     """
    #     Read wheel encoders from the Board SDK.

    #     The original code supplied by the user does not call any encoder method,
    #     so the exact SDK function and return format are unknown.

    #     Replace this implementation with the real API, for example:

    #         return self.board.get_motor_encoders()

    #     Expected result used by normalize_encoder_data():

    #         [(position_rad, velocity_rad_s), ...]   # four wheels

    #     or:

    #         {
    #             1: {'position': ..., 'velocity': ...},
    #             2: {'position': ..., 'velocity': ...},
    #             3: {'position': ..., 'velocity': ...},
    #             4: {'position': ..., 'velocity': ...},
    #         }
    #     """

    #     # Adapt this line after checking ros_robot_controller_sdk.py.
    #     if hasattr(self.board, 'get_motor_encoders'):
    #         return self.board.get_motor_encoders()

    #     return None
    
    # def normalize_encoder_data(self, raw_data: Any) -> tuple[list[float], list[float]]:
    #     """Convert SDK encoder output into JointState position and velocity."""
    #     positions: list[float] = []
    #     velocities: list[float] = []

    #     return positions, velocities
    
    # def encoder_loop(self) -> None:
    #     """Read and publish wheel encoder measurements periodically."""
    #     period = 1.0 / self.encoder_publish_rate
    #     missing_api_logged = False

    #     while rclpy.ok() and self.running:
    #         raw_data = self.read_encoder_data()

    #         if raw_data is None:
    #             if not missing_api_logged:
    #                 self.get_logger().warning(
    #                     'No encoder API is connected. Adapt read_encoder_data() '
    #                     'to the Board SDK before using ~/wheel_states.'
    #                 )
    #                 missing_api_logged = True
    #             time.sleep(period)
    #             continue

    #         try:
    #             positions, velocities = self.normalize_encoder_data(raw_data)
    #         except (KeyError, TypeError, ValueError) as exc:
    #             self.get_logger().error(f'Invalid encoder data: {exc}')
    #             time.sleep(period)
    #             continue

    #         msg = JointState()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.name = self.wheel_names
    #         msg.position = positions
    #         msg.velocity = velocities

    #         self.encoder_pub.publish(msg)
    #         time.sleep(period)

    def destroy_node(self) -> bool:
        """Stop motors before destroying the ROS node."""
        self.running = False
        try:
            self.stop_motors()
        finally:
            return super().destroy_node()
        
    def pub_imu_data(self):
        data = self.board.get_imu()

        if data is None:
            return
        
        ax, ay, az, gx, gy, gz = data

        msg = Imu()
        msg.header.frame_id = 'imu_link'
        msg.header.stamp = self.clock.now().to_msg()

        msg.orientation.w = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        msg.linear_acceleration.x = ax * self.gravity
        msg.linear_acceleration.y = ay * self.gravity
        msg.linear_acceleration.z = az * self.gravity

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        msg.linear_acceleration_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.004
        ]

        self.imu_pub.publish(msg)










def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosRobotMotorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
