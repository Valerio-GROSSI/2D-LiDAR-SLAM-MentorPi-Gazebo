#!/usr/bin/python3
# coding=utf8

import math
from ros_robot_controller_msgs.msg import MotorState, MotorsState

class DifferentialChassis:
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_convert(self, speed):
        """
        convert speed m/s to rps/s
        :param speed:
        :return:
        """
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        left = linear_x - angular_z * (self.track_width / 2)
        right = linear_x + angular_z * (self.track_width / 2)

        motor1 = left
        motor2 = right
        motor3 = left
        motor4 = right

        v_s = [self.speed_convert(v) for v in [motor1, motor2, motor3, motor4]]

        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)

        msg = MotorsState()
        msg.data = data
        return msg