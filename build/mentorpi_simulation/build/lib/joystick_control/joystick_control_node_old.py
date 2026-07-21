#!/usr/bin/env python3
# coding=utf8

import os
import sys
import threading
import rclpy
import pygame as pg
from rclpy.node import Node
from geometry_msgs.msg import Twist

AXES_MAP = ('0', '1', '2', '3', 'hat_x', 'hat_y')

BUTTONS = [
    ("cross", "circle", "", "square", "triangle", "", "l1",
     "R1", "L2", "R2", "select", "start", "mode", "lc", "rc"),

    ("triangle", "circle", "cross", "square", "l1", "r1", "l2", "r2",
     "select", "start", "lc", "rc", "mode", "", "")
]

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pg.display.init()

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.cmd_vel_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

        self.js = None
        self.BUTTONS = BUTTONS[0]
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = [0] * len(self.BUTTONS)
        self.lock = threading.Lock()

        self.create_timer(0.1, self.update_buttons)
        threading.Thread(target=self.connect, daemon=True).start()

    def connect(self):
        while True:
            if os.path.exists("/dev/input/js0"):
                with self.lock:
                    if self.js is None:
                        pg.joystick.init()
                        try:
                            self.js = pg.joystick.Joystick(0)
                            self.js.init()

                            if self.js.get_name() == 'SHANWAN Android Gamepad':
                                self.BUTTONS = BUTTONS[0]
                                self.last_buttons = [0] * len(self.BUTTONS)
                            elif self.js.get_name() == 'USB WirelessGamepad':
                                self.BUTTONS = BUTTONS[1]
                                self.last_buttons = [0] * len(self.BUTTONS)
                        except Exception as e:
                            print(e)
                            self.js = None
            else:
                with self.lock:
                    if self.js is not None:
                        self.js.quit()
                        self.js = None
            pg.time.delay(200)

    def axes_callback(self, axes):
        twist = Twist()

        for key in ('0', '1', '2', '3'):
            if abs(axes[key]) < self.min_value:
                axes[key] = 0

        twist.linear.y = val_map(axes['0'], 1, -1, -self.max_linear, self.max_linear)
        twist.linear.x = val_map(axes['1'], 1, -1, -self.max_linear, self.max_linear)
        twist.angular.z = val_map(axes['2'], 1, -1, -self.max_angular, self.max_angular)

        if axes['hat_x'] == 1:
            twist.linear.x = self.max_linear
        elif axes['hat_x'] == -1:
            twist.linear.x = -self.max_linear

        if axes['hat_y'] == 1:
            twist.linear.y = self.max_linear
        elif axes['hat_y'] == -1:
            twist.linear.y = -self.max_linear

        self.cmd_vel_pub.publish(twist)

    def update_buttons(self):
        try:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit(0)

                elif event.type == pg.JOYAXISMOTION:
                    axis_key = AXES_MAP[event.axis]
                    self.last_axes[axis_key] = event.value

                elif event.type == pg.JOYHATMOTION:
                    hat_y, hat_x = event.value
                    self.last_axes['hat_x'] = hat_x
                    self.last_axes['hat_y'] = hat_y

                self.axes_callback(self.last_axes)

        except KeyboardInterrupt:
            pg.quit()
            sys.exit(0)

def main():
    node = JoystickController('joystick_control')
    rclpy.spin(node)


if __name__ == "__main__":
    main()