#!/usr/bin/env python3
# coding=utf8
 
import os
import sys
import threading
import rclpy
import pygame as pg
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
 
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
        self.declare_parameter('frame_id', 'base_link')
 
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.frame_id = self.get_parameter('frame_id').value
 
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'controller/cmd_vel', 1)
 
        self.js = None
        self.BUTTONS = BUTTONS[0]
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = [0] * len(self.BUTTONS)
        self.lock = threading.Lock()
 
        self._running = True
 
        # Boucle de lecture manette indépendante de l'horloge ROS (sim ou pas),
        # pour ne pas dépendre de /clock qui pourrait ne pas avancer.
        threading.Thread(target=self.update_buttons_loop, daemon=True).start()
        threading.Thread(target=self.connect, daemon=True).start()
 
    def connect(self):
        while self._running:
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
            pg.time.delay(50)
 
    def axes_callback(self, axes):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id
 
        for key in ('0', '1', '2', '3'):
            if abs(axes[key]) < self.min_value:
                axes[key] = 0
 
        twist_stamped.twist.linear.y = val_map(axes['0'], 1, -1, -self.max_linear, self.max_linear)
        twist_stamped.twist.linear.x = val_map(axes['1'], 1, -1, -self.max_linear, self.max_linear)
        twist_stamped.twist.angular.z = val_map(axes['2'], 1, -1, -self.max_angular, self.max_angular)
 
        if axes['hat_x'] == 1:
            twist_stamped.twist.linear.x = self.max_linear
        elif axes['hat_x'] == -1:
            twist_stamped.twist.linear.x = -self.max_linear
 
        if axes['hat_y'] == 1:
            twist_stamped.twist.linear.y = self.max_linear
        elif axes['hat_y'] == -1:
            twist_stamped.twist.linear.y = -self.max_linear
 
        self.cmd_vel_pub.publish(twist_stamped)
 
    def update_buttons_loop(self):
        while self._running and rclpy.ok():
            self.update_buttons()

            with self.lock:
                axes = self.last_axes.copy()

            # Publication régulière à 10 Hz,
            # même si le joystick ne change pas de position
            self.axes_callback(axes)

            pg.time.delay(100)


    def update_buttons(self):
        try:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self._running = False
                    return

                elif event.type == pg.JOYAXISMOTION:
                    if event.axis < 4:
                        axis_key = AXES_MAP[event.axis]

                        with self.lock:
                            self.last_axes[axis_key] = event.value

                elif event.type == pg.JOYHATMOTION:
                    hat_x, hat_y = event.value

                    with self.lock:
                        self.last_axes['hat_x'] = hat_x
                        self.last_axes['hat_y'] = hat_y

        except KeyboardInterrupt:
            self._running = False
 
def main():
    node = JoystickController('joystick_control')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()