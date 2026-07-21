import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectInfo, ObjectsInfo
import queue
from std_srvs.srv import Trigger
import signal
import time

class ObjectTrackerNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.msg_queue = queue.Queue(maxsize=1)
        signal.signal(signal.SIGINT, self.shutdown)
        self.target_class_name = self.get_parameter('target_class_name').value
        self.create_service(Trigger, 'object_tracker/start', self.start_srv_callback)
        self.create_service(Trigger, 'object_tracker/stop', self.stop_srv_callback)
        self.objects_sub = self.create_subscription(ObjectsInfo, 'yolov5/object_detect', self.msg_callback, 1)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        threading.Thread(target=self.object_tracker_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

    def get_node_state(self, request, response):
        response.success = True
        return response
    
    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start object tracker")

        self.start = True
        response.success = True
        response.message = "start"
        return response
    
    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop object tracker")

        self.start = False
        response.success = True
        response.message = "start"
        return response

    def msg_callback(self, msg):
        if self.msg_queue.full():
            self.msg_queue.get_nowait()
        self.msg_queue.put_nowait(msg)

    def object_tracker_callback(self):
        while self.running:
            try:
                msg = self.msg_queue.get(block=True, timeout=1.0)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                if self.start:
                    objects = msg.objects

                    object = next(
                        (obj for obj in objects if obj.class_name == self.target_class_name),
                        None
                    )

                    if not object:
                        # self.get_logger().info(f"Target object '{self.target_class_name}' not found.")
                        twist = Twist()
                        self.twist_pub.publish(twist)
                        continue
                    
                    box = object.box
                    width = object.width
                    height = object.height

                    twist = self.follow_object(box, width, height)

                    self.twist_pub.publish(twist)

                else:
                    time.sleep(0.01)
            except BaseException as e:
                print('error', e)
        
        else:
            time.sleep(0.01)
        rclpy.shutdown()

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def follow_object(self, box, width, height):

        twist = Twist()

        x_min, y_min, x_max, y_max = box

        c_x = (x_min + x_max) / 2.0
        c_y = (y_min + y_max) / 2.0

        image_center_x = width / 2.0
        image_center_y = height / 2.0

        error_x = (c_x - image_center_x) / image_center_x

        box_width = x_max - x_min
        box_height = y_max - y_min
        box_area = box_width * box_height

        area_ratio = box_area / (width * height)

        angular_gain = 1.5
        max_angular_speed = 1.5

        forward_speed = 0.25
        target_area_ratio = 0.20

        horizontal_dead_zone = 0.05

        if abs(error_x) > horizontal_dead_zone:
            twist.angular.z = -angular_gain * error_x
            twist.angular.z = max(-max_angular_speed, min(max_angular_speed, twist.angular.z))

        if area_ratio < target_area_ratio:
            twist.linear.x = forward_speed

        return twist

def main():
    rclpy.init()
    node = ObjectTrackerNode('object_tracker')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()