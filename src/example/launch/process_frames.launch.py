
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context):

    classes = LaunchConfiguration('classes').perform(context)
    target_class_name = LaunchConfiguration('target_class_name').perform(context)

    yolov5_node = Node(
        package='yolov5_ros2',
        executable='yolov5_node',
        output='screen',
        parameters=[{'classes': classes}, {'conf': 0.5, 'iou': 0.4}],
    )

    object_tracker_node = Node(
        package='yolov5_ros2',
        executable='tracking_node',
        output='screen',
        parameters=[{'target_class_name': target_class_name}],
    )

    return [
        yolov5_node,
        object_tracker_node
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('classes', default_value='["person", "cat", "pizza"]', description='List of classes to detect (default: ["person", "cat", "pizza"]), put [] for all classes'),
        DeclareLaunchArgument('target_class_name', default_value='person', description='Target class name for object tracking (default: "person")'),
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()