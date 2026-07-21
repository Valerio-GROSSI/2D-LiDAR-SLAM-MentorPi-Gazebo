from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ros_robot_controller_gen_commands = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller_gen_commands',
        output='screen',
    )

    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        output='screen',
    )

    return LaunchDescription([
        ros_robot_controller_gen_commands,
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
