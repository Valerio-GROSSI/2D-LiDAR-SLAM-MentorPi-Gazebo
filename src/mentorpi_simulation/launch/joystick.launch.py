from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    max_linear_arg = DeclareLaunchArgument('max_linear', default_value='0.5')
    max_angular_arg = DeclareLaunchArgument('max_angular', default_value='2.0')
    remap_cmd_vel_arg = DeclareLaunchArgument('remap_cmd_vel', default_value='cmd_vel')

    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    remap_cmd_vel = LaunchConfiguration('remap_cmd_vel')

    # joy_node = Node(
    # package='joy',
    # executable='joy_node',
    # name='joy_node',
    # output='screen',
    # parameters=[{'dev': '/dev/input/js0', 'autorepeat_rate': 20.0}] 
    # )

    joystick_control_node = Node(
        package='mentorpi_simulation',
        executable='joystick_control',
        name='joystick_control',
        output='screen',
        parameters=[
            {'max_linear': max_linear,
             'max_angular': max_angular}
        ],
        remappings=[('controller/cmd_vel', remap_cmd_vel)]
    )

    return LaunchDescription([
        max_linear_arg,
        max_angular_arg,
        remap_cmd_vel_arg,
        # joy_node,  
        joystick_control_node
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()