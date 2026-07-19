from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context):

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim = LaunchConfiguration('use_sim')

    controller_config_path = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","controller.yaml"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution([
                    FindPackageShare("mentorpi_simulation"),
                    "urdf",
                    "mentorpi.urdf.xacro",
                    ]),
            " use_sim:=",
            use_sim,
            " controller_config_file:=",
            controller_config_path,                          
        ]
    )

    # robot_description = {"robot_description": robot_description_content}

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return [
        rsp_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    
def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"rviz","config.rviz"]))
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')
    declare_headless_arg = DeclareLaunchArgument('headless', default_value='False', description='Run Gazebo Ignition in the headless mode')
    world_config_arg = DeclareLaunchArgument('world_config', 
                                             default_value=PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"world","empty_with_plugins_obstacles.sdf"]),
                                             description='Path to SDF world file')
    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_sim_arg,
        declare_headless_arg,
        world_config_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        OpaqueFunction(function = launch_setup),
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()