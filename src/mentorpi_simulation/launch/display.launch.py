from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue

#export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix mentorpi_simulation)/share

def launch_setup(context):

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim = LaunchConfiguration('use_sim')

    headless = LaunchConfiguration('headless').perform(context)
    world_config = LaunchConfiguration('world_config').perform(context)

    gz_args = f"--headless-rendering -s -v 4 -r {world_config}" if eval(headless) else f"-r {world_config}"

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

    ekf_config = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","ekf.yaml"])

    slam_toolbox_config = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","slam_toolbox.yaml"])
    
    nav2_params = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","nav2_params.yaml"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"],
        output="screen",
    )

    joint_state_broadcaster_delayed = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster_spawner],
    )

    diff_drive_controller_delayed = TimerAction(
        period=8.0,
        actions=[diff_drive_controller_spawner],
    )

    imu_broadcaster_delayed = TimerAction(
        period=12.0,
        actions=[imu_broadcaster_spawner],
    )

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

    gz_sim = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        ])
                    ),
                    launch_arguments={
                        "gz_args": gz_args,
                        "on_exit_shutdown": "True",
                    }.items(),
                    condition=IfCondition(use_sim),
            )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "MentorPi",
                   "-topic", "robot_description",
                   "-x",
                   LaunchConfiguration("x", default="0.00"),
                   "-y",
                   LaunchConfiguration("y", default="0.00"),
                   "-z",
                   LaunchConfiguration("z", default="0.00"),
                   "-R",
                   LaunchConfiguration("roll", default="0.00"),
                   "-P",
                   LaunchConfiguration("pitch", default="0.00"),
                   "-Y",
                   LaunchConfiguration("yaw", default="0.00"),],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    ground_truth_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ground_truth_pose_bridge",
        arguments=["/model/MentorPi/pose@geometry_msgs/msg/Pose[gz.msgs.Pose"],
        output="screen",
    )

    ground_truth_tf_odom_bridge = Node(
        package="mentorpi_simulation",
        executable="ground_truth_tf_odom_broadcaster",
        output="screen",
    )

    robot_localization_node =Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_config],
        output="screen",
    )

    lidar_scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_scan_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_toolbox_config],
        output="screen",
    )

    slam_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["slam_toolbox"]},
        ],
    )

    slam_lifecycle_manager_delayed = TimerAction(
        period=16.0,
        actions=[slam_lifecycle_manager],
    )

    nav2_launch = GroupAction([
            SetRemap(src="/cmd_vel", dst="/diff_drive_controller/cmd_vel"),
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "navigation_launch.py",
                        ])
                    ),
                    launch_arguments={
                        "use_sim_time": "true",
                        "params_file": nav2_params,
                        "autostart": "true",
                    }.items(),
            )
            ])

    nav2_delayed_launch = TimerAction(
        period=20.0,
        actions=[nav2_launch],
    )

    return [
        rsp_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gz_sim,
        gz_spawn_entity,
        clock_bridge,
        ground_truth_pose_bridge,
        joint_state_broadcaster_delayed,
        diff_drive_controller_delayed,
        imu_broadcaster_delayed,
        robot_localization_node,
        #ground_truth_tf_odom_bridge,
        lidar_scan_bridge,
        slam_toolbox_node,
        slam_lifecycle_manager_delayed,
        nav2_delayed_launch,
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