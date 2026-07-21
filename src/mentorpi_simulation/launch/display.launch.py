import yaml
import tempfile

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

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    odom_tf_true_override = {
            'diff_drive_controller': {'ros__parameters': {'enable_odom_tf': True}},
            'mecanum_drive_controller': {'ros__parameters': {'enable_odom_tf': True}}
        }
    odom_tf_true_override_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(odom_tf_true_override, odom_tf_true_override_file)
    odom_tf_true_override_file.close()
    odom_tf_true_override_path = odom_tf_true_override_file.name

    ekf_config = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","ekf.yaml"])

    slam_toolbox_config = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","slam_toolbox.yaml"])
    
    nav2_params = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","nav2_params.yaml"])
    
    joystick_control_node = Node(
        package='mentorpi_simulation',
        executable='joystick_control',
        name='joystick_control',
        output='screen',
        parameters=[
            {'max_linear': 0.5,
             'max_angular': 2.0}
        ],
        remappings=[('controller/cmd_vel', 'controller/cmd_vel')] # controller/cmd_vel #/mecanum_drive_controller/reference #/diff_drive_controller/cmd_vel
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen",
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    mecanum_tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='mecanum_tf_relay',
        arguments=['/mecanum_drive_controller/tf_odometry', '/tf'],
        output='screen',
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel_velocity_controller", "--controller-manager-timeout", "30"],
        output="screen",
    )

    motors_state_gaz_bridge = Node(
        package="mentorpi_simulation",
        executable="motors_state_gaz_bridge",
        name='motors_state_gaz_bridge',
        output='screen',
    )

    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--controller-manager-timeout", "30"],
        output="screen",
    )

    joint_state_broadcaster_delayed = TimerAction(
        period=12.0,
        actions=[joint_state_broadcaster_spawner],
    )

    diff_drive_controller_delayed = TimerAction(
        period=16.0,
        actions=[diff_drive_controller_spawner],
    )

    mecanum_drive_controller_delayed = TimerAction(
        period=16.0,
        actions=[mecanum_drive_controller_spawner],
    )

    wheel_velocity_controller_delayed = TimerAction(
        period=16.0,
        actions=[wheel_velocity_controller_spawner],
    )

    imu_broadcaster_delayed = TimerAction(
        period=20.0,
        actions=[imu_broadcaster_spawner],
    )

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "imu_broadcaster", "wheel_velocity_controller"]  #wheel_velocity_controller #mecanum_drive_controller #diff_drive_controller
                + (["-p", odom_tf_true_override_path] if False else []) #bool a adapter
                + ["--controller-manager-timeout", "30"],
        output="screen",
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

    odom_publisher_node = Node(
        package="mentorpi_simulation",
        executable="odom_publisher",
        output="screen",
        name="odom_publisher",
        parameters=[{'model': 'differential'},  #a adapter
                    {'pub_tf': True}],  #a adapter
        remappings=[('cmd_vel', 'controller/cmd_vel')]  #controller/cmd_vel #/mecanum_drive_controller/reference #/diff_drive_controller/cmd_vel
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
        parameters=[ekf_config,
                    {'odom0': '/odom_raw'}, #/odom_raw /mecanum_drive_controller/odometry /diff_drive_controller/odom
                    {'imu0': '/imu_broadcaster/imu'}, #/ros_robot_controller/imu_raw /imu_broadcaster/imu 
                    {'publish_tf': True}],# a adapter
        output="screen",
    ) #/odometry/filtered /set_pose

    lidar_scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_scan_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
    ) #/scan

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
        joystick_control_node,
        rsp_node,
        # joint_state_publisher_gui_node,
        rviz_node,
        gz_sim,
        gz_spawn_entity,
        clock_bridge,
        # joint_state_broadcaster_delayed,
        # diff_drive_controller_delayed,
        # mecanum_drive_controller_delayed,
        # imu_broadcaster_delayed,
        controllers_spawner,
        # mecanum_tf_relay,
        odom_publisher_node,
        motors_state_gaz_bridge,
        ros_robot_controller_node,
        # robot_localization_node,
        # ground_truth_pose_bridge,
        # ground_truth_tf_odom_bridge,
        lidar_scan_bridge,
        # slam_toolbox_node,
        # slam_lifecycle_manager_delayed,
        # nav2_delayed_launch,
    ]

    
def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
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