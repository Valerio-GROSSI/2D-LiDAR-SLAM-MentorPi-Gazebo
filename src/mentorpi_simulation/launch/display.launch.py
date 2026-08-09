import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from launch.actions import LogInfo

COLORS = {'reset': '\033[0m', 'red': '\033[31m', 'green': '\033[32m', 'yellow': '\033[33m', 'blue': '\033[34m', 'magenta': '\033[35m', 'cyan': '\033[36m', 'bold': '\033[1m',}

def display_launch_parameters(parameters):
    cyan = COLORS['cyan']
    green = COLORS['green']
    bold = COLORS['bold']
    reset = COLORS['reset']

    return [LogInfo(msg=(f"{cyan}[launch]{reset} "f"{bold}{name}{reset} = "f"{green}{value}{reset}")) for name, value in parameters.items()]

def launch_setup(context):

    set_gz_ressource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.pathsep, os.path.join(get_package_prefix('mentorpi_simulation'),'share')])
    #export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix mentorpi_simulation)/share

    ##DECLARATION DES PARAMETRES
     
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    headless = (LaunchConfiguration('headless').perform(context).lower() == "true")
    world_config = LaunchConfiguration('world_config').perform(context)

    gz_args = f"--headless-rendering -s -v 4 -r {world_config}" if headless else f"-r {world_config}"

    classes = LaunchConfiguration('classes').perform(context)
    target_class_name = LaunchConfiguration('target_class_name').perform(context)

    prior_model = LaunchConfiguration('model').perform(context)
    # controller = LaunchConfiguration('controller')
    enable_driver_odom_tf = LaunchConfiguration("enable_driver_odom_tf")
    odom0_from_odom_publisher_node = (LaunchConfiguration("odom0_from_odom_publisher_node").perform(context).lower() == "true")
    # enable_real_odom_tf = LaunchConfiguration("enable_real_odom_tf")
    perception_framework = LaunchConfiguration("perception_framework").perform(context).lower()
    scan_matching = (LaunchConfiguration("scan_matching").perform(context).lower() == "true")
    slam_toolbox_database_path = LaunchConfiguration("slam_toolbox_database_path").perform(context)
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path").perform(context)
    amcl_map_path = LaunchConfiguration('amcl_map_path').perform(context)

    auto_coverage = LaunchConfiguration('auto_coverage')
    coverage_params = PathJoinSubstitution([FindPackageShare('mentorpi_simulation'), 'config', 'coverage.yaml'])

    use_sim_value = (LaunchConfiguration("use_sim").perform(context).lower() == "true")
    controller_value = LaunchConfiguration('controller').perform(context)
    enable_driver_odom_tf_value = (LaunchConfiguration("enable_driver_odom_tf").perform(context).lower() == "true")
    enable_real_odom_tf_value = (LaunchConfiguration("enable_real_odom_tf").perform(context).lower() == "true")

    if controller_value == 'wheel_velocity_controller':
        cmd_vel_topic = 'controller/cmd_vel'
        odom0_topic = '/odom_raw'
        model = prior_model
        odom0_from_odom_publisher_node = True
    elif controller_value == 'diff_drive_controller':
        cmd_vel_topic = '/diff_drive_controller/cmd_vel'
        odom0_topic = '/diff_drive_controller/odom'
        model = 'differential'
    elif controller_value == 'mecanum_drive_controller':
        cmd_vel_topic = '/mecanum_drive_controller/reference'
        odom0_topic = '/mecanum_drive_controller/odometry'
        model = 'mecanum'
    else:
        raise RuntimeError(f"Contrôleur non pris en charge : {controller_value}")
    
    if odom0_from_odom_publisher_node:
        odom0_topic = '/odom_raw'

    if not use_sim_value:
        controller_value = 'wheel_velocity_controller'
        cmd_vel_topic = 'controller/cmd_vel'
        odom0_topic = '/odom_raw'
        imu0_topic = '/ros_robot_controller/imu_raw'
        use_gui = True
        odom0_from_odom_publisher_node = True
        enable_real_odom_tf_value = False
        model = prior_model
    else:
        imu0_topic = '/imu_broadcaster/imu'
        use_gui = False

    if enable_driver_odom_tf_value and enable_real_odom_tf_value:
            raise RuntimeError("enable_driver_odom_tf et enable_real_odom_tf ne peuvent etre tous les deux à True")

    parameter_logs = display_launch_parameters({
        "use_sim": use_sim_value,
        "use_gui": use_gui,
        "use_rviz": (use_rviz.perform(context).lower() == "true"),
        "headless": headless,
        "world_config": os.path.basename(world_config),
        "classes": classes,
        "target_class_name": target_class_name,
        "controller": controller_value,
        "model": model,
        "cmd_vel_topic": cmd_vel_topic,
        "kalman_used": (not enable_driver_odom_tf_value and not enable_real_odom_tf_value),
        "odom0_from_odom_publisher_node": odom0_from_odom_publisher_node,
        "odom0_topic": odom0_topic,
        "imu0_topic": imu0_topic,
        "enable_driver_odom_tf": enable_driver_odom_tf_value,
        "enable_real_odom_tf": enable_real_odom_tf_value,
        "perception_framework": perception_framework,
        "scan_matching": scan_matching,
        "slam_toolbox_database_path": slam_toolbox_database_path,
        "rtabmap_database_path": rtabmap_database_path,
        "amcl_map_path": amcl_map_path,
        })

    prior_controller_config_path = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","controller.yaml"])
    controller_config_path = RewrittenYaml(
        source_file=prior_controller_config_path,
        param_rewrites={
            "diff_drive_controller.ros__parameters.enable_odom_tf": enable_driver_odom_tf,
            "mecanum_drive_controller.ros__parameters.enable_odom_tf": enable_driver_odom_tf,
        },
        convert_types=True,
    )

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

    prior_ekf_config_path = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","ekf.yaml"])
    ekf_config_path = RewrittenYaml(
        source_file=prior_ekf_config_path,
        param_rewrites={
            "odom0": odom0_topic,
            "imu0": imu0_topic,
            "publish_tf": "true",
        },
        convert_types=True,
    )

    if scan_matching:
        slam_toolbox_file = "slam_toolbox.yaml"
    else:
        slam_toolbox_file = "slam_toolbox_no_scan_matching.yaml"

    slam_toolbox_config = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config", slam_toolbox_file])
    
    nav2_params = PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"config","nav2_params.yaml"])
    

    ##DECLARATION DES NOEUDS
    
    joystick_control_node = Node(
        package='mentorpi_simulation',
        executable='joystick_control',
        name='joystick_control',
        output='screen',
        parameters=[
            {'max_linear': 0.5,
             'max_angular': 2.0}
        ],
        remappings=[('controller/cmd_vel', cmd_vel_topic)] #controller/cmd_vel /diff_drive_controller/cmd_vel /mecanum_drive_controller/reference
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    ##use_sim == False
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        name="joint_state_publisher_gui",
        condition=IfCondition(str(use_gui).lower()),
    )

    ##use_rviz == True
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    ##use_sim == True
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

    ##use_sim == True
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
                   LaunchConfiguration("z", default="0.10"),
                   "-R",
                   LaunchConfiguration("roll", default="0.00"),
                   "-P",
                   LaunchConfiguration("pitch", default="0.00"),
                   "-Y",
                   LaunchConfiguration("yaw", default="0.00"),],
        output="screen",
        condition=IfCondition(use_sim),
    )

    ##use_sim == True
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        condition=IfCondition(use_sim),
    )

    ##use_sim == False
    camera_node = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("depthai_ros_driver_v3"),
                            "launch",
                            "driver.launch.py",
                        ])
                    ),
                    launch_arguments={
                        "camera_model": "OAK-D-LITE",
                        "enable_depth": "false",
                        "use_rviz": "false",
                    }.items(),
                    condition=UnlessCondition(use_sim), # A faire: étendre en simulation
                )

    ##use_sim == False
    yolov5_node = Node(
        package='yolov5_ros2',
        executable='yolov5_node',
        output='screen',
        parameters=[{'classes': classes}, {'conf': 0.5, 'iou': 0.4}], # robustifier type de classes
        remappings=[('depth_cam/rgb/image_raw', '/oak/rgb/image_raw')],
        condition=UnlessCondition(use_sim), # A faire: étendre en simulation 
    )

    ##use_sim == False
    object_tracker_node = Node(
        package='yolov5_ros2',
        executable='tracking_node',
        output='screen',
        parameters=[{'target_class_name': target_class_name}],
        remappings=[('cmd_vel', cmd_vel_topic)], #controller/cmd_vel /diff_drive_controller/cmd_vel /mecanum_drive_controller/reference 
        condition=UnlessCondition(use_sim), # A faire: étendre en simulation 
    )

    ##use_sim == True
    depth_cam_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_cam_bridge",
        arguments=[
            "/depth_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/depth_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_cam/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/depth_cam/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],        
        output="screen",
        condition=IfCondition(use_sim),
    )

    #joint_state_broadcaster_spawner
    #imu_broadcaster_spawner
    #wheel_velocity_controller_spawner
    #diff_drive_controller_spawner
    #mecanum_drive_controller_spawner

    #joint_state_broadcaster_delayed
    #imu_broadcaster_delayed
    #wheel_velocity_controller_delayed
    #diff_drive_controller_delayed
    #mecanum_drive_controller_delayed

    ##use_sim == True and controller == 'wheel_velocity_controller'
    motors_state_gaz_bridge = Node(
        package="mentorpi_simulation",
        executable="motors_state_gaz_bridge",
        name='motors_state_gaz_bridge',
        output='screen',
        condition=IfCondition(str(use_sim_value and controller_value == 'wheel_velocity_controller').lower()),
    )

    ##use_sim == True and controller == 'mecanum_drive_controller' and enable_driver_odom_tf == True
    mecanum_tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='mecanum_tf_relay',
        arguments=['/mecanum_drive_controller/tf_odometry', '/tf'],
        output='screen',
        condition=IfCondition(str(use_sim_value and controller_value == 'mecanum_drive_controller' and enable_driver_odom_tf_value).lower()),
    )

    ##use_sim == True
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner", # les bon parametres du controleur ont été spécifiés lors de la déclaration des paramètres avec un RewrittenYaml sur prior_controller_config_path
        arguments=["joint_state_broadcaster", "imu_broadcaster", controller_value]  #wheel_velocity_controller diff_drive_controller mecanum_drive_controller
                + ["--controller-manager-timeout", "30"], # redéfinir des paramètres de controller_config_path à ce niveau ne fonctionne pas, d'où l'utilisation d'un RewrittenYaml lors de la déclaration des paramètres
        output="screen",
        condition=IfCondition(use_sim),
        )
    
    ##use_sim == False
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        output='screen',
        condition=UnlessCondition(use_sim),
    )

    ##mandatory if use_sim == False
    odom_publisher_node = Node(
        package="mentorpi_simulation",
        executable="odom_publisher",
        output="screen",
        name="odom_publisher",
        parameters=[{'model': model},
                    {'pub_tf': enable_driver_odom_tf_value and (not use_sim_value or controller_value == "wheel_velocity_controller")}], # si enable_driver_odom_tf=true avec controleur diff/mecanum lancé, les tf publiées sont celles du controleur plutot qu'odom_publisher_node
        remappings=[('cmd_vel', cmd_vel_topic)],  #controller/cmd_vel /diff_drive_controller/cmd_vel /mecanum_drive_controller/reference
        condition=IfCondition(str(not use_sim_value or controller_value == "wheel_velocity_controller" or odom0_from_odom_publisher_node).lower()) # peut etre enlevé pour comparaison des odometries issues du drive controller et d'odom_publisher_node
    ) #odom_raw ros_robot_controller/set_motor

    ##possibility if use_sim == True
    ground_truth_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ground_truth_pose_bridge",
        arguments=["/model/MentorPi/pose@geometry_msgs/msg/Pose[gz.msgs.Pose"],
        output="screen",
        condition=IfCondition(use_sim),
    )

    ##possibility if use_sim == True
    ground_truth_pose_stamped_bridge = Node(
        package="mentorpi_simulation",
        executable="ground_truth_pose_stamped_publisher",
        name="ground_truth_pose_stamped_bridge",
        output="screen",
        condition=IfCondition(use_sim),
    )

    ##use_sim == True and enable_real_odom_tf == True
    ground_truth_tf_odom_bridge = Node(
        package="mentorpi_simulation",
        executable="ground_truth_tf_odom_broadcaster",
        output="screen",
        condition=IfCondition(str(use_sim_value and enable_real_odom_tf_value).lower()),
    )

    ##enable_driver_odom_tf == False and enable_real_odom_tf == False
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node", # les bon parametres de l'ekf ont été spécifiés lors de la déclaration des paramètres avec un RewrittenYaml sur prior_ekf_config_path
        name="ekf_filter_node",
        parameters=[ekf_config_path,
                    ], # redéfinir des paramètres de ekf_config_path à ce niveau ne fonctionne pas, d'où l'utilisation d'un RewrittenYaml lors de la déclaration des paramètres
        output="screen",
        condition=IfCondition(str(not enable_driver_odom_tf_value and not enable_real_odom_tf_value).lower()),
    ) #/odometry/filtered /set_pose

    ##use_sim == True
    lidar_scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_scan_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        output="screen",
        condition=IfCondition(use_sim),
    ) #/scan

    ##use_sim == False
    ld19_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {
                'topic_name': 'scan',
                'product_name': 'LDLiDAR_LD19',
                'port_baudrate': 230400,
                'port_name': '/dev/ttyUSB0',
                'frame_id': 'lidar_frame',
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0
            }
        ],
        remappings=[('scan', 'scan')],
        condition=UnlessCondition(use_sim),
    )

    slam_toolbox_enabled = perception_framework in ('slam_toolbox', 'slam_toolbox_localization',)
    slam_toolbox_localization = (perception_framework == 'slam_toolbox_localization')

    slam_toolbox_mode_parameters = {'mode':'localization' if slam_toolbox_localization else 'mapping',}

    if slam_toolbox_localization:
        slam_toolbox_mode_parameters.update({
        'map_file_name': slam_toolbox_database_path,
        # 'map_start_at_dock': True, #non supported #puis 2D Pose Estimate éventuellement
        # Optionnel si la position initiale est approximativement connue:
        'map_start_pose': [0.0, 0.0, 0.0], #puis 2D ose Estimate éventuellement
    })

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable=('localization_slam_toolbox_node' if slam_toolbox_localization else 'async_slam_toolbox_node'),
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_toolbox_config,
            slam_toolbox_mode_parameters,
        ],
    condition=IfCondition(str(slam_toolbox_enabled).lower()),
    ) # Dans le cas sync_slam_toolbox_node, il s'agit d'un lifecycle node donc lancer ensuite la commande 
      # ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p use_sim_time:=use_sim autostart:=True node_names:="['slam_toolbox_node']"

    # ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '.../src/mentorpi_simulation/maps/slam_toolbox_database/slam_toolbox_database'}"
    # ros2 run nav2_map_server map_saver_cli -f .../src/mentorpi_simulation/maps/map/map

    rtabmap_enabled = perception_framework in ('rtabmap_slam', 'rtabmap_localization')
    rtabmap_localization = (perception_framework == 'rtabmap_localization')

    rtabmap_parameters = {
        # Repères
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        # Base RTAB-Map utilisée par les deux modes
        'database_path': rtabmap_database_path,

        # Entrées RGB-D et LiDAR
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,

        'approx_sync': True,
        'sync_queue_size': 10,
        'use_sim_time': use_sim,

        # 0: Vision (RGB-D), 1: ICP (scan LiDAR), 2: Vision (RGB-D) + ICP (scan LiDAR) combinés
        'Reg/Strategy': '2' if scan_matching else '0',

        # Raffinement LiDAR
        'RGBD/NeighborLinkRefining': 'true' if scan_matching else 'false',

        'RGBD/ProximityPathMaxNeighbors': '5' if scan_matching else '0',

        'RGBD/ProximityOdomGuess': 'true' if scan_matching else 'false',

        # Recherche de fermetures de boucle/proximité
        'RGBD/ProximityBySpace': 'true',

        # ICP adapté à un LaserScan 2D
        'Icp/PointToPlane': 'false',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/Iterations': '30',
        'Icp/VoxelSize': '0.0',
        'Icp/CorrespondenceRatio': '0.2',

        # Robot terrestre
        'Reg/Force3DoF': 'true',

        # Validation des contraintes visuelles
        'Vis/MinInliers': '15',

        # Seuils de création/traitement des signatures
        'RGBD/AngularUpdate': '0.1',
        'RGBD/LinearUpdate': '0.1',

        # Grille d’occupation 2D créée à partir du LiDAR
        'Grid/Sensor': '0',
        'Grid/FromDepth': 'false',
        'Grid/RangeMax': '8.0',
        'Grid/CellSize': '0.05',

        # Publication de map -> odom
        'publish_tf': True,

        'Mem/IncrementalMemory': 'false' if rtabmap_localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if rtabmap_localization else 'false',

        # Le robot redémarre physiquement près de l’endroit où la session précédente s’est terminée == false
        'RGBD/StartAtOrigin': 'true',

        # Facultatif : pose initiale explicitement connue
        # 'initial_pose': '-1 -1 0.0 0.0 0.0 0.50',
    }

    rtabmap_remappings = [
        ('rgb/image', '/depth_cam/image'),
        ('depth/image', '/depth_cam/depth_image'),
        ('rgb/camera_info', '/depth_cam/camera_info'),
        ('scan', '/scan'),
        ('grid_map', '/map'),
        ('initialpose', '/initialpose'),
        # ('odom', '/odometry/filtered'), # si l'on souhaite utiliser cette source pour les tf odom <-> base_footprint
    ]

    # A faire: les bonnes entrées sont à génerer dans le cas use_sim == False
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        arguments=[] if rtabmap_localization else ['-d'],
        condition=IfCondition(str(rtabmap_enabled).lower()),
    )

    # A faire: les bonnes entrées sont à génerer dans le cas use_sim == False
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',

        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            'use_sim_time': use_sim,

            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,

            'approx_sync': True,
            'sync_queue_size': 10,
        }],

        remappings=rtabmap_remappings,

        condition=IfCondition(str(rtabmap_enabled).lower()),
    )

    # ros2 run nav2_map_server map_saver_cli -f .../src/mentorpi_simulation/maps/map/map

    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim,
            'params_file': nav2_params,
            'map': amcl_map_path,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(str(perception_framework == 'amcl').lower()),
    ) 
    # ros2 service call /reinitialize_global_localization \std_srvs/srv/Empty "{}"

    nav2_navigation_launch = GroupAction([
        SetRemap(src="cmd_vel", dst="cmd_vel_nav2"), # voir consequences #controller/cmd_vel /diff_drive_controller/cmd_vel /mecanum_drive_controller/reference
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "navigation_launch.py",
                    ])
                ),
                launch_arguments={
                    "use_sim_time": use_sim, # plus robuste
                    "params_file": nav2_params,
                    "autostart": "true",
                }.items(),
        )
        ])

    delayed_nav2_localization_launch = TimerAction(
        period=5.0,
        actions=[nav2_localization_launch],
    )

    delayed_nav2_navigation_launch = TimerAction(
        period=15.0,
        actions=[nav2_navigation_launch],
    )

    twist_to_twist_stamped_bridge = Node(
        package="mentorpi_simulation",
        executable="twist_to_twist_stamped",
        name="twist_to_twist_stamped_bridge",
        remappings=[("/cmd_vel", "cmd_vel_nav2"),
                    ("/cmd_vel_stamped", cmd_vel_topic)],
        output="screen",
    )
    
    coverage_node = Node(
        package='mentorpi_simulation',
        executable='online_coverage',
        name='online_coverage',
        output='screen',
        parameters=[coverage_params, 
                {'use_sim_time': use_sim,
                'enabled_on_startup': auto_coverage,}
                ],
        condition=IfCondition(auto_coverage),
    )

    delayed_coverage_node = TimerAction(period=20.0, actions=[coverage_node])

    # ros2 service call /coverage/set_enabled std_srvs/srv/SetBool "{data: true}"
    # ros2 service call /coverage/set_enabled std_srvs/srv/SetBool "{data: false}"


    ##DECLARATION DES GROUPES D'ACTIONS

    hardware_group = GroupAction(
        condition=UnlessCondition(use_sim),
        actions=[
            joystick_control_node,
            rsp_node,
            joint_state_publisher_gui_node,
            rviz_node,
            ros_robot_controller_node,
            odom_publisher_node,
            robot_localization_node,
            ld19_node,
            slam_toolbox_node,
            delayed_nav2_localization_launch,
            delayed_nav2_navigation_launch,
            twist_to_twist_stamped_bridge,
        ]
    )

    return [
        set_gz_ressource_path,
        *parameter_logs,

        joystick_control_node,
        rsp_node,
        joint_state_publisher_gui_node,
        rviz_node,
        gz_sim,
        gz_spawn_entity,
        clock_bridge,
        # camera_node,
        # yolov5_node,
        # object_tracker_node,
        depth_cam_bridge,
        motors_state_gaz_bridge,
        mecanum_tf_relay,
        controllers_spawner,
        ros_robot_controller_node,
        odom_publisher_node,
        ground_truth_pose_bridge,
        ground_truth_pose_stamped_bridge,
        ground_truth_tf_odom_bridge,
        robot_localization_node,
        lidar_scan_bridge,
        ld19_node,
        slam_toolbox_node,
        rtabmap_node,
        rtabmap_viz_node,
        delayed_nav2_localization_launch,
        delayed_nav2_navigation_launch,
        twist_to_twist_stamped_bridge,
        delayed_coverage_node,
    ]

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    launch_directory = os.path.dirname(os.path.realpath(__file__))
    default_slam_toolbox_database_path = os.path.normpath(os.path.join(launch_directory,'..','maps','slam_toolbox_database','slam_toolbox_database'))
    default_rtabmap_database_path = os.path.normpath(os.path.join(launch_directory,'..','maps','rtabmap_database','rtabmap_database.db',))
    default_amcl_map_path = os.path.normpath(os.path.join(launch_directory,'..','maps','map','map.yaml',))

    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='true')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"rviz","nav2_default_view.rviz"]))
    declare_headless_arg = DeclareLaunchArgument('headless', default_value='False', description='Run Gazebo Ignition in the headless mode')
    world_config_arg = DeclareLaunchArgument('world_config', default_value=PathJoinSubstitution([FindPackageShare("mentorpi_simulation"),"world","obstacles.sdf"]), description='Path to SDF world file')
    classes_arg = DeclareLaunchArgument('classes', default_value='["person", "cat", "pizza"]', description='List of classes to detect (default: ["person", "cat", "pizza"]), put [] for all classes')
    target_class_name_arg = DeclareLaunchArgument('target_class_name', default_value='person', description='Target class name for object tracking (default: "person")')
    model_arg = DeclareLaunchArgument('model', default_value='differential', choices=['differential','mecanum'], description='Navigation model for odom_publisher_node')
    controller_arg = DeclareLaunchArgument('controller', default_value='diff_drive_controller', choices=['wheel_velocity_controller','diff_drive_controller','mecanum_drive_controller'], description='Controller to load')
    enable_driver_odom_tf_arg = DeclareLaunchArgument('enable_driver_odom_tf', default_value='False', description='Whether the drive controller should publish the odom -> base transform instead of using transforms produced by sensor fusion')
    odom0_from_odom_publisher_node_arg = DeclareLaunchArgument('odom0_from_odom_publisher_node', default_value='False', description='When using robot_localization package, is the odometry source provided by odom_publisher_node or drive controller')
    enable_real_odom_tf_arg = DeclareLaunchArgument('enable_real_odom_tf', default_value='False', description='Whether the Gazebo simulation should publish the odom -> base transform instead of using transforms produced by sensor fusion')
    perception_framework_arg = DeclareLaunchArgument('perception_framework', default_value='slam_toolbox', choices=['rtabmap_slam','rtabmap_localization','slam_toolbox','slam_toolbox_localization','amcl'], description='Perception framework used')
    scan_matching_arg = DeclareLaunchArgument('scan_matching', default_value='True', description='Using scan matching or not (for slam_toolbox and rtabmap)')
    slam_toolbox_database_path_arg = DeclareLaunchArgument('slam_toolbox_database_path', default_value=default_slam_toolbox_database_path, description='Path to the SLAM Toolbox database')
    rtabmap_database_path_arg = DeclareLaunchArgument('rtabmap_database_path', default_value=default_rtabmap_database_path, description='Path to the RTAB-Map database')
    amcl_map_path_arg = DeclareLaunchArgument('amcl_map_path', default_value=default_amcl_map_path, description='Path to the AMCL map file')
    auto_coverage_arg = DeclareLaunchArgument('auto_coverage', default_value='false', choices=['true', 'false'], description='Run online complete coverage with mapping and Nav2')

    return LaunchDescription([
        use_sim_arg,
        use_rviz_arg,
        rviz_config_arg,
        declare_headless_arg,
        world_config_arg,
        classes_arg,
        target_class_name_arg,
        model_arg,
        controller_arg,
        enable_driver_odom_tf_arg,
        odom0_from_odom_publisher_node_arg,
        enable_real_odom_tf_arg,
        perception_framework_arg,
        scan_matching_arg,
        slam_toolbox_database_path_arg,
        rtabmap_database_path_arg,
        amcl_map_path_arg,
        auto_coverage_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        OpaqueFunction(function = launch_setup),
    ])

if __name__ == "__main__":
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()