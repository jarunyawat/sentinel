import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import UnlessCondition

def generate_launch_description():

    sentinel_nav2_pkg = get_package_share_directory('sentinel_nav2')
    launch_dir = os.path.join(sentinel_nav2_pkg, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    # use_localization = LaunchConfiguration('use_localization')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(sentinel_nav2_pkg, 'config', 'sentinel_nav2_params.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(sentinel_nav2_pkg, 'map', 'home.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # declare_use_localization_cmd = DeclareLaunchArgument(
    #     'use_localization', default_value='True',
    #     description='Whether to enable localization or not'
    # )

    nav2_bringup_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_namespace': use_namespace,
                    'slam': slam,
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'log_level': log_level,
                    # 'use_localization': use_localization
                }.items(),
            )

    collision_monitor_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'collision_monitor_node.launch.py')
                ),
                condition=UnlessCondition(slam),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': use_composition,
                }.items()
            )
    
    keepout_filter_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'keepout.launch.py')
                ),
                condition=UnlessCondition(slam),
                launch_arguments={
                    'namespace': namespace,
                    'mask': os.path.join(sentinel_nav2_pkg, "map", "map_20_keepout.yaml"),
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': use_composition,
                }.items()
            )
    
    speed_limit_filter_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'speed_limit.launch.py')
                ),
                condition=UnlessCondition(slam),
                launch_arguments={
                    'namespace': namespace,
                    'mask': os.path.join(sentinel_nav2_pkg, "map", "map_20_speed_limit.yaml"),
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': use_composition,
                }.items()
            )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # ld.add_action(declare_use_localization_cmd)
    ld.add_action(nav2_bringup_launch)
    # ld.add_action(keepout_filter_launch)
    # ld.add_action(speed_limit_filter_launch)
    ld.add_action(collision_monitor_launch)
    
    return ld