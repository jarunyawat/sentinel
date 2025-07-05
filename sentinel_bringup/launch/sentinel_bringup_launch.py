from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    ekf_params_file = os.path.join(get_package_share_directory('sentinel_bringup'), 'config/ekf_local.yaml')

    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=ekf_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    description_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('sentinel_description'),
            'launch',
            'description_launch.py'
        ])
    )
    
    micro_ros_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '1500000'],
    )

    lidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('sllidar_ros2'),
            'launch',
            'sllidar_c1_launch.py'
        ])
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_local',
        output='screen',
        remappings=[
            ("odometry/filtered", "odom"),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        parameters=[params_file, {'use_sim_time': False}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(description_launch)
    ld.add_action(micro_ros_node)
    ld.add_action(lidar_launch)
    ld.add_action(robot_localization)

    return ld