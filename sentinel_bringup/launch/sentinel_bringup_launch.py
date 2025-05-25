from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

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

    ld = LaunchDescription()
    ld.add_action(description_launch)
    ld.add_action(micro_ros_node)
    ld.add_action(lidar_launch)

    return ld