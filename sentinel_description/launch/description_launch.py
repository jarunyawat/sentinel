from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('sentinel_description'),
        'urdf',
        'sentinel.xacro'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}, {'use_sim_time': False}],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)

    return ld