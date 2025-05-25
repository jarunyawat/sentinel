from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    rvizconfig = os.path.join(
            get_package_share_directory('sentinel_description'),
            'config',
            'model_display.rviz',
        )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizconfig]
        )

    ld = LaunchDescription()
    ld.add_action(rviz)

    return ld