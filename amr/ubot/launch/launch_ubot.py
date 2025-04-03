import launch
from launch import LaunchDescription
from launch_ros.actions import Node  
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ubot',
            executable='Ubot_Local_Motor_Controller.py',
            name='node1',
            output='screen'
        ),
    ])

