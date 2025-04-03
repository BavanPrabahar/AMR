import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([


        Node(
            package='ubot',
            executable='end.py',
            name='motor_controller_node',
            output='screen',

        ),

 

        Node(
            package='ubot',
            executable='odom_baselink.py',
            name='odom_base',
            output='screen'
        ),
        Node(
            package='ubot',
            executable='revised_rpm.py',
            name='crobot_odom',
            output='screen'
        ),
 
                
    ])


