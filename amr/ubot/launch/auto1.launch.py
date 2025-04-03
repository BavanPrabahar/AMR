import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sllidar_ros2'),
                    'launch',
                    'view_sllidar_a2m12_launch.py'
                )
            )
        ),


        Node(
            package='ubot',
            executable='diff_motor_controller_2.py',
            name='motor_controller_node',
            output='screen',

        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.4',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.140',
                '--frame-id', 'base_link', '--child-frame-id', 'laser'
            ],
            output='screen'
        ),
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
    ])


