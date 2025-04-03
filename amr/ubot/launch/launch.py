import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    realsense_launch_file = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch',
            'rs_launch.py'
        )
    )
    
    sllidar_launch_file = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'launch',
            'view_sllidar_a2m12_launch.py'
        )
    )

    return LaunchDescription([
        IncludeLaunchDescription(realsense_launch_file),
        IncludeLaunchDescription(sllidar_launch_file),
        Node(
            package='ubot',
            executable='Ubot_Motor_Controller.py',
            name='node_1',
            output='screen',
            parameters=[{'arg1': 0.15, 'arg2': 0.42}]
        ),
        Node(
            package='ubot',
            executable='Ubot_Obstacle_Avoidance.py',
            name='node_2',
            output='screen'
        ),
        Node(
            package='ubot',
            executable='Ubot_Realsense_Publisher.py',
            name='node_3',
            output='screen'
        ),
        Node(
            package='ubot',
            executable='Ubot_Safety.py',
            name='node_4',
            output='screen'
        ),
        Node(
            package='ubot',
            executable='Ubot_Servo_Subscriber.py',
            name='node_5',
            output='screen'
        ),
        Node(
            package='ubot',
            executable='Ubot_Send_Data.py',
            name='node_6',
            output='screen'
        ),
        Node(
            package='ubot',
            executable='Ubot_Receive_Data.py',
            name='node_7',
            output='screen'
        ),
    ])
