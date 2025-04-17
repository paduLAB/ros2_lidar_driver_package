#ros2 launch lidar_driver delta2cPro_full_launch.py

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Serial Reader Node
        Node(
            package='lidar_driver',
            executable='serial_reader_node.py',
            name='serial_reader_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200},
                {'topic_out': '/serial'}
            ]
        ),

        # Parser and Publisher Node
        Node(
            package='lidar_driver',
            executable='delta2cPro_lidar_node.py',
            name='delta2cPro_lidar_node',
            output='screen',
            parameters=[
                {'topic_in': '/serial'},
                {'topic_out': '/scan2'},
                {'rpm_topic': '/rpm'}
            ]
        )
    ])
