from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('sllidar_ros2'),
        #             'launch',
        #             'sllidar_a1.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'serial_port': '/dev/ttyUSB0',
        #         'frame_id': 'laser',
        #         'inverted':'true',
        #     }.items()
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link',
            parameters=[
                "0.1", "0.0", "0.2", "-1.57", "0.0", "0.0", "/base_link", "/laser", "40"
            ],
        ),
    ])