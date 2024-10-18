import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # File paths
    dashgo_driver_config_path = os.path.join(get_package_share_directory('my_package'), 'config', 'my_dashgo_params_imu.yaml') 
    # yocs_velocity_smoother_config_path = os.path.join(get_package_share_directory('dashgo_driver'), 'config', 'yocs_velocity_smoother.yaml')
    # velocity_smoother_launch_path = os.path.join(get_package_share_directory('yocs_velocity_smoother'), 'launch', 'velocity_smoother.launch.py')

    return LaunchDescription([
        # Declare launch arguments
        # DeclareLaunchArgument('node_name', default_value='velocity_smoother'),
        # DeclareLaunchArgument('nodelet_manager_name', default_value='nodelet_manager'),
        # # DeclareLaunchArgument('config_file', default_value=yocs_velocity_smoother_config_path),
        # DeclareLaunchArgument('raw_cmd_vel_topic', default_value='cmd_vel'),
        # DeclareLaunchArgument('smooth_cmd_vel_topic', default_value='smoother_cmd_vel'),
        # DeclareLaunchArgument('robot_cmd_vel_topic', default_value='robot_cmd_vel'),
        # DeclareLaunchArgument('odom_topic', default_value='odom'),

        # Use simulation time parameter
        Node(
            package='my_package',
            executable='dashgo_driver',
            name='dashgo_driver',
            output='screen',
            respawn=True,
            parameters=[{'use_sim_time': False}, dashgo_driver_config_path]
        ),

        # Nodelet manager (nodelet package is deprecated in ROS2, but assuming you're using an equivalent package)
        # Node(
        #     package='nodelet',
        #     executable='nodelet',
        #     name=LaunchConfiguration('nodelet_manager_name'),
        #     arguments=['manager'],
        # ),

        # Include velocity smoother launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(velocity_smoother_launch_path),
        #     launch_arguments={
        #         'node_name': LaunchConfiguration('node_name'),
        #         'nodelet_manager_name': LaunchConfiguration('nodelet_manager_name'),
        #         'config_file': LaunchConfiguration('config_file'),
        #         'raw_cmd_vel_topic': LaunchConfiguration('raw_cmd_vel_topic'),
        #         'smooth_cmd_vel_topic': LaunchConfiguration('smooth_cmd_vel_topic'),
        #         'robot_cmd_vel_topic': LaunchConfiguration('robot_cmd_vel_topic'),
        #         'odom_topic': LaunchConfiguration('odom_topic'),
        #     }.items()
        # ),

        # Static transform publishers for sonar
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar0',
        #     arguments=['0.18', '0.10', '0.115', '0.524', '0.0', '0.0', '/base_footprint', '/sonar0', '40'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar1',
        #     arguments=['0.20', '0.0', '0.115', '0.0', '0.0', '0.0', '/base_footprint', '/sonar1', '40'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar2',
        #     arguments=['0.18', '-0.10', '0.115', '-0.524', '0.0', '0.0', '/base_footprint', '/sonar2', '40'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_sonar3',
        #     arguments=['-0.20', '0.0', '0.115', '3.14', '0.0', '0.0', '/base_footprint', '/sonar3', '40'],
        # ),
    ])