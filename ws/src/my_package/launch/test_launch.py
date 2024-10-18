import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('my_package'),
      'config',
      'test.yaml'
      )
    return LaunchDescription([
        Node(
            package='my_package',
            executable='dashgo_driver2',
            name='DashgoDriver',
            output='screen',
            emulate_tty=True,
            parameters=[
                # {'my_parameter': 'test'},
                config
            ],
        ),
    ])