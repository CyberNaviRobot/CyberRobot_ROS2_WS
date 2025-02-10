import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('sensor04_odom_kc'),
        'params',
        'odom_kc_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='sensor04_odom_kc',
            executable='odom_kc',
            name='Odom_KC_Node',  # 必须与YAML中的键一致
            parameters=[params_file]
            # arguments=['--ros-args', '--log-level', 'debug']
        )
    ])