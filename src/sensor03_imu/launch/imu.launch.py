import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('sensor03_imu'),
        'params',
        'imu_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='sensor03_imu',
            executable='imu_node',
            name='ImuNode_node_cpp',  # 必须与YAML中的键一致
            parameters=[params_file]
        )
    ])