import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'true')

    amcl_yaml = os.path.join(get_package_share_directory('nav04_amcl_localization'),
        'params', 'amcl.yaml')
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml]
    )
    manager_localization_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['amcl']}]
    )
    map_server_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=([get_package_share_directory("mycar_map_server"),"/launch/map_server.launch.py"])
        )
    )
    return LaunchDescription([use_sim_time_arg,amcl_node,manager_localization_node,map_server_launch])
