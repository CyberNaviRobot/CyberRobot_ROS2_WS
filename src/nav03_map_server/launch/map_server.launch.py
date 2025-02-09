import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():

  use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'true')
  map_file = os.path.join('map', 'my_map.yaml')
  map_server_node = Node(
      package='nav2_map_server',
      executable='map_server',
      name='map_server',
      output='screen',
      parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},        #是否启用仿真时间
                  {'yaml_filename':map_file}]
  )
  manager_mapper_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_mapper',
    output='screen',
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},   #是否启用仿真时间
      {'autostart': True},
      {'node_names': ['map_server']}]
  )
  return LaunchDescription([use_sim_time_arg,map_server_node,manager_mapper_node])