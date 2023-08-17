import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nik_bot'), 'launch'), '/rsp.launch.py'
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    # Launch Gazebo Simulator
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ])
    )
    # Spawn robot in sim
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'nik_bot']
    )
    
    return LaunchDescription([
        rsp_launch,
        gazebo_launch,
        spawn_entity_node
    ])