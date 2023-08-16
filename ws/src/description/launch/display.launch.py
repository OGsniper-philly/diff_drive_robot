import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    nik_bot_description_share_dir = get_package_share_directory("description")
    urdf_path = os.path.join(nik_bot_description_share_dir, 'description/nik_bot.urdf.xacro')

    # Takes in URDF, listens to /joint_states topic, and
    # publishes all robot transforms + copy of URDF to /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )

    # Listens to /robot_description and publishes simulated joint movements on 
    # /joint_states based on an interactive GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])