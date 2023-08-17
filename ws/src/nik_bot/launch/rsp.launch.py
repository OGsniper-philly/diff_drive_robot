import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nik_bot_share_dir = get_package_share_directory("nik_bot")
    urdf_path = os.path.join(nik_bot_share_dir, 'description/nik_bot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true')

    # Robot State Publisher: takes URDF file and listens to /joint_states topic, publishes 
    # static and dynamic robot transforms and a copy of URDF to /robot_description
    #   :param use_sim_time (bool, false): Use sim time if true
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', urdf_path])},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        # Launch Arguments:
        use_sim_time_arg,
        # Nodes:
        robot_state_publisher_node,
    ])