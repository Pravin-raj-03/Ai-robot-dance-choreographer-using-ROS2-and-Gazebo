import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_launch_dir = os.path.join(
        get_package_share_directory('ros_gz_robot_bringup'), 'launch')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo_sim.launch.py')))

    dance_gui = Node(
        package='robot_dance',
        executable='dance_gui',
        name='dance_gui',
        output='screen',
    )

    return LaunchDescription([gazebo, dance_gui])
