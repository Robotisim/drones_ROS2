#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription , ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    maze_path = os.path.join(get_package_share_directory('drive_tb3'),'models','maze','model.sdf')

    ############# Robot and simulation bringup
    gazebo_rviz_drone_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('drone_basic'), 'launch', 'b_gazebo_rviz_drone.launch.py')
    ),
    )

    ############# Maze Spawn
    maze_spawner = Node(
        package='drive_tb3',
        executable='p6_b_sdf_spawner',
        name='maze_model',
        arguments=[maze_path,"Maze","0.0","0.0"]

    )
    ############# Mapping Functionality
    odom_generator = Node(
        package='sjtu_drone_bringup',
        executable='odom_pub',
        name='odometery'

    )
    ############# Mapping Functionality
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )



    nodes_to_run = [
        gazebo_rviz_drone_bringup,
        maze_spawner,
        odom_generator,
        mapping
    ]

    return LaunchDescription(nodes_to_run)
