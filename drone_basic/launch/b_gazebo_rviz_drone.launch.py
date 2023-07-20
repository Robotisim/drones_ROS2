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

    gazebo_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    ),
    launch_arguments={
        "use_sim_time" : use_sim_time,
        "verbose"      : "true",
    }.items()
    )

    rviz_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'a_rviz_drone_bringup.launch.py')
        )
    )


    takeoff_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/drone/takeoff', 'std_msgs/msg/Empty', '{}'],
        shell=True,
        output="screen"
    )



    nodes_to_run = [
        gazebo_node,
        rviz_bringup,
        # gazebo_spawn,
        takeoff_cmd
    ]

    return LaunchDescription(nodes_to_run)
