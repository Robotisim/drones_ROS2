from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
import xacro

def generate_launch_description():
    tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')  # set TB3_MODEL to 'burger'

    # Path to Robot's Xacro File
    pkg_path= get_package_share_directory("sjtu_drone_description")
    xacro_file= os.path.join(pkg_path,'urdf','sjtu_drone.urdf.xacro')

    #Processing Xacro File
    xacro_parser=xacro.parse(open(xacro_file))
    xacro.process_doc(xacro_parser)

    #Feeding URDF to ROS
    parameters = {'robot_description': xacro_parser.toxml()}

    #Ros Packages
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[parameters]
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={
            "use_sim_time" : use_sim_time,
            "verbose"      : "true",
        }.items()
    )


    tb3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/spawn_turtlebot3.launch.py']),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hector_Drone',
            '-x', '0.0',  # x coordinate
            '-y', '0.0',  # y coordinate
            '-z', '3.0',  # z coordinate
        ],
    )

    drive_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        namespace="hector_Drone",
        output="screen",
        prefix="xterm -e"
    )

    takeoff_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/hector_Drone/takeoff', 'std_msgs/msg/Empty', '{}'],
        shell=True,
        output="screen"
    )

    #Running all definitions
    nodes_to_run = [
        robot_state_publisher_node,
        gazebo_node,
        tb3_model,  # setting TB3_MODEL environment variable
        spawn_robot_node,
        takeoff_cmd,
        drive_node,
        tb3_spawn,
    ]

    return LaunchDescription(nodes_to_run)
