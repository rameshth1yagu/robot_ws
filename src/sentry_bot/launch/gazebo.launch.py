import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='sentry_bot'

    # 1. Include the existing robot_state_publisher launch
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','display.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Include the Gazebo launch file provided by ROS 2
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # 3. Run the spawner node to put the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'sentry_bot'],
                        output='screen')

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])