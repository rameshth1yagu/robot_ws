import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Setup paths
    pkg_path = os.path.join(get_package_share_directory('sentry_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')

    # 2. Process Xacro to URDF
    robot_description_config = Command(['xacro ', xacro_file])
    
    # 3. Create the Robot State Publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 4. Create the Joint State Publisher GUI node
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 5. Create the RViz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # Launch everything!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])