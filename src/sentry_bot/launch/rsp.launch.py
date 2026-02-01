import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from rclpy.parameter import ParameterValue

def generate_launch_description():

    # 1. SETUP: Define where our package and xacro file are
    # This automatically finds the folder, no matter where you assume it is.
    pkg_name = 'sentry_bot'
    file_subpath = 'description/robot.urdf.xacro'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. CONFIGURATION: Allow us to turn on "Sim Time" for later (Phase 5)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 3. NODE: Create the 'robot_state_publisher' node
    # This is the node that broadcasts your URDF to the TF tree.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_raw, value_type=str),
             'use_sim_time': use_sim_time}]
    )

    # 4. LAUNCH: Return the description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
            
        node_robot_state_publisher
    ])