import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Use xacro to process the URDF file
    pkg_dir = get_package_share_directory('arm_description')
    
    # --- THIS IS THE FIX ---
    # The filename MUST match your file in the urdf folder
    # It was 'arm_description.urdf', I changed it to 'arm_description.urdf.xacro'
    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm_description.urdf')
    
    # # This line processes the .xacro file into a plain URDF XML string
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Get path to RViz config
    # We should create this file for a better experience
    # rviz_config_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Use a default config if 'display.rviz' doesn't exist yet
        # arguments=['-d', rviz_config_file], 
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node
    ])
