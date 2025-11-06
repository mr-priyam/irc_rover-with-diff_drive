from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('irc_rover')
    urdf_path = os.path.join(pkg_share, 'urdf', 'irc_rover.urdf.xacro')

    # ✅ Read the URDF file contents
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at {urdf_path}")

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ✅ Get launch arguments
    use_gui = LaunchConfiguration('use_joint_state_publisher_gui').perform(context).lower() == 'true'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    # ✅ Node: joint_state_publisher or joint_state_publisher_gui
    if use_gui:
        joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    else:
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )

    # ✅ Node: robot_state_publisher (handles TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ✅ Node: rviz2 (optional config)
    rviz_args = []
    if rviz_config and os.path.exists(rviz_config):
        rviz_args = ['-d', rviz_config]

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    # Return all nodes to launch
    return [joint_state_publisher, robot_state_publisher, rviz2]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui',
            default_value='true',
            description='Whether to use joint_state_publisher_gui (shows joint sliders)'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Path to an optional RViz2 config file'
        ),
        OpaqueFunction(function=_launch_setup)
    ])
