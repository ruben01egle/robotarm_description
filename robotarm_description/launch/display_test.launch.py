import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Holt den Installationspfad dieses Pakets
    pkg_share = get_package_share_directory('robotarm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robotarm.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # Startet den robot_state_publisher und füttert ihn mit dem geparsten Xacro
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_file, ' tool:=', LaunchConfiguration('tool')])}]
    )

    # Erzeugt Schieberegler in einem kleinen Extra-Fenster, um die Gelenke in RViz zu drehen
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Startet RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'tool', default_value='none',
            description='Werkzeug am Flansch: Dateiname in urdf/tools/ ohne .xacro'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])