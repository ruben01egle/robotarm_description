import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robotarm_description')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}] 
    )

    return LaunchDescription([
        rviz_node
    ])