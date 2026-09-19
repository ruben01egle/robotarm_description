from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robotarm_description'), 'urdf', 'robotarm.urdf.xacro'])

    # value_type=str keeps the URDF from being YAML-parsed as a parameter value
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str)

    kinematics_core_test_node = Node(
        package='robotarm_kinematics',
        executable='kinematics_core_test',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        kinematics_core_test_node
    ])
