from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robotarm_description'), 'urdf', 'robotarm.urdf.xacro'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robotarm_description'), 'rviz', 'urdf_config.rviz'])

    # value_type=str keeps the URDF from being YAML-parsed as a parameter value
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str)

    use_rviz = LaunchConfiguration('rviz')
    use_gui = LaunchConfiguration('gui')
    # damping of the damped least squares in calculate_jacobian_inverse (plugin default 0.01)
    damping = ParameterValue(LaunchConfiguration('lambda'), value_type=float)

    # robot_state_publisher turns /joint_states into TF, like in the display test
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # replaces joint_state_publisher_gui: integrates twists on 'cmd_vel' (base frame) into joint
    # positions with the kinematics plugin and publishes /joint_states. Do not run both, they
    # would fight over /joint_states.
    cartesian_jog_node = Node(
        package='robotarm_kinematics',
        executable='cartesian_jog',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'lambda': damping,
        }]
    )

    # six sliders (x, y, z, wx, wy, wz) that publish the twist on cmd_vel
    jog_gui_node = Node(
        package='robotarm_kinematics',
        executable='cartesian_jog_gui',
        output='screen',
        condition=IfCondition(use_gui)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='start the slider GUI that publishes twists on cmd_vel'),
        DeclareLaunchArgument(
            'lambda', default_value='0.01',
            description='damping of the inverse Jacobian: larger is safer near singularities, '
                        'smaller is more accurate'),
        robot_state_publisher_node,
        cartesian_jog_node,
        jog_gui_node,
        rviz_node
    ])
