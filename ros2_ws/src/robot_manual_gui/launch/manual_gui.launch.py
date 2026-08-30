"""Launch the manual GUI, optionally with exactly one bridge/FSM stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock_mode = LaunchConfiguration('mock_mode')
    read_only = LaunchConfiguration('read_only')
    start_stack = LaunchConfiguration('start_stack')
    tool_type = LaunchConfiguration('tool_type')
    control_scope = LaunchConfiguration('control_scope')
    gripper_tolerance = LaunchConfiguration('gripper_target_tolerance_ticks')
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dynamixel_control'), 'launch',
            'interchangeable_tool.launch.py'])),
        launch_arguments={
            'mock_mode': mock_mode,
            'read_only': read_only,
            'tool_type': tool_type,
            'control_scope': control_scope,
            'gripper_target_tolerance_ticks': gripper_tolerance,
        }.items(),
        condition=IfCondition(start_stack),
    )
    return LaunchDescription([
        DeclareLaunchArgument('mock_mode', default_value='true'),
        DeclareLaunchArgument(
            'read_only', default_value='true',
            description='Hardware launch defaults to no actuator writes.'),
        DeclareLaunchArgument(
            'start_stack', default_value='true',
            description='Set false when bridge/FSM are already running.'),
        DeclareLaunchArgument(
            'tool_type', default_value='spur_1motor_gripper'),
        DeclareLaunchArgument(
            'control_scope', default_value='FULL_ROBOT',
            description='FULL_ROBOT or explicit END_EFFECTOR_ONLY test scope.'),
        DeclareLaunchArgument(
            'gripper_target_tolerance_ticks', default_value='20'),
        stack,
        Node(
            package='robot_manual_gui', executable='manual_gui', output='screen',
            parameters=[{
                'mock_mode': ParameterValue(mock_mode, value_type=bool),
                'tool_type': tool_type,
                'control_scope': control_scope,
            }],
        ),
    ])
