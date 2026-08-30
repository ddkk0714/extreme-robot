"""Manual interchangeable-tool selection for hardware and mock validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tool_type = LaunchConfiguration('tool_type')
    profile_file = LaunchConfiguration('tool_profile_file')
    mock_mode = LaunchConfiguration('mock_mode')
    read_only = LaunchConfiguration('read_only')
    control_scope = LaunchConfiguration('control_scope')
    gripper_tolerance = LaunchConfiguration('gripper_target_tolerance_ticks')
    cleaner_joint = LaunchConfiguration('cleaning_actuator_joint')
    cleaner_id = LaunchConfiguration('cleaning_actuator_id')
    cleaner_direction = LaunchConfiguration('cleaning_direction')
    cleaner_velocity = LaunchConfiguration('cleaning_velocity_raw')
    common = {
        'tool_type': tool_type,
        'tool_profile_file': profile_file,
    }
    return LaunchDescription([
        DeclareLaunchArgument('tool_type', default_value='spur_1motor_gripper'),
        DeclareLaunchArgument(
            'tool_profile_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('dynamixel_control'), 'config',
                'tool_profiles.yaml'])),
        DeclareLaunchArgument('mock_mode', default_value='false'),
        DeclareLaunchArgument('read_only', default_value='false'),
        DeclareLaunchArgument('control_scope', default_value='FULL_ROBOT'),
        DeclareLaunchArgument(
            'gripper_target_tolerance_ticks', default_value='20'),
        DeclareLaunchArgument('cleaning_actuator_joint', default_value=''),
        DeclareLaunchArgument('cleaning_actuator_id', default_value='-1'),
        DeclareLaunchArgument('cleaning_direction', default_value='0'),
        DeclareLaunchArgument('cleaning_velocity_raw', default_value='0'),
        Node(
            package='dynamixel_control',
            executable='moveit_dynamixel_bridge', output='screen',
            parameters=[common, {
                'mock_mode': ParameterValue(mock_mode, value_type=bool),
                'read_only': ParameterValue(read_only, value_type=bool),
                'control_scope': control_scope,
                'gripper_target_tolerance_ticks': ParameterValue(
                    gripper_tolerance, value_type=int),
                'cleaning_actuator_joint': cleaner_joint,
                'cleaning_actuator_id': ParameterValue(cleaner_id, value_type=int),
                'cleaning_direction': ParameterValue(
                    cleaner_direction, value_type=int),
                'cleaning_velocity_raw': ParameterValue(
                    cleaner_velocity, value_type=int),
            }],
        ),
        Node(
            package='dynamixel_control', executable='arm_fsm', output='screen',
            parameters=[common, {
                'dry_run_mode': ParameterValue(mock_mode, value_type=bool),
                'sensor_mock_mode': ParameterValue(mock_mode, value_type=bool),
                'vla_standalone_mode': ParameterValue(mock_mode, value_type=bool),
                'mock_contact': True, 'mock_distance': 1.0,
                'mock_lock_confirmed': True,
                'cleaning_actuator_joint': cleaner_joint,
                'cleaning_start_time': 0.05,
                'clean_duration': 0.1,
                'locked_dwell': 0.0,
            }],
        ),
    ])
