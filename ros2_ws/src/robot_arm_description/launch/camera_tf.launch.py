"""카메라 static TF 발행 — 전방 RGB-D(RealSense D435i).

perception_node 는 pyrealsense2 를 직접 쓰므로(realsense-ros 드라이버 미사용) 아무도
TF 를 내지 않는다. /pick_target 의 frame_id='camera_color_optical_frame' 을 MoveIt 이
base_link(planning frame)로 변환하려면 이 체인이 TF 트리에 있어야 한다 (Phase3 §6-E).

전방 RGB-D (차체 고정):
  base_link ──(CAD 오프셋)──▶ camera_link
  camera_link ──(REP-103 optical 회전, 고정)──▶ camera_color_optical_frame

손목 RGB (그리퍼 위, 팔에 장착):
  2026-07-31부로 URDF 관절(robot_arm.urdf의 link_036→link_051/052→wrist_camera_link)로
  통합됨 — robot_state_publisher가 팔 자세에 따라 동적으로 발행하므로 여기서 static TF를
  더 이상 내지 않는다(같은 프레임을 두 곳에서 발행하면 TF 트리 충돌). robot_state_publisher가
  뜬 launch(display.launch.py 등)를 같이 켜야 wrist_camera_link가 TF에 나타난다.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# REP-103 optical frame: body(X전방·Y좌·Z상) → optical(Z전방·X우·Y하)
OPTICAL_ROLL = -math.pi / 2.0
OPTICAL_PITCH = 0.0
OPTICAL_YAW = -math.pi / 2.0


def generate_launch_description():
    args = [
        # ── 전방 RGB-D 카메라 (차체 고정, CAD 실측값) ──
        DeclareLaunchArgument('cam_x',     default_value='0.123'),
        DeclareLaunchArgument('cam_y',     default_value='0.0'),
        DeclareLaunchArgument('cam_z',     default_value='0.082'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='-0.26'),
        DeclareLaunchArgument('cam_yaw',   default_value='0.0'),
    ]

    # ── 전방 RGB-D: base_link → camera_link ──
    front_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 전방 RGB-D: camera_link → camera_color_optical_frame (REP-103 고정) ──
    front_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll',  str(OPTICAL_ROLL),
            '--pitch', str(OPTICAL_PITCH),
            '--yaw',   str(OPTICAL_YAW),
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )

    return LaunchDescription(args + [front_mount_tf, front_optical_tf])
