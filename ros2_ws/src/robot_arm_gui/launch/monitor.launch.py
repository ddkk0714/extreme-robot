"""관제 GUI 런치 — 읽기 전용 모니터.

기동 대상:
  - monitor        : 텔레메트리 노드 + HTTP 서버 (항상)
  - fake_publisher : fake:=true 일 때만 (하드웨어 없는 검증용 가짜 토픽)

`keyboard_teleop` 과 달리 tty 가 필요 없으므로 launch 에 정상적으로 들어간다.

사용 예)
  # 실기 옆에서 (SSH 포트포워딩으로 원격 브라우저 접속)
  ros2 launch robot_arm_gui monitor.launch.py
  #   → 다른 PC 에서:  ssh -L 8088:localhost:8088 <jetson>   후 http://localhost:8088

  # 하드웨어 없이 화면만 검증
  ros2 launch robot_arm_gui monitor.launch.py fake:=true

  # 현장 Wi-Fi 의 아무 기기에서 보기 (⚠️ 포트가 그대로 노출된다)
  ros2 launch robot_arm_gui monitor.launch.py bind:=0.0.0.0

⚠️ 이 노드는 퍼블리셔를 하나도 만들지 않는다(읽기 전용). 계약이 owner 를 정해 둔
   토픽을 발행할 경로가 코드에 존재하지 않는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bind = LaunchConfiguration('bind')
    port = LaunchConfiguration('port')
    fake = LaunchConfiguration('fake')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bind', default_value='127.0.0.1',
            description=('HTTP 바인드 주소. network_mode: host 라 0.0.0.0 으로 열면 '
                         '현장 네트워크에 그대로 노출된다 — 원격은 ssh -L 권장'),
        ),
        DeclareLaunchArgument(
            'port', default_value='8088',
            description='HTTP 포트. 5000/5002/5003/5004(SRT·메타데이터)와 겹치지 않게 고른 값',
        ),
        DeclareLaunchArgument(
            'video_fps', default_value='10.0',
            description='MJPEG 인코딩 상한. 클라이언트 수와 무관하게 프레임당 1회만 인코딩한다',
        ),
        DeclareLaunchArgument(
            'video_quality', default_value='70', description='JPEG 품질 (대역폭 조절)',
        ),
        DeclareLaunchArgument(
            # ⚠️ 'off' 금지 — launch 는 파라미터를 YAML 로 넘기고, YAML 1.1 은
            #    off/on/yes/no 를 불리언으로 바꿔 버려 STRING 파라미터가 깨진다.
            'video_default_source', default_value='none',
            description=('UI 초기 영상 소스(none|debug|raw). 기본 none — 페이지를 여는 것만으로 '
                         'perception_node 에 오버레이 비용이 생기면 안 된다'),
        ),
        DeclareLaunchArgument(
            'web_root', default_value='',
            description='빈값이면 설치된 share/web. 소스 트리를 가리키면 재빌드 없이 HTML 수정 가능',
        ),
        DeclareLaunchArgument(
            'fake', default_value='false',
            description='true 면 가짜 토픽 발행자도 기동 (하드웨어 없는 검증 전용)',
        ),

        Node(
            package='robot_arm_gui', executable='monitor', name='robot_arm_monitor',
            output='screen',
            parameters=[{
                'bind_address': bind,
                'port': port,
                'video_fps': LaunchConfiguration('video_fps'),
                'video_quality': LaunchConfiguration('video_quality'),
                'video_default_source': LaunchConfiguration('video_default_source'),
                'web_root': LaunchConfiguration('web_root'),
            }],
        ),

        Node(
            package='robot_arm_gui', executable='fake_publisher',
            name='robot_arm_gui_fake_publisher', output='screen',
            condition=IfCondition(fake),
        ),
    ])
