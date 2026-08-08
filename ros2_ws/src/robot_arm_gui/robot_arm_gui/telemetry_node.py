#!/usr/bin/env python3
"""관제 GUI 텔레메트리 노드 — **읽기 전용**.

브라우저 한 페이지에서 서보 진단(전류·온도·트립 여유)·관절 상태·FSM/계약
상태·YOLO 인식·텔레옵 현황을 동시에 보기 위한 노드다. 지금은 이 정보들이
서로 다른 터미널 로그에 흩어져 있어서, 현장에서 "왜 안 움직이지?"를 찾는 데
매번 여러 창을 뒤져야 한다.

## ⚠️ 이 노드는 퍼블리셔를 하나도 만들지 않는다

계약이 owner 를 못 박아 둔 토픽들(`/arm_status`·`/chassis_mode`·
`/arrival_status`·`/detected_objects`·`/joint_states`)은 물론, 하드웨어 상태를
바꾸는 어떤 토픽도 발행하지 않는다. 특히 `/arm_status` 는 발행 경로가 둘이 되면
header.stamp 가 역행할 수 있고, 그러면 파워트레인이 **영구 latch** 를 건다
(프로세스 재시작 전까지 해제 불가).

`/dynamixel/goal_position` 은 **구독**한다 — 계약이 금지하는 건 발행이지 구독이
아니고, 목표 대비 오차를 관측할 유일한 정직한 경로다.

파라미터도 **읽기만** 한다. 특히 `teleop_core.publish_rate_hz` 는 타이머 주기가
생성자에서 고정되는데 dt 계산만 런타임에 다시 읽으므로, `ros2 param set` 하면
조그 속도가 통째로 틀어진다 — 절대 건드리지 않는다.

## 스레드 배치

`rclpy.spin()` 이 메인 스레드, HTTP 서버가 daemon 스레드다.
`vision_test_node` 는 반대로(`while rclpy.ok()` + `spin_once(0.0)`) 되어 있는데,
그건 `cv2.imshow`(GTK)가 메인 스레드를 요구해서 생긴 구조라 여기엔 해당하지
않는다. 블록하는 쪽(HTTP)을 스레드로 빼는 편이 단순하고 바쁜 대기도 없다.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from control_msgs.msg import JointJog
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Image, JointState, Joy
from std_msgs.msg import Bool, Int32MultiArray, String

from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, ChassisMode
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray

# 계약 어휘의 단일 출처. 여기 없는 값을 GUI 가 새로 만들면 화면과 실제 게이트가
# 어긋난다 — 파워트레인 contract.py 와 짝인 파일을 그대로 읽는다.
from dynamixel_control.contract import (
    DRIVE_READY_STATUSES, HEARTBEAT_TIMEOUT_S, LOCK_MODES, MODE_MISSION_STOP,
)
from dynamixel_control.qos_profiles import ARRIVAL_QOS, HEARTBEAT_QOS

from . import system_stats
from .http_server import serve_forever_in_thread
from .hw_error_parse import parse_hardware_error
from .state_store import StateStore
from .topic_health import DEFAULT_STALE_AFTER, STALE_AFTER
from .video_hub import SOURCES, VideoHub


#: transient_local 발행자에 붙으려면 구독자도 durability 를 맞춰야 한다.
#: (안 맞추면 연결 자체가 안 되고, 늦게 뜬 GUI 가 마지막 값을 못 받는다.)
LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: `dynamixel_position_node` 기본값(모터가 없거나 파라미터 조회 실패 시 폴백).
FALLBACK_MOTOR_IDS = [11, 14, 13, 12, 16, 3]
FALLBACK_JOINT_NAMES = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3',
                        'arm_joint_4', 'arm_joint_5', 'gripper_left_pinion_joint']

#: `joystick_teleop` 버튼 인덱스 기본값(실측 확정: 데드맨 9, 나머지는 미배선 -1).
#: 노드가 안 떠 있으면 이 값을 쓰되 화면에는 '기본값 가정'을 붙인다.
FALLBACK_JOY_BUTTONS = {'deadman': 9, 'turbo': -1, 'estop': -1}


def _obj_to_dict(obj):
    """DetectedObject → JSON 직렬화 가능한 dict.

    `pose.position.z == 0.0` 은 **깊이 없음** 규약이다(perception_node 가
    depth 를 못 얻으면 0 을 채운다). "원점에 있다"로 읽으면 안 된다.
    """
    p = obj.pose.position
    return {
        'class_id': int(obj.class_id),
        'class_name': obj.class_name,
        'confidence': round(float(obj.confidence), 3),
        'position': [round(float(p.x), 4), round(float(p.y), 4), round(float(p.z), 4)],
        'has_depth': float(p.z) != 0.0,
        'bbox': [int(obj.bbox.x_offset), int(obj.bbox.y_offset),
                 int(obj.bbox.width), int(obj.bbox.height)],
    }


class TelemetryNode(Node):

    def __init__(self):
        super().__init__('robot_arm_monitor')

        # ── HTTP ──────────────────────────────────
        # 기본은 localhost. network_mode: host 라 0.0.0.0 으로 열면 곧바로 현장
        # 네트워크에 노출된다 — 원격은 `ssh -L 8088:localhost:8088` 을 권장한다.
        self.declare_parameter('bind_address', '127.0.0.1')
        self.declare_parameter('port', 8088)
        self.declare_parameter('web_root', '')        # 빈값 = 설치된 share/web

        # ── 영상 ──────────────────────────────────
        self.declare_parameter('video_fps', 10.0)     # 인코딩 상한(클라이언트 수 무관)
        self.declare_parameter('video_quality', 70)   # JPEG 품질
        # UI 초기 선택값: 'none' | 'debug' | 'raw'.
        # ⚠️ 'off' 를 쓰면 안 된다 — launch 가 파라미터를 YAML 로 넘기는데 YAML 1.1 은
        #    off/on/yes/no 를 **불리언으로 강제 변환**한다. 실제로 여기서
        #    InvalidParameterTypeException(BOOL vs STRING)이 나서 노드가 죽었다.
        self.declare_parameter('video_default_source', 'none')

        # ── 표시 임계 ─────────────────────────────
        self.declare_parameter('warn_temp_c', 60.0)
        self.declare_parameter('warn_current_ratio', 0.7)

        # ── 관측 대상 노드 이름(파라미터 읽기용) ──
        self.declare_parameter('driver_node', 'dynamixel_position_node')
        self.declare_parameter('joystick_node', 'joystick_teleop')

        self.warn_temp_c = float(self.get_parameter('warn_temp_c').value)
        self.warn_current_ratio = float(self.get_parameter('warn_current_ratio').value)
        self._driver_node = self.get_parameter('driver_node').value
        self._joystick_node = self.get_parameter('joystick_node').value

        self.store = StateStore()
        self.video = VideoHub(fps=float(self.get_parameter('video_fps').value),
                              quality=int(self.get_parameter('video_quality').value))
        self._video_subs = {}          # source -> Subscription (동적으로 생성/파괴)
        self._motor_names_resolved = False
        self._joy_params_resolved = False
        self._param_clients = {}

        try:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        except ImportError:                                   # pragma: no cover
            self._bridge = None
            self.get_logger().error('cv_bridge 없음 — 영상 패널이 비활성화된다')

        self._subscribe_all()

        # 2Hz 조정 타이머: 영상 구독 생성/파괴, 발행자 조회, 파라미터 조회, 자원 지표.
        # rclpy 엔티티 조작은 HTTP 스레드가 아니라 여기(실행기 스레드)에서 한다.
        self._reconcile_timer = self.create_timer(0.5, self._reconcile)

        self.store.set_motor_names(dict(zip(FALLBACK_MOTOR_IDS, FALLBACK_JOINT_NAMES)))
        self.store.set_joy_params(**FALLBACK_JOY_BUTTONS, resolved=False)

        self._server = None
        self._start_http()

    # ------------------------------------------------------------ 구독
    def _subscribe_all(self):
        n = self
        # 서보 드라이버 (position_node 경로)
        n.create_subscription(Int32MultiArray, '/dynamixel/state', self._on_dxl_state, 10)
        n.create_subscription(String, '/dynamixel/hardware_error', self._on_hw_error, 10)
        n.create_subscription(Int32MultiArray, '/dynamixel/goal_position', self._on_goal, 10)
        n.create_subscription(Int32MultiArray, '/dynamixel/tick_limits',
                              self._on_tick_limits, LATCHED)
        n.create_subscription(Int32MultiArray, '/dynamixel/current_trip_config',
                              lambda m: self._on_threshold('trip', m), 10)
        n.create_subscription(Int32MultiArray, '/dynamixel/current_spike_config',
                              lambda m: self._on_threshold('spike', m), 10)
        # MoveIt 브릿지 경로
        n.create_subscription(Bool, '/dynamixel/controller_fault',
                              self._on_controller_fault, 10)
        n.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        # 파워트레인 계약
        n.create_subscription(ArmStatus, '/arm_status', self._on_arm_status, HEARTBEAT_QOS)
        n.create_subscription(ChassisMode, '/chassis_mode', self._on_chassis, HEARTBEAT_QOS)
        n.create_subscription(ArrivalStatus, '/arrival_status', self._on_arrival, ARRIVAL_QOS)
        # 인식
        n.create_subscription(DetectedObjectArray, '/detected_objects',
                              self._on_detections, 10)
        n.create_subscription(DetectedObject, '/pick_target', self._on_pick_target, LATCHED)
        # 텔레옵 (프론트엔드가 보내는 것을 엿보기만 한다)
        n.create_subscription(JointJog, '/arm/teleop_jog', self._on_jog, 10)
        n.create_subscription(String, '/arm/teleop_cmd', self._on_cmd, 10)
        n.create_subscription(String, '/arm/teleop_poses', self._on_poses, LATCHED)
        n.create_subscription(String, '/arm/calib_status', self._on_calib, LATCHED)
        n.create_subscription(Joy, '/joy', self._on_joy, 10)

    # ------------------------------------------------------------ 콜백
    def _on_dxl_state(self, msg):
        """`[id, position, velocity, current, temperature]` × 모터 수."""
        data = list(msg.data)
        samples = [tuple(data[i:i + 5]) for i in range(0, len(data) - 4, 5)]
        if samples:
            self.store.update_motors(samples, time.monotonic())

    def _on_hw_error(self, msg):
        self.store.set_hardware_error(parse_hardware_error(msg.data), time.monotonic())

    def _on_goal(self, msg):
        if len(msg.data) >= 2:
            self.store.set_goal(msg.data[0], msg.data[1], time.monotonic())

    def _on_tick_limits(self, msg):
        self.store.set_tick_limits(list(msg.data), time.monotonic())

    def _on_threshold(self, kind, msg):
        data = list(msg.data)
        enabled = bool(data[0]) if data else True
        value = data[1] if len(data) > 1 else None
        self.store.set_threshold(kind, enabled, value, time.monotonic())

    def _on_controller_fault(self, msg):
        self.store.set_controller_fault(msg.data, time.monotonic())

    def _on_joint_states(self, msg):
        self.store.set_joint_states(list(msg.name), list(msg.position),
                                    list(msg.velocity), list(msg.effort),
                                    time.monotonic())

    def _on_arm_status(self, msg):
        # 계약: stamp 는 단조 증가해야 하고 0.5초 이상 낡으면 파워트레인이 차를 세운다.
        stamp_age = None
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds > 0:
            stamp_age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        self.store.set_arm_status(msg.status, int(msg.mission_id), time.monotonic(),
                                  stamp_age=stamp_age)

    def _on_chassis(self, msg):
        self.store.set_chassis_mode(msg.mode, time.monotonic())

    def _on_arrival(self, msg):
        self.store.set_arrival(msg.status, int(msg.mission_id), time.monotonic())

    def _on_detections(self, msg):
        self.store.set_detections([_obj_to_dict(o) for o in msg.objects], time.monotonic())

    def _on_pick_target(self, msg):
        self.store.set_pick_target(_obj_to_dict(msg), time.monotonic())

    def _on_jog(self, msg):
        self.store.set_teleop_jog(list(msg.joint_names), list(msg.velocities),
                                  time.monotonic())

    def _on_cmd(self, msg):
        self.store.set_teleop_cmd(msg.data, time.monotonic())

    def _on_poses(self, msg):
        names = [s for s in msg.data.split(',') if s]
        self.store.set_poses(names, time.monotonic())

    def _on_calib(self, msg):
        self.store.set_calib_status(msg.data, time.monotonic())

    def _on_joy(self, msg):
        self.store.set_joy(list(msg.buttons), list(msg.axes), time.monotonic())

    def _on_image(self, source, msg):
        if self._bridge is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                              # noqa: BLE001
            self.get_logger().warn(f'{source} 이미지 변환 실패: {exc}',
                                   throttle_duration_sec=5.0)
            return
        self.video.offer(source, frame, time.monotonic())

    # ------------------------------------------------------------ 2Hz 조정
    def _reconcile(self):
        self._reconcile_video()
        self._reconcile_publishers()
        self._reconcile_params()
        self.store.set_system(system_stats.snapshot())
        stats = self.video.stats()
        active = next((s for s in SOURCES if s in self._video_subs), 'off')
        self.store.set_video(active, sum(v['clients'] for v in stats.values()),
                             self.video.fps, time.monotonic())

    def _reconcile_video(self):
        """구독자가 0이면 **구독 자체를 파괴**한다.

        `perception_node` 는 `/perception/debug_image` 를 구독자가 있을 때만
        만든다. 구독을 남겨두면 브라우저에서 영상을 꺼도 Jetson 이 계속 오버레이를
        그린다 — 그래서 refcount 가 0이 되는 순간 끊는다.
        """
        desired = self.video.desired()
        for source in list(self._video_subs):
            if source not in desired:
                self.destroy_subscription(self._video_subs.pop(source))
                self.get_logger().info(f'영상 구독 해제: {SOURCES[source]} (구독자 0)')
        for source in desired:
            if source in self._video_subs:
                continue
            topic = SOURCES[source]
            self._video_subs[source] = self.create_subscription(
                Image, topic, lambda m, s=source: self._on_image(s, m), 1)
            self.get_logger().info(f'영상 구독 생성: {topic}')

    def _reconcile_publishers(self):
        try:
            js = [i.node_name for i in self.get_publishers_info_by_topic('/joint_states')]
            jog = [i.node_name for i in self.get_publishers_info_by_topic('/arm/teleop_jog')]
        except Exception:                                     # noqa: BLE001
            return
        self.store.set_joint_publishers(js)
        self.store.set_teleop_publishers(jog)

    def _reconcile_params(self):
        """다른 노드의 파라미터를 **읽기만** 한다 (set 은 절대 안 한다).

        `call_async` + done 콜백으로만 처리한다 — 타이머 콜백 안에서 결과를
        기다리며 spin 하면 재진입으로 데드락이 난다(`arm_fsm` 의 FK 클라이언트가
        같은 이유로 별도 노드를 쓴다).
        """
        if not self._motor_names_resolved:
            self._fetch_params(self._driver_node, ['motor_ids', 'joint_names'],
                               self._apply_motor_names)
        if not self._joy_params_resolved:
            self._fetch_params(self._joystick_node,
                               ['deadman_button', 'turbo_button', 'estop_button'],
                               self._apply_joy_params)

    def _fetch_params(self, node_name, names, done):
        client = self._param_clients.get(node_name)
        if client is None:
            client = self.create_client(GetParameters, f'/{node_name}/get_parameters')
            self._param_clients[node_name] = client
        if not client.service_is_ready():
            return
        future = client.call_async(GetParameters.Request(names=names))
        future.add_done_callback(lambda f: self._on_params(f, done))

    def _on_params(self, future, done):
        try:
            result = future.result()
        except Exception:                                     # noqa: BLE001
            return
        if result is not None:
            done(result.values)

    def _apply_motor_names(self, values):
        if len(values) < 2:
            return
        ids = list(values[0].integer_array_value)
        names = list(values[1].string_array_value)
        if not ids or not names:
            return
        self.store.set_motor_names(dict(zip(ids, names)))
        self._motor_names_resolved = True
        self.get_logger().info(f'모터 이름 매핑 확인: {dict(zip(ids, names))}')

    def _apply_joy_params(self, values):
        if len(values) < 3:
            return
        buttons = [int(v.integer_value) for v in values[:3]]
        self.store.set_joy_params(deadman=buttons[0], turbo=buttons[1],
                                  estop=buttons[2], resolved=True)
        self._joy_params_resolved = True
        self.get_logger().info(
            f'조이스틱 버튼 인덱스 확인: deadman={buttons[0]} '
            f'turbo={buttons[1]} estop={buttons[2]}')

    # ------------------------------------------------------------ HTTP 기동
    def _web_root(self):
        configured = self.get_parameter('web_root').value
        if configured:
            return configured
        from ament_index_python.packages import get_package_share_directory
        import os
        return os.path.join(get_package_share_directory('robot_arm_gui'), 'web')

    def _start_http(self):
        bind = self.get_parameter('bind_address').value
        port = int(self.get_parameter('port').value)
        root = self._web_root()
        self._server, _ = serve_forever_in_thread(
            store=self.store, video_hub=self.video, web_root=root,
            bind=bind, port=port, logger=self.get_logger())

        # 화면이 계약 어휘를 파생할 수 있도록 상수를 그대로 실어 보낸다.
        # (2Hz 로 갱신되는 system 과 분리해 둔다 — 덮이면 안 된다.)
        self.store.set_contract(self.contract_constants())
        self.store.add_event('boot', f'모니터 시작 — http://{bind}:{port} (읽기 전용)',
                             'info', time.monotonic())
        self.get_logger().info(
            f'robot_arm_monitor 시작 — http://{bind}:{port} '
            f'(읽기 전용, web_root={root})')
        if bind not in ('127.0.0.1', 'localhost'):
            self.get_logger().warn(
                f'bind_address={bind} — network_mode: host 라 현장 네트워크에 '
                '그대로 노출된다. 원격은 ssh -L 8088:localhost:8088 을 권장한다.')

    def contract_constants(self):
        """화면이 파생 판정(작업 허가·주행 가능·신선도)에 쓰는 상수 묶음.

        어휘를 프론트엔드에 복사해 두면 언젠가 실제 게이트와 어긋나므로,
        `contract.py` 값을 그대로 실어 보내고 화면은 그걸 읽기만 한다.
        """
        stale = dict(STALE_AFTER)
        stale['_default'] = DEFAULT_STALE_AFTER
        return {
            'lock_modes': sorted(LOCK_MODES),
            'drive_ready': sorted(DRIVE_READY_STATUSES),
            'mission_stop': MODE_MISSION_STOP,
            'heartbeat_timeout_s': HEARTBEAT_TIMEOUT_S,
            'warn_temp_c': self.warn_temp_c,
            'warn_current_ratio': self.warn_current_ratio,
            'video_default_source': self.get_parameter('video_default_source').value,
            'stale_after': stale,
        }

    def destroy_node(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
