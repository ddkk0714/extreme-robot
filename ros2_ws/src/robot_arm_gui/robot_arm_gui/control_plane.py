#!/usr/bin/env python3
"""제어 평면 — GUI 가 실제로 ROS 에 쓰는 유일한 곳.

## 왜 별도 객체인가 (런타임 분기가 아니라 구조로 막는다)

**퍼블리셔가 이 클래스의 생성자에서만 만들어진다.** `control:=false` 면 이 객체를
아예 안 만들고, 그러면 `/arm/teleop_jog`·`/arm/teleop_cmd` 퍼블리셔가 **존재 자체를
하지 않는다.** 노드 코드 안에 "제어 모드면 건너뛰기" 같은 분기를 두지 않는 이유는
이 저장소가 이미 정한 원칙이다 — 안전 게이트에 스킵 분기가 있으면 실기에서 켜진 채
도는 사고가 난다(벤치 텔레옵 격리를 launch 파일로만 하는 것과 같은 사상).

## 발행하는 것과 발행하지 않는 것

발행: `/arm/teleop_jog`(JointJog) · `/arm/teleop_cmd`(String) — 계약상 owner 가 없다.
발행 안 함: `/arm_status`·`/chassis_mode`·`/arrival_status`(계약 owner 존재) 와
`/dynamixel/goal_position`(계약이 금지하는 direct goal publisher). GUI 는
`teleop_core` 를 거쳐서만 서보에 닿는다 — 기존 벤치 텔레옵과 정확히 같은 위치다.

## 스레드

HTTP 스레드는 `CommandBus` 에 의도만 적는다. 여기 `tick()` 은 **실행기 스레드**의
타이머에서만 불리며, rclpy 엔티티(퍼블리셔·서비스 클라이언트)는 전부 그 안에서만
만진다.
"""

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from control_msgs.msg import JointJog
from std_msgs.msg import String

from .command_bus import CommandBus

#: teleop_core 기본값과 같은 안전 상한. 파라미터 조회에 성공하면 그 값으로 대체된다.
DEFAULT_MAX_VEL_RAD_S = 1.0


class ControlPlane:
    """노드가 제어 모드일 때만 생성하는 쓰기 경로."""

    def __init__(self, node, *, joint_names, publish_hz=20.0,
                 token_ttl_s=5.0, intent_timeout_s=0.3):
        self.node = node
        self.bus = CommandBus(token_ttl_s=token_ttl_s, intent_timeout_s=intent_timeout_s)
        self.joint_names = list(joint_names)
        self.max_vel_rad_s = DEFAULT_MAX_VEL_RAD_S
        self.jog_step_rad = 0.05

        # ⚠️ 이 두 줄이 읽기 전용/제어 모드를 가르는 실제 경계다.
        self.pub_jog = node.create_publisher(JointJog, '/arm/teleop_jog', 10)
        self.pub_cmd = node.create_publisher(String, '/arm/teleop_cmd', 10)

        self._set_param_clients = {}
        self._task_handlers = {}
        self._last_publish_state = 'idle'

        period = 1.0 / max(float(publish_hz), 1.0)
        self._timer = node.create_timer(period, self.tick)

    # ------------------------------------------------------------ 타이머
    def tick(self):
        """실행기 스레드 — 버스에 쌓인 의도를 실제 ROS 동작으로 옮긴다."""
        now = time.monotonic()
        state, velocities = self.bus.take_jog(now)
        if state == 'active':
            self._publish_jog(velocities)
        elif state == 'stop':
            # 워치독: 정확히 한 번 0 을 쏘고 멈춘다. 계속 쏘면 teleop_core 의
            # deadman(0.5초 무입력 → 적분 정지)이 영원히 발동하지 못한다.
            self._publish_jog({})
            self.node.get_logger().warn(
                '텔레옵 워치독: 조그 의도가 끊겨 전 관절 0 을 발행했다')
            self.node.store.add_event('teleop', '워치독 정지 — 조그 의도 끊김',
                                      'warning', now)
        self._last_publish_state = state

        for cmd in self.bus.drain_cmds():
            self.pub_cmd.publish(String(data=cmd))
            self.node.get_logger().info(f'teleop_cmd 발행: {cmd}')
            self.node.store.add_event('teleop', f'명령 발행: {cmd}', 'info', now)

        for task in self.bus.drain_tasks():
            self._run_task(task, now)

        # 화면의 '조종 중/관전 중'과 워치독 잔여 시간은 1Hz 로는 이미 늦다 —
        # 이 타이머 주기(기본 20Hz)로 store 에 밀어 넣어 hot 스냅샷에 실리게 한다.
        self.node.store.set_control(self.bus.snapshot(time.monotonic()))

    def _publish_jog(self, velocities):
        """⚠️ **전 관절을 매번 실어야 한다.**

        `teleop_core.on_jog` 는 메시지에 없는 관절의 속도를 0 으로 만들지 않는다.
        움직이는 관절만 골라 보내면 **놓은 관절이 마지막 속도로 계속 돈다** —
        실제로 있었던 버그라 그쪽 모듈 docstring 에도 못박혀 있다.
        """
        limit = self.max_vel_rad_s
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)
        msg.velocities = [
            max(-limit, min(limit, float(velocities.get(name, 0.0))))
            for name in self.joint_names
        ]
        self.pub_jog.publish(msg)

    # ------------------------------------------------------------ 세션 훅
    def on_claim(self, label, force):
        note = f'조종권 획득: {label}' + (' (강제)' if force else '')
        self.node.get_logger().info(note)
        self.node.store.add_event('control', note,
                                  'warning' if force else 'info', time.monotonic())

    def on_release(self):
        self.node.get_logger().info('조종권 반납')
        self.node.store.add_event('control', '조종권 반납', 'info', time.monotonic())

    # ------------------------------------------------------------ 작업
    def register_task(self, kind, validator, runner):
        """단계별 기능이 자기 작업 종류를 여기에 등록한다.

        `validator(payload) -> (정규화된 payload, None)` 또는 `(None, 사유)`
        `runner(payload) -> (state, detail)`  # state: 'done' | 'error'
        """
        self._task_handlers[kind] = (validator, runner)

    def validate_task(self, kind, payload):
        handler = self._task_handlers.get(kind)
        if handler is None:
            known = sorted(self._task_handlers) or ['(없음)']
            return None, f'알 수 없는 작업: {kind!r} (가능: {", ".join(known)})'
        return handler[0](payload)

    def _run_task(self, task, now):
        handler = self._task_handlers.get(task['kind'])
        if handler is None:
            self.bus.finish_task(task['id'], 'error', '핸들러가 사라졌습니다', now)
            return
        self.bus.finish_task(task['id'], 'running', '', now)
        try:
            state, detail = handler[1](task['payload'])
        except Exception as exc:                              # noqa: BLE001
            state, detail = 'error', f'{type(exc).__name__}: {exc}'
            self.node.get_logger().error(f'작업 실패 ({task["kind"]}): {detail}')
        self.bus.finish_task(task['id'], state, detail, time.monotonic())
        self.node.store.add_event(
            'task', f'{task["kind"]}: {detail or state}',
            'critical' if state == 'error' else 'info', time.monotonic())

    # ------------------------------------------------------------ 파라미터 쓰기
    def set_remote_params(self, node_name, values, done=None):
        """다른 노드의 파라미터를 설정한다 (읽기 전용 모드에는 이 경로가 없다).

        ⚠️ `spin_until_future_complete` 를 쓰지 않는다 — 타이머 콜백 안에서 결과를
        기다리며 spin 하면 재진입으로 데드락이 난다(`arm_fsm` 의 FK 클라이언트가
        같은 이유로 별도 노드를 쓴다). 결과는 done 콜백으로 받는다.

        `values`: `{이름: 파이썬 값}`. bool/int/float/str/list 를 지원한다.
        """
        client = self._set_param_clients.get(node_name)
        if client is None:
            client = self.node.create_client(
                SetParameters, f'/{node_name}/set_parameters')
            self._set_param_clients[node_name] = client
        if not client.service_is_ready():
            return False, f'{node_name} 의 파라미터 서비스가 준비되지 않았습니다'

        params = []
        for name, value in values.items():
            converted = _to_parameter_value(value)
            if converted is None:
                return False, f'{name}: 지원하지 않는 파라미터 타입 {type(value).__name__}'
            params.append(Parameter(name=name, value=converted))

        future = client.call_async(SetParameters.Request(parameters=params))
        if done is not None:
            future.add_done_callback(done)
        return True, None

    # ------------------------------------------------------------ HTTP 쪽 인터페이스
    def describe(self):
        """화면이 어떤 제어가 가능한지 판단할 근거."""
        return {
            'joint_names': list(self.joint_names),
            'max_vel_rad_s': self.max_vel_rad_s,
            'jog_step_rad': self.jog_step_rad,
            'task_kinds': sorted(self._task_handlers),
        }

    def list_models(self):
        return []

    def cloud_frame(self, since_seq):
        return None


def _to_parameter_value(value):
    """파이썬 값 → `rcl_interfaces/ParameterValue` (지원 안 하면 None)."""
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                              integer_value=int(value))
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                              double_value=float(value))
    if isinstance(value, str):
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    if isinstance(value, (list, tuple)):
        items = list(value)
        if all(isinstance(v, str) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                                  string_array_value=items)
        if all(isinstance(v, bool) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY,
                                  bool_array_value=items)
        if all(isinstance(v, int) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                  integer_array_value=items)
        if all(isinstance(v, (int, float)) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                  double_array_value=[float(v) for v in items])
    return None
