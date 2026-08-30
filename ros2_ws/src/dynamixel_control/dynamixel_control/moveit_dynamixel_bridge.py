#!/usr/bin/env python3

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead
from ament_index_python.packages import get_package_share_directory

from dynamixel_control.tool_manager import (
    ParameterToolIdentityProvider, ToolManager)
from dynamixel_control.tool_profiles import (
    load_profiles, ToolProfileError, validate_control_scope)

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_GOAL_VELOCITY = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_LOAD = 2
LEN_PRESENT_POSITION = 4

# HARDWARE_ERROR_STATUS(70,1) ~ PRESENT_POSITION(132,4) 은 X-시리즈 컨트롤 테이블에서
# 연속 주소 범위라, 70부터 66바이트를 한 번의 SyncRead 로 받아 fault/load/position 을
# 함께 추출(버스 트랜잭션 1회). 중간의 다른 필드(Profile Accel/Velocity 등)도 같이
# 읽히지만 안 쓰고 버림 — 주소가 연속이기만 하면 여분을 읽는 건 무해함.
# (XL430/XC430/XM 계열 공통. 다른 모델이면 주소 재확인 필요 — CLAUDE.md §8 모터모델 미확정.)
ADDR_SYNC_READ_START = ADDR_HARDWARE_ERROR_STATUS
LEN_SYNC_READ = (ADDR_PRESENT_POSITION + LEN_PRESENT_POSITION) - ADDR_HARDWARE_ERROR_STATUS  # = 66

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_CENTER_POSITION = 2048

TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)


# 팔 관절 ↔ 다이나믹셀 ID 매핑. (현재 3개만 — 팔 DOF 확정 시 arm_joint_4~ 추가, CLAUDE.md §8)
# 조인트 이름은 2026-07-15 Isaac Sim 기반 재export(robotarm_urdf_20260711.urdf) 기준
# (arm_joint_1~5) — 기존 fusion2urdf 계열(joint_1~5)에서 갈아탐.
JOINT_CONFIG = {
    "arm_joint_1": {"id": 0, "center": 2048, "direction": 1},
    "arm_joint_2": {"id": 1, "center": 2048, "direction": 1},
    "arm_joint_3": {"id": 2, "center": 2048, "direction": 1},
}


def to_signed(value, byte_len):
    """무부호 정수를 byte_len 바이트 2의 보수 부호 정수로 변환."""
    bits = byte_len * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


class MoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__("moveit_dynamixel_bridge")

        self.declare_parameter("read_only", False)
        self.declare_parameter("mock_mode", False)
        self.declare_parameter("tool_type", "spur_1motor_gripper")
        self.declare_parameter("control_scope", "FULL_ROBOT")
        self.declare_parameter("gripper_target_tolerance_ticks", 20)
        default_profiles = str(Path(get_package_share_directory(
            'dynamixel_control')) / 'config' / 'tool_profiles.yaml')
        self.declare_parameter("tool_profile_file", default_profiles)
        # 털털이 ZIP에 모터 ID/방향/속도가 없어 모두 fail-closed 기본값이다.
        self.declare_parameter("cleaning_actuator_joint", "")
        self.declare_parameter("cleaning_actuator_id", -1)
        self.declare_parameter("cleaning_direction", 0)
        self.declare_parameter("cleaning_velocity_raw", 0)

        self.read_only = bool(self.get_parameter("read_only").value)
        self.mock_mode = bool(self.get_parameter("mock_mode").value)
        self.tool_type = str(self.get_parameter("tool_type").value)
        self.control_scope = validate_control_scope(
            self.get_parameter("control_scope").value)
        self.gripper_target_tolerance = int(
            self.get_parameter("gripper_target_tolerance_ticks").value)
        if self.gripper_target_tolerance < 0:
            raise ValueError('gripper_target_tolerance_ticks must be non-negative')
        self.control_mode = 'FSM'
        self.emergency_stop_active = False
        self.tool_detached = False
        self._tool_samples = {}
        self._gripper_goal_active = False
        self._gripper_goal_lock = threading.Lock()
        self._bus_lock = threading.RLock()
        self.cleaning_actuator_joint = self.get_parameter("cleaning_actuator_joint").value
        self.cleaning_actuator_id = int(self.get_parameter("cleaning_actuator_id").value)
        self.cleaning_direction = int(self.get_parameter("cleaning_direction").value)
        self.cleaning_velocity_raw = int(self.get_parameter("cleaning_velocity_raw").value)
        self.cleaning_configured = (
            bool(self.cleaning_actuator_joint) and self.cleaning_actuator_id >= 0
            and self.cleaning_direction in (-1, 1) and self.cleaning_velocity_raw > 0
        )
        try:
            profiles = load_profiles(self.get_parameter('tool_profile_file').value)
            if self.tool_type == 'cleaner' and self.cleaning_configured:
                profiles['cleaner'].update({
                    'calibrated': True,
                    'actuator_ids': [self.cleaning_actuator_id],
                    'joint_names': [self.cleaning_actuator_joint],
                    'direction': self.cleaning_direction,
                    'profile_velocity': self.cleaning_velocity_raw,
                    'profile_acceleration': 1,
                })
            self.tool_manager = ToolManager(
                profiles, ParameterToolIdentityProvider(self.tool_type),
                mock_mode=self.mock_mode)
            self.tool_selection = self.tool_manager.refresh('IDLE')
        except (ToolProfileError, KeyError) as exc:
            self.get_logger().error(f'tool profile rejected: {exc}')
            self.tool_selection = None
        self.tool_motion_allowed = bool(
            self.tool_selection and self.tool_selection.valid
            and not self.read_only and not self.mock_mode)
        self.tool_profile = (
            self.tool_selection.profile if self.tool_selection else {})

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.port_connected = self.mock_mode
        if not self.mock_mode:
            try:
                self.port_connected = bool(self.port_handler.openPort())
                if self.port_connected:
                    self.port_connected = bool(
                        self.port_handler.setBaudRate(BAUDRATE))
            except Exception as exc:
                self.port_connected = False
                self.get_logger().error(f'Cannot open {DEVICENAME}: {exc}')
            if not self.port_connected and not self.read_only:
                raise RuntimeError(f"Failed to open/configure port: {DEVICENAME}")

        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        # hardware error+address 126 feedback+position 블록을 한 번에 읽는 SyncRead
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_SYNC_READ_START,
            LEN_SYNC_READ,
        )

        # 토크 ON에 성공해 SyncRead 에 실제로 등록된 ID만 추적 — 이후 매 tick 이 ID들의
        # 응답 유무/Hardware Error Status 로 controller fault 를 판정한다(등록 안 된 ID는
        # 애초에 버스에 없거나 비활성화된 것으로 간주해 fault 판정에서 제외).
        self.active_ids = set()

        if not self.read_only and not self.mock_mode:
            if self.control_scope == 'FULL_ROBOT':
                # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
                for joint_name, config in JOINT_CONFIG.items():
                    if self._enable_torque(config["id"], joint_name):
                        self.group_sync_read.addParam(config["id"])
                        self.active_ids.add(config["id"])
            if self.cleaning_configured:
                self._configure_cleaning_actuator()

        self.tool_ids = list(self.tool_profile.get('actuator_ids', []))
        self.tool_discovered = self.mock_mode
        if not self.mock_mode and self.port_connected:
            self.tool_discovered = self._discover_tool_ids()
            if self.read_only:
                for dxl_id in self.tool_ids:
                    self.group_sync_read.addParam(dxl_id)
                    self.active_ids.add(dxl_id)
            elif self.tool_motion_allowed and self.tool_discovered:
                self._configure_tool_actuators()
            elif self.tool_ids and not self.tool_discovered:
                self.tool_motion_allowed = False

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )
        self.create_subscription(Bool, "/cleaning/enable", self._on_cleaning_enable, 10)
        self.create_subscription(Bool, "/tool/emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(Bool, "/tool/detached", self._on_tool_detached, 10)
        self.create_subscription(
            String, "/control/mode_status", self._on_control_mode, 10)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.gripper_cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # 계약 §5.1 "locked heartbeat는 ... controller fault 0 ... 을 실제 확인한다" 대응.
        # arm_fsm 이 CARRYING_LOCKED/STOWED_LOCKED 발행 전 게이트로 구독(내부용 — 파워트레인
        # 쪽 DDS 경계를 넘지 않음, robot_arm_msgs 계약과 무관).
        self.fault_pub = self.create_publisher(
            Bool,
            "/dynamixel/controller_fault",
            10,
        )
        self.tool_type_pub = self.create_publisher(String, '/tool/type', 10)
        self.tool_status_pub = self.create_publisher(String, '/tool/status', 10)

        self.feedback_timer = self.create_timer(0.05, self.publish_joint_states)
        self.tool_status_timer = self.create_timer(0.5, self.publish_tool_status)

        self.get_logger().info(
            f"MoveIt Dynamixel bridge started (arm={list(JOINT_CONFIG)}, "
            f"cleaning_actuator={self.cleaning_actuator_joint or 'UNCONFIGURED'}, "
            f"tool_type={self.tool_type}, tool_ready={self.tool_motion_allowed}, "
            f"control_scope={self.control_scope}, "
            f"read_only={self.read_only}, mock_mode={self.mock_mode})"
        )

    # ------------------------------------------------------------------ helpers
    def _enable_torque(self, dxl_id, label):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != 0 or error != 0:
            self.get_logger().warn(
                f"Torque enable failed: {label}, id={dxl_id}, result={result}, error={error}"
            )
            return False
        else:
            self.get_logger().info(f"Torque enabled: {label} -> id {dxl_id}")
            return True

    def _discover_tool_ids(self):
        """Ping every configured actuator; any missing ID closes the backend."""
        if not self.tool_ids:
            return False
        missing = []
        for dxl_id in self.tool_ids:
            _model, result, error = self.packet_handler.ping(
                self.port_handler, dxl_id)
            if result != 0 or error != 0:
                missing.append(dxl_id)
        if missing:
            self.get_logger().error(f'tool actuator IDs not discovered: {missing}')
            return False
        return True

    def _configure_tool_actuators(self):
        """Apply profile motion limits only after strict validation and discovery."""
        if self.tool_profile.get('backend') == 'cleaner':
            return
        modes = self.tool_profile.get('required_operating_modes', {})
        for dxl_id in self.tool_ids:
            mode = modes.get(dxl_id, modes.get(str(dxl_id), 3))
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE, int(mode))
            if result != 0 or error != 0:
                self.tool_motion_allowed = False
                self.get_logger().error(f'operating mode setup failed: id={dxl_id}')
                return
            for address, value in (
                    (ADDR_PROFILE_ACCELERATION,
                     self.tool_profile['profile_acceleration']),
                    (ADDR_PROFILE_VELOCITY,
                     self.tool_profile['profile_velocity'])):
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, address, int(value))
            goal_pwm = int(self.tool_profile.get('goal_pwm', 0))
            if goal_pwm > 0:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_PWM, goal_pwm)
            if self._enable_torque(dxl_id, f'{self.tool_type} tool'):
                self.group_sync_read.addParam(dxl_id)
                self.active_ids.add(dxl_id)
            else:
                self.tool_motion_allowed = False

    def _stop_tool(self, reason):
        """Best-effort stop for emergency, detach, cancellation, and shutdown."""
        self.tool_motion_allowed = False
        if self.mock_mode or self.read_only:
            return
        with self._bus_lock:
            for dxl_id in self.tool_ids:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.get_logger().warn(f'tool actuator stopped: {reason}')

    def _on_emergency_stop(self, msg):
        if msg.data:
            self.emergency_stop_active = True
            self._stop_tool('emergency stop')

    def _on_tool_detached(self, msg):
        if msg.data:
            self.tool_detached = True
            self._stop_tool('tool detach')

    def _on_control_mode(self, msg):
        mode = msg.data.strip().upper()
        if mode not in ('MANUAL', 'FSM'):
            self.get_logger().warn(f'unknown control mode ignored: {msg.data!r}')
            return
        self.control_mode = mode

    def publish_tool_status(self):
        self.tool_type_pub.publish(String(data=self.tool_type))
        reason = ''
        if not self.tool_selection:
            reason = 'profile load failed'
        elif not self.tool_selection.valid:
            reason = self.tool_selection.reason
        elif not self.tool_discovered and not self.mock_mode:
            reason = 'actuator not discovered'
        elif self.read_only:
            reason = 'read-only diagnostic mode'
        status = {
            'control_scope': self.control_scope,
            'tool_type': self.tool_type,
            'backend': self.tool_profile.get('backend', 'invalid'),
            'profile_valid': bool(self.tool_selection and self.tool_selection.valid),
            'calibrated': bool(self.tool_profile.get('calibrated')),
            'actuators_discovered': self.tool_discovered,
            'motion_allowed': self._tool_backend_ready(),
            'read_only': self.read_only, 'mock_mode': self.mock_mode,
            'bridge_connected': True,
            'u2d2_connected': self.port_connected,
            'control_mode': self.control_mode,
            'emergency_stop': self.emergency_stop_active,
            'tool_detached': self.tool_detached,
            'actuators': [self._tool_samples.get(dxl_id, {
                'id': dxl_id, 'joint': '', 'position': None,
                'effort': 0.0 if self.mock_mode else None,
                'online': self.mock_mode}) for dxl_id in self.tool_ids],
            'reason': reason,
        }
        self.tool_status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _tool_actuators_online(self):
        """Return true only when every profile-selected actuator is online."""
        if not self.tool_ids:
            return False
        return all(
            (self._tool_samples.get(dxl_id) or {}).get('id') == dxl_id
            and bool((self._tool_samples.get(dxl_id) or {}).get('online'))
            for dxl_id in self.tool_ids)

    def _tool_backend_ready(self):
        if self.mock_mode:
            return True
        return bool(
            self.tool_selection and self.tool_selection.valid
            and self.tool_profile.get('calibrated')
            and self.tool_discovered and self._tool_actuators_online()
            and self.tool_motion_allowed and not self.read_only
            and not self.emergency_stop_active and not self.tool_detached)

    def _configure_cleaning_actuator(self):
        """Dynamixel Protocol 2.0 velocity mode(Operating Mode=1)로 설정한다."""
        dxl_id = self.cleaning_actuator_id
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, 1)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"Cleaning actuator velocity-mode setup failed: id={dxl_id}")
            self.cleaning_configured = False
            return
        if self._enable_torque(dxl_id, self.cleaning_actuator_joint):
            self.group_sync_read.addParam(dxl_id)
            self.active_ids.add(dxl_id)
        else:
            self.cleaning_configured = False

    def _on_cleaning_enable(self, msg):
        if (self.read_only or self.mock_mode or not self.cleaning_configured
                or self.tool_type != 'cleaner'
                or self.control_mode != 'MANUAL'
                or not self._tool_backend_ready()):
            if msg.data:
                self.get_logger().error(
                    "Cleaning command rejected: actuator ID/direction/velocity not configured")
            return
        velocity = self.cleaning_direction * self.cleaning_velocity_raw if msg.data else 0
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.cleaning_actuator_id, ADDR_GOAL_VELOCITY,
            velocity & 0xffffffff)
        if result != 0 or error != 0:
            self.get_logger().error(
                f"Cleaning velocity write failed: result={result}, error={error}")

    def rad_to_tick(self, joint_name, rad):
        config = JOINT_CONFIG[joint_name]
        tick = config["center"] + config["direction"] * rad * TICKS_PER_RAD
        tick = int(round(tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        config = JOINT_CONFIG[joint_name]
        return (tick - config["center"]) / (config["direction"] * TICKS_PER_RAD)

    def int_to_little_endian_4bytes(self, value):
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def _tool_position_tick(self, dxl_id, raw_tick):
        endpoints = self.tool_profile.get('motor_endpoints') or {}
        endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id), {}))
        if any(value is not None and value < 0 for value in endpoint.values()):
            return to_signed(raw_tick, LEN_PRESENT_POSITION)
        return int(raw_tick)

    def goal_callback(self, goal_request):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.get_logger().warn(
                'Arm trajectory rejected: control_scope=END_EFFECTOR_ONLY')
            return GoalResponse.REJECT
        if (self.read_only or not self.tool_selection
                or not self.tool_selection.valid
                or (not self.mock_mode and not self.tool_motion_allowed)):
            self.get_logger().warn(
                "Arm trajectory rejected: read-only or tool interlock not ready")
            return GoalResponse.REJECT
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        self._stop_tool('action cancelled')
        return CancelResponse.ACCEPT

    def gripper_goal_callback(self, goal_request):
        if self.tool_profile.get('backend') != 'gripper':
            self.get_logger().error('gripper goal rejected: selected tool is not a gripper')
            return GoalResponse.REJECT
        if (not self.mock_mode and
                (self.control_mode != 'MANUAL' or not self._tool_backend_ready())):
            self.get_logger().error(
                'gripper goal rejected: MANUAL ownership or tool backend '
                'interlock not ready')
            return GoalResponse.REJECT
        with self._gripper_goal_lock:
            if self._gripper_goal_active:
                self.get_logger().warn(
                    'gripper goal rejected: another gripper goal is active')
                return GoalResponse.REJECT
            self._gripper_goal_active = True
        self.get_logger().info('gripper goal accepted')
        return GoalResponse.ACCEPT

    def gripper_cancel_callback(self, _goal_handle):
        self.get_logger().info('gripper cancel requested')
        return CancelResponse.ACCEPT

    def execute_gripper(self, goal_handle):
        try:
            return self._execute_gripper(goal_handle)
        finally:
            with self._gripper_goal_lock:
                self._gripper_goal_active = False

    def _execute_gripper(self, goal_handle):
        """Map one logical gripper joint to one or two calibrated actuators."""
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        if not trajectory.points or not trajectory.points[-1].positions:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'empty gripper trajectory'
            goal_handle.abort()
            return result
        if self.mock_mode:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'mock gripper action'
            goal_handle.succeed()
            return result
        position = float(trajectory.points[-1].positions[0])
        open_pos = float(self.tool_profile.get('open_position', 1.0))
        close_pos = float(self.tool_profile.get('close_position', 0.0))
        denominator = open_pos - close_pos
        if denominator == 0.0:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'invalid logical gripper endpoints'
            goal_handle.abort()
            return result
        ratio = max(0.0, min(1.0, (position - close_pos) / denominator))
        endpoints = self.tool_profile.get('motor_endpoints') or {
            self.tool_ids[0]: {
                'open': self.tool_profile['open_tick'],
                'close': self.tool_profile['close_tick']}}
        low = int(self.tool_profile['safe_min_tick'])
        high = int(self.tool_profile['safe_max_tick'])
        targets = {}
        try:
            with self._bus_lock:
                for dxl_id in self.tool_ids:
                    ep = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
                    tick = int(round(ep['close'] + ratio *
                                     (ep['open'] - ep['close'])))
                    if not low <= tick <= high:
                        raise RuntimeError(
                            f'id {dxl_id} goal {tick} outside [{low},{high}]')
                    comm, error = self.packet_handler.write4ByteTxRx(
                        self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                        tick & 0xffffffff)
                    if comm != 0 or error != 0:
                        raise RuntimeError(f'goal write failed for id {dxl_id}')
                    targets[dxl_id] = tick
            self.get_logger().info(
                f'gripper targets dispatched: normalized={ratio:.6f}, '
                f'targets={targets}')
            deadline = time.monotonic() + float(
                self.tool_profile.get('action_time', 0.0))
            max_effort = float(self.tool_profile.get(
                'max_abs_effort', float('inf')))
            errors = {}
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    self._hold_tool_position()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'gripper goal canceled and held'
                    goal_handle.canceled()
                    self.get_logger().info(result.error_string)
                    return result
                with self._bus_lock:
                    for dxl_id in self.tool_ids:
                        position, load = self._read_tool_state(dxl_id)
                        if abs(load) > max_effort:
                            raise RuntimeError(
                                f'id {dxl_id} effort limit exceeded')
                        errors[dxl_id] = targets[dxl_id] - position
                if all(abs(error) <= self.gripper_target_tolerance
                       for error in errors.values()):
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = (
                        f'gripper targets reached: targets={targets}, '
                        f'errors={errors}')
                    goal_handle.succeed()
                    self.get_logger().info(result.error_string)
                    return result
                time.sleep(0.05)
            raise RuntimeError(
                f'gripper target tolerance not reached: targets={targets}, '
                f'errors={errors}, tolerance={self.gripper_target_tolerance}')
        except Exception as exc:
            self._stop_tool(str(exc))
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            goal_handle.abort()
            return result

    def _read_tool_state(self, dxl_id):
        hw, comm, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        load, load_comm, load_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_LOAD)
        position, pos_comm, pos_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if (comm != 0 or error != 0 or load_comm != 0 or load_error != 0
                or pos_comm != 0 or pos_error != 0 or hw != 0):
            raise RuntimeError(f'fault reading id {dxl_id}')
        return self._tool_position_tick(dxl_id, position), to_signed(load, 2)

    def _hold_tool_position(self):
        with self._bus_lock:
            positions = {
                dxl_id: self._read_tool_state(dxl_id)[0]
                for dxl_id in self.tool_ids}
            for dxl_id, position in positions.items():
                comm, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_GOAL_POSITION,
                    position & 0xffffffff)
                if comm != 0 or error != 0:
                    raise RuntimeError(f'hold write failed for id {dxl_id}')

    # ------------------------------------------------------------------ arm
    def execute_follow_joint_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        self.get_logger().info(
            f"Executing FollowJointTrajectory with {len(trajectory.points)} points"
        )

        self.trajectory_callback(trajectory)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory sent to Dynamixel motors"
        return result

    def trajectory_callback(self, msg):
        if self.control_scope == 'END_EFFECTOR_ONLY':
            self.get_logger().warn(
                'Ignoring arm trajectory: control_scope=END_EFFECTOR_ONLY')
            return
        if (self.read_only or not self.tool_selection
                or not self.tool_selection.valid
                or (not self.mock_mode and not self.tool_motion_allowed)):
            self.get_logger().warn(
                "Ignoring arm trajectory: tool interlock not ready")
            return
        if not msg.points:
            return

        point = msg.points[-1]

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn("JointTrajectory names/positions length mismatch")
            return

        self.group_sync_write.clearParam()

        for joint_name, rad in zip(msg.joint_names, point.positions):
            if joint_name not in JOINT_CONFIG:
                self.get_logger().warn(f"Unknown joint from MoveIt: {joint_name}")
                continue

            dxl_id = JOINT_CONFIG[joint_name]["id"]
            goal_tick = self.rad_to_tick(joint_name, rad)
            param_goal_position = self.int_to_little_endian_4bytes(goal_tick)

            ok = self.group_sync_write.addParam(dxl_id, param_goal_position)
            if not ok:
                self.get_logger().warn(f"Failed to add sync write param: id={dxl_id}")

            self.get_logger().info(
                f"{joint_name} -> id {dxl_id}: {rad:.3f} rad -> {goal_tick}"
            )

        result = self.group_sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(f"GroupSyncWrite failed: result={result}")

        self.group_sync_write.clearParam()

    # ------------------------------------------------------------------ feedback
    def publish_joint_states(self):
        if self.mock_mode:
            self.joint_state_pub.publish(JointState())
            self.fault_pub.publish(Bool(data=False))
            return
        if not self.port_connected:
            self.joint_state_pub.publish(JointState())
            self.fault_pub.publish(Bool(data=True))
            return
        with self._bus_lock:
            self.group_sync_read.txRxPacket()
        # 일부 ID가 버스에 없어도 응답받은 ID만 처리 (result 무시)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # controller fault 집계 — SyncRead 에 등록된(토크 ON 성공) ID 중 하나라도
        # Hardware Error Status != 0 이거나 이번 tick 응답이 없으면 fault=True.
        # 응답 없음도 fault 로 보는 이유: 활성 등록된 서보가 갑자기 무응답이면 버스/전원
        # 이상일 수 있어 "정상"으로 오인하면 안 됨(안전 측 기본값).
        fault = False

        # 팔 관절: position(rad) + address-126 feedback(raw signed).
        # 주소 126의 의미는 실제 장착 모터 control table로 확인해야 한다.
        for joint_name, config in JOINT_CONFIG.items():
            dxl_id = config["id"]
            if dxl_id not in self.active_ids:
                continue
            sample = self._read_sample(dxl_id)
            if sample is None:
                fault = True
                continue
            feedback_raw, tick, hw_error = sample
            if hw_error != 0:
                fault = True
            msg.name.append(joint_name)
            msg.position.append(self.tick_to_rad(joint_name, tick))
            msg.effort.append(float(feedback_raw))

        if (self.cleaning_configured
                and self.cleaning_actuator_id in self.active_ids):
            sample = self._read_sample(self.cleaning_actuator_id)
            if sample is None:
                fault = True
                self._tool_samples[self.cleaning_actuator_id] = {
                    'id': self.cleaning_actuator_id,
                    'joint': self.cleaning_actuator_joint, 'position': None,
                    'effort': None, 'online': False}
            else:
                load_raw, tick, hw_error = sample
                tick = self._tool_position_tick(dxl_id, tick)
                fault = fault or hw_error != 0
                self._tool_samples[self.cleaning_actuator_id] = {
                    'id': self.cleaning_actuator_id,
                    'joint': self.cleaning_actuator_joint,
                    'position': int(tick), 'effort': float(abs(load_raw)),
                    'online': hw_error == 0}
                msg.name.append(self.cleaning_actuator_joint)
                msg.position.append(float(to_signed(tick, LEN_PRESENT_POSITION)))
                msg.effort.append(float(load_raw))

        if self.tool_profile.get('backend') == 'gripper':
            joint_names = self.tool_profile.get('joint_names', [])
            loads = []
            positions = []
            for dxl_id in self.tool_ids:
                if dxl_id not in self.active_ids:
                    fault = True
                    self._tool_samples[dxl_id] = {
                        'id': dxl_id, 'joint': '', 'position': None,
                        'effort': None, 'online': False}
                    continue
                sample = self._read_sample(dxl_id)
                if sample is None:
                    fault = True
                    self._tool_samples[dxl_id] = {
                        'id': dxl_id, 'joint': '', 'position': None,
                        'effort': None, 'online': False}
                    continue
                load_raw, tick, hw_error = sample
                fault = fault or hw_error != 0
                loads.append(abs(load_raw))
                tick = self._tool_position_tick(dxl_id, tick)
                positions.append(tick)
                self._tool_samples[dxl_id] = {
                    'id': dxl_id,
                    'joint': (joint_names[0] if joint_names else ''),
                    'position': int(tick), 'effort': float(abs(load_raw)),
                    'online': hw_error == 0}
            if joint_names and loads:
                msg.name.append(joint_names[0])
                msg.position.append(float(positions[0]))
                msg.effort.append(float(max(loads)))

        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """SyncRead 블록에서 (signed address-126 feedback, position, hw error) 추출.

        미수신 시 None.
        """
        with self._bus_lock:
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS,
                    LEN_HARDWARE_ERROR_STATUS):
                return None
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
                return None
            if not self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                return None
            hw_error = self.group_sync_read.getData(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS,
                LEN_HARDWARE_ERROR_STATUS)
            feedback_raw = to_signed(
                self.group_sync_read.getData(
                    dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD),
                LEN_PRESENT_LOAD,
            )
            tick = self.group_sync_read.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return feedback_raw, tick, hw_error

    def destroy_node(self):
        if not self.read_only and not self.mock_mode:
            self._stop_tool('node shutdown')
            if self.cleaning_configured:
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, self.cleaning_actuator_id, ADDR_GOAL_VELOCITY, 0)
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.cleaning_actuator_id,
                    ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if self.control_scope == 'FULL_ROBOT':
                for config in JOINT_CONFIG.values():
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, config["id"],
                        ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if not self.mock_mode and self.port_connected:
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItDynamixelBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
