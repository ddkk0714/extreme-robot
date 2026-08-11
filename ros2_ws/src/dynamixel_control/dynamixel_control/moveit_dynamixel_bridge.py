#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from control_msgs.action import FollowJointTrajectory
from robot_arm_msgs.action import ArmRecordedPath, ArmTestMove, EndEffectorRotate
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead

from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
from dynamixel_control import joint_limits


ADDR_TORQUE_ENABLE = 64
# Goal PWM (RAM) — 서보가 낼 수 있는 토크 상한. 그리퍼 과전류 트립 방지용.
# XL430 은 전류 제어(모드 5 / Goal Current)가 없어 이게 유일한 힘 제한 수단이다.
ADDR_GOAL_PWM = 100
# EEPROM 영역 — **토크 OFF 상태에서만 써진다.**
ADDR_OPERATING_MODE = 11
MODE_POSITION = 3           # 단일회전 0~4095
MODE_EXTENDED_POSITION = 4  # 다회전, tick 이 범위를 넘고 음수도 된다
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_MOVING_STATUS = 123
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_LOAD = 2
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_POSITION = 4

# X-시리즈(XL430/XC430/XM 공통) Present Velocity 데이터시트 고정값: signed, 1 LSB = 0.229 rev/min.
# PRESENT_VELOCITY 는 이미 아래 SyncRead 범위(70~135) 안에 있어 버스 트랜잭션 추가 없이
# 파싱만 하면 된다 — 그동안 버려지던 바이트를 꺼내 쓰는 것뿐(Notion "그리퍼 tick/
# wrist_to_gripper/PRESENT_VELOCITY 실측·검증 절차" §2-3).
VELOCITY_LSB_TO_RAD_S = 0.229 * 2.0 * math.pi / 60.0

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
DXL_TICKS_PER_REV = 4096.0  # 물리 인코더 상수(=TICKS_PER_RAD*2π) — Present Velocity 환산용


# 팔 관절 ↔ 다이나믹셀 ID 매핑.
#
# 2026-08-07 실기 버스 스캔에 맞춰 갱신. 그 전까지 이 dict 는 id 0/1/2 를 가리켰는데
# **버스에 존재하지 않는 ID** 라 팔 서보 토크 인가가 전부 실패했다. 값의 출처는
# teleop_core_node.py 의 DEFAULT_MOTOR_IDS/DEFAULT_DIRECTIONS(2026-08-01/08-02 벤치
# 실측) 이며, 이 두 파일은 같은 물리 버스를 가리키므로 **한쪽을 바꾸면 다른 쪽도
# 같이 바꿔야 한다.**
#
# ⚠️ arm_joint_1(ID 11, 베이스 요축)은 **모터가 물리적으로 없다**(2026-08-07 사용자
#    확인). 스캔에도 안 잡혀서 아예 등록하지 않는다 — 등록해두면 매 tick 무응답으로
#    fault 가 서게 된다. 모터가 붙으면 여기에 한 줄 추가하면 된다.
#
# `gear_ratio` = 서보축 회전 / 관절 회전. 1.0 이면 직결.
#
# arm_joint_2/3 값은 2026-08-07 `scripts/measure_gear_ratio.py` 로 실측했다(토크를
# 끄고 관절을 손으로 돌려 서보각 변화 / 관절각 변화). **두 축의 감속비는 서로 다르다**
# (사용자 확인) — 그 전까지 teleop_core_node.py 주석이 두 축을 묶어 "약 10:1" 로
# 추정했지만 실측 결과 arm_joint_2 만 10:1 에 가깝고 arm_joint_3 은 절반 이하다.
#
# ⚠️ 실측 정밀도는 관절각을 얼마나 정확히 쟀는지에 달려 있다(90° 를 ±9° 오차로 재면
#    기어비도 약 ±10% 흔들린다). 파지 위치가 계통적으로 어긋나면 이 값부터 의심할 것.
#    재측정 없이 시험할 땐 `gear_ratios` 파라미터로 덮어쓸 수 있다.
#
# 🔒 **모를 때는 낮은 값을 쓴다.** 기어비를 실제보다 낮게 잡으면 관절이 명령보다 덜
#    움직여(언더슈트) 안전하지만, 높게 잡으면 그 배수만큼 과주행해 구조물을 때린다.
#
# `extended` = Extended Position Control Mode(다회전) 축. tick 이 0~4095 를 넘어가고
# 음수도 나오므로 부호 있는 정수로 해석해야 한다(teleop_core 의 EXTENDED_POSITION_NAMES 와 짝).
#
# `center` = 관절 0도에 해당하는 tick. 2026-08-07 `scripts/measure_zero_offset.py` 로
# URDF home 자세(전 관절 0도)에서 실측했다. 그 전까지는 전 축 2048(서보 중앙값)이라는
# **검증된 적 없는 가정**이었고, 실제로 축마다 최대 1100 tick(≈97°) 어긋나 있었다.
# 기어비와 마찬가지로 영점이 틀리면 IK 결과가 통째로 그만큼 어긋난다.
# ⚠️ 팔을 분해·재조립하거나 서보를 뿔에서 뺐다 끼우면 이 값은 무효다 — 다시 측정할 것.
#
# 🔁 **2026-08-09 재측정.** `extended` 축(arm_joint_2/3)의 다회전 카운트는 전원을 내리면
#    초기화되므로, 한 바퀴(4096) 밖의 center 는 그걸 잰 전원 세션 안에서만 유효하다.
#    실제로 arm_joint_3 의 구 center=4281 은 전원 사이클 후 관절각을 -1.58 rad 로
#    읽게 만들었다(안전범위는 0~2.034) — 그대로 구동하면 틀린 기준점 위에서 +90°
#    스윙한다. 아래 값은 그래서 다시 잰 것이다.
#      교차검증: 순수 카운트 초기화라면 새 center 는 4281-4096=185 여야 하는데 278 이
#      나왔다(관절 2.0° 차) — 즉 -87° 편차는 전부 카운트 초기화分이고 자세 재현
#      오차는 2° 수준이었다는 뜻.
#
# ⚠️ **직결(1:1) 축인 arm_joint_4/5 는 토크를 끄면 중력으로 흘러내린다** — 감속기 축
#    (9:1, 4:1)은 역구동이 안 돼 손으로 세운 자세가 유지되지만 이 둘은 손을 떼면 처진다.
#    영점 측정은 반드시 **팔을 붙잡은 상태에서** 할 것.
#
# 🔁 **2026-08-09 2차 재측정** (아래 값). 1차 측정 후 실기 구동 중 손목(arm_joint_5)
#    결합이 물리적으로 빠져 재조립했고, 서보 전원도 내려갔다 — 둘 다 영점 무효 사유다.
#    재조립 후 측정에서는 붙잡은 상태의 드리프트가 8초간 arm_joint_2/5 **정확히 0.000°**
#    로 나왔다.
#    ⚠️ 1차 측정 때 arm_joint_5 가 붙잡고 있는데도 +1.5°/s 로 미끄러졌던 것은 중력이
#       아니라 **결합이 이미 헐거웠다는 신호**였다(수리 후 그 드리프트가 완전히 사라진
#       것으로 확인). 어떤 축이 "잡아도 계속 흐르면" 측정을 계속하지 말고 결합부터
#       점검할 것 — 그대로 두면 구동 중 빠진다.
JOINT_CONFIG = {
    # 2026-08-07 실측: 9.034:1 (관절 90° 회전 기준)
    "arm_joint_2": {"id": 14, "center": 641, "direction": -1,
                    "gear_ratio": 9.034, "extended": True},
    # 2026-08-07 실측: 4.040:1 — arm_joint_2 와 다른 감속기다(오타 아님)
    "arm_joint_3": {"id": 13, "center": 207, "direction": 1,
                    "gear_ratio": 4.040, "extended": True},
    "arm_joint_4": {"id": 12, "center": 2510, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    "arm_joint_5": {"id": 16, "center": 985, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
}
ARM_IDS = {config["id"] for config in JOINT_CONFIG.values()}
ARM_ID_SEQUENCE = [config["id"] for config in JOINT_CONFIG.values()]
ARM_TEST_SEQUENCE = ((14, 5), (13, 10), (12, 10), (16, 20))
RANDOM_ARM_RANGES = {14: 20, 13: 40, 12: 40, 16: 80}
RECORDED_PATH_IDS = (14, 13, 12)
RECORDED_PATH_START_TOLERANCE = {14: 20, 13: 30, 12: 20}
RECORDED_PATH_MAX_WAYPOINT_STEP = 50

# 모터가 없어 실측할 수 없지만 **URDF 상으로는 존재하는** 관절 — 고정값으로 발행한다.
#
# arm_joint_1(베이스 요축)은 서보가 물리적으로 없다(2026-08-07). 그런데 URDF 에서는
# `link_002 → link_004` 를 잇는 관절이라, /joint_states 에 값이 없으면
# `robot_state_publisher` 가 이 관절을 못 넘어가 **TF 트리가 두 조각으로 갈린다**
# ("Tf has two or more unconnected trees") → `base_link → link_043`(tip) 변환이 아예
# 안 만들어져서 arm_fsm 의 IK/carry pose 계산이 전부 실패한다. MoveIt 도 5축 전체
# 관절값을 기대하므로 같은 이유로 필요하다.
#
# 값의 근거 (2026-08-07):
#   arm_joint_2/3/4 는 축이 서로 평행해서 이 팔은 **평면 로봇**이고, 그 평면의 방위를
#   정하는 유일한 관절이 arm_joint_1 이다(축 0 0 1, 회전중심은 base_link 원점 위
#   z=0.0465). 즉 이 값이 틀리면 팔이 향하는 방향 전체가 틀린다.
#
#   처음엔 0.0 으로 뒀는데, 그 상태의 FK 는 그리퍼를 방위각 -80.5°(거의 정오른쪽,
#   반경 12.8cm)에 놓았다 — 실기 확인 결과 팔은 **정면(+x)** 을 향하고 있어서
#   가정이 틀렸다. 반경은 그대로 두고 방위각만 0 으로 돌리는 값이 +1.405 rad 다.
#     회전 검산: (0.021, -0.126) 을 +80.5° 회전 → (0.128, 0.000)
#
# ⚠️ 이 축은 모터가 없다. **기구적으로 고정돼 있다는 전제**이며, 만약 자유회전
#    상태라면 팔의 평면이 운용 중 돌아가고 이 값은 무의미해진다 — 그 경우 IK 목표가
#    조용히 틀어지므로, 물리적으로 고정돼 있는지 반드시 확인할 것.
STATIC_JOINTS = {
    "arm_joint_1": 1.405,
}

# X 시리즈 Extended Position Control Mode 의 raw tick 한계(약 ±256회전).
DXL_EXTENDED_MIN_TICK = -1_048_575
DXL_EXTENDED_MAX_TICK = 1_048_575

# Profile Acceleration(108) / Velocity(112). 기본값 0 은 "최고속 즉시 이동" 이라
# 그리퍼가 움직일 때마다 순간 과전류로 토크가 풀린다(HW-8 실기, 재현율 100%,
# 명령 후 0.3초 내 트립). 트립이 풀리면 Hardware Error Status 도 0 으로 돌아가
# 나중에 보면 흔적이 안 남는다 — 반드시 기동 시 넣어야 한다(CLAUDE.md).
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
PROFILE_ACCELERATION = 25
PROFILE_VELOCITY = 80


def to_signed(value, byte_len):
    """무부호 정수를 byte_len 바이트 2의 보수 부호 정수로 변환."""
    bits = byte_len * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value


class MoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__("moveit_dynamixel_bridge")

        # --- 그리퍼 파라미터 (preset에 따라 단일/복수 모터 공통 처리) ---
        # gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter("end_effector_preset", DEFAULT_GRIPPER)
        self.gripper_type = self.get_parameter("end_effector_preset").value
        preset = get_preset(self.gripper_type, self.get_logger())
        self.declare_parameter("gripper_change_mode", False)
        self.declare_parameter("gripper_disabled", False)
        self.gripper_change_mode = bool(
            self.get_parameter("gripper_change_mode").value)
        self.gripper_disabled = (
            self.gripper_change_mode
            or bool(self.get_parameter("gripper_disabled").value))

        self.declare_parameter("gripper_joints", preset["gripper_joints"])
        self.declare_parameter("gripper_ids", preset["gripper_ids"])  # 빈 배열이면 그리퍼 비활성
        self.declare_parameter("gripper_open_rad", preset["gripper_open_rad"])
        self.declare_parameter("gripper_close_rad", preset["gripper_close_rad"])
        self.declare_parameter("gripper_open_tick", preset["gripper_open_tick"])
        self.declare_parameter("gripper_close_tick", preset["gripper_close_tick"])
        # 다회전 그리퍼 여부. preset 에 없으면 단일회전으로 본다(보수적 — 다회전을
        # 잘못 켜면 tick 이 wrap 없이 계속 나가 랙 끝단을 밀어붙인다).
        self.declare_parameter("gripper_extended", bool(preset.get("extended", False)))
        # 0 이면 쓰지 않는다(서보 기본 885=100% 유지). preset 주석에 값 근거 있음.
        self.declare_parameter("gripper_goal_pwm", int(preset.get("gripper_goal_pwm", 0)))
        self.declare_parameter(
            "gripper_command_calibrated", preset["command_calibrated"])
        self.declare_parameter(
            "gripper_observed_operating_mode",
            preset["observed_operating_mode"])
        self.declare_parameter(
            "gripper_required_operating_mode",
            preset["required_operating_mode"])
        self.declare_parameter("end_effector_kind", preset["kind"])
        self.declare_parameter(
            "end_effector_profile_acceleration",
            preset["profile_acceleration"])
        self.declare_parameter(
            "end_effector_profile_velocity", preset["profile_velocity"])
        self.declare_parameter(
            "end_effector_max_abs_current", preset["max_abs_current"])
        self.declare_parameter(
            "end_effector_stall_timeout", preset["stall_timeout"])
        self.declare_parameter(
            "end_effector_motion_timeout", preset["motion_timeout"])
        self.declare_parameter(
            "end_effector_goal_tolerance_ticks",
            preset["goal_tolerance_ticks"])
        self.declare_parameter("read_only", False)
        self.declare_parameter("gripper_only_mode", False)
        self.declare_parameter("integrated_test_mode", False)
        self.declare_parameter("random_demo_enabled", False)
        self.declare_parameter("arm_test_max_abs_current", 300)
        self.declare_parameter("arm_test_stall_timeout", 2.0)
        self.declare_parameter("arm_test_step_timeout", 8.0)
        self.declare_parameter("arm_test_goal_tolerance_ticks", 10)
        self.declare_parameter("trajectory_goal_tolerance_rad", 0.03)
        self.declare_parameter("trajectory_goal_timeout_s", 10.0)
        self.declare_parameter("trajectory_feedback_timeout_s", 0.5)

        self.gripper_joints = list(self.get_parameter("gripper_joints").value)
        self.gripper_ids = list(self.get_parameter("gripper_ids").value)
        if self.gripper_disabled:
            # 논리 그리퍼 관절과 MoveIt 형상은 유지하되 직렬 포트를 열기 전에 모든
            # 물리 그리퍼 ID를 제거한다. 모든 버스 등록, 피드백, 토크, PWM, 궤적
            # 경로는 이 목록을 하드웨어 대상의 기준으로 사용한다.
            self.gripper_ids = []
        self.gripper_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.gripper_open_tick = int(self.get_parameter("gripper_open_tick").value)
        self.gripper_close_tick = int(self.get_parameter("gripper_close_tick").value)
        self.gripper_extended = bool(self.get_parameter("gripper_extended").value)
        self.gripper_goal_pwm = int(self.get_parameter("gripper_goal_pwm").value)
        self.gripper_command_calibrated = bool(
            self.get_parameter("gripper_command_calibrated").value)
        self.gripper_observed_operating_mode = self.get_parameter(
            "gripper_observed_operating_mode").value
        self.gripper_required_operating_mode = int(self.get_parameter(
            "gripper_required_operating_mode").value)
        self.end_effector_kind = str(
            self.get_parameter("end_effector_kind").value)
        self.end_effector_profile_acceleration = int(self.get_parameter(
            "end_effector_profile_acceleration").value)
        self.end_effector_profile_velocity = int(self.get_parameter(
            "end_effector_profile_velocity").value)
        self.end_effector_max_abs_current = int(self.get_parameter(
            "end_effector_max_abs_current").value)
        self.end_effector_stall_timeout = float(self.get_parameter(
            "end_effector_stall_timeout").value)
        self.end_effector_motion_timeout = float(self.get_parameter(
            "end_effector_motion_timeout").value)
        self.end_effector_goal_tolerance_ticks = int(self.get_parameter(
            "end_effector_goal_tolerance_ticks").value)
        self.gripper_motor_endpoints = {
            int(dxl_id): {name: int(value) for name, value in endpoints.items()}
            for dxl_id, endpoints in preset.get("motor_endpoints", {}).items()
        }
        self.gripper_required_operating_modes = {
            int(dxl_id): int(mode) for dxl_id, mode in
            preset.get("required_operating_modes", {}).items()
        }
        self.gripper_observed_operating_modes = {
            int(dxl_id): int(mode) for dxl_id, mode in
            preset.get("observed_operating_modes", {}).items()
        }
        self.read_only = bool(self.get_parameter("read_only").value)
        self.gripper_only_mode = bool(
            self.get_parameter("gripper_only_mode").value)
        self.integrated_test_mode = bool(
            self.get_parameter("integrated_test_mode").value)
        self.random_demo_enabled = bool(
            self.get_parameter("random_demo_enabled").value)
        self.arm_test_max_abs_current = int(
            self.get_parameter("arm_test_max_abs_current").value)
        self.arm_test_stall_timeout = float(
            self.get_parameter("arm_test_stall_timeout").value)
        self.arm_test_step_timeout = float(
            self.get_parameter("arm_test_step_timeout").value)
        self.arm_test_goal_tolerance_ticks = int(
            self.get_parameter("arm_test_goal_tolerance_ticks").value)
        self.trajectory_goal_tolerance = float(
            self.get_parameter("trajectory_goal_tolerance_rad").value)
        self.trajectory_goal_timeout = float(
            self.get_parameter("trajectory_goal_timeout_s").value)
        self.trajectory_feedback_timeout = float(
            self.get_parameter("trajectory_feedback_timeout_s").value)

        self._bus_lock = threading.Lock()
        self._feedback_lock = threading.Lock()
        self._latest_arm_positions = {}
        self._latest_arm_feedback_time = None
        self._torque_states = {}
        self._random_arm_baseline = None

        # 최신 upstream의 실측 zero/direction/gear-ratio와 보수적 joint-limit을
        # 그대로 사용한다. 미확정 리밋이 하나라도 있으면 일반 arm write는 닫는다.
        self.declare_parameter("gear_ratios", [])
        self.gear_ratios = {}
        for entry in self.get_parameter("gear_ratios").value:
            name, _, value = str(entry).partition(":")
            if name not in JOINT_CONFIG:
                self.get_logger().warn(f"gear_ratios: 모르는 관절 '{name}' 무시")
                continue
            try:
                ratio = float(value)
            except ValueError:
                self.get_logger().warn(f"gear_ratios: '{entry}' 파싱 실패 — 무시")
                continue
            if ratio > 0.0:
                self.gear_ratios[name] = ratio

        self.arm_command_calibrated = not any(
            name in JOINT_CONFIG for name in joint_limits.provisional_joints()
        ) and all(joint_limits.get_limits(name) is not None for name in JOINT_CONFIG)
        if (not self.read_only and not self.gripper_only_mode
                and not self.integrated_test_mode
                and not self.arm_command_calibrated):
            raise RuntimeError(
                "Arm writes blocked: physical joint limits are not fully calibrated. "
                "Use read_only:=true or the explicitly bounded integrated_test_mode.")

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {DEVICENAME}")

        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate: {BAUDRATE}")

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

        # SyncRead 등록 ID와 이 프로세스가 토크를 켠 ID를 별도로 추적한다.
        # gripper-only/read-only에서는 register write 없이 그리퍼 ID만 active_ids에 등록된다.
        self.active_ids = set()
        self.torque_enabled_ids = set()

        if self.read_only or self.gripper_only_mode:
            # register write 없이 SyncRead 대상만 등록한다(토크 인가·Profile 설정 안 함).
            # read-only 는 **팔 서보까지** 등록한다 — 실기 구동 전에 ID 매핑/기어비/
            # tick→rad 환산을 /joint_states 로 눈으로 검증하는 게 이 모드의 목적이라,
            # 그리퍼만 읽으면 정작 검증할 대상이 안 보인다. gripper-only 는 이름대로
            # 그리퍼만 본다(팔은 다른 노드가 잡고 있을 수 있어 건드리지 않음).
            if not self.gripper_only_mode:
                for joint_name, config in JOINT_CONFIG.items():
                    if self.group_sync_read.addParam(config["id"]):
                        self.active_ids.add(config["id"])
            for gid in self.gripper_ids:
                if self.group_sync_read.addParam(gid):
                    self.active_ids.add(gid)
            if self.gripper_only_mode:
                self.get_logger().info(
                    "Gripper-only mode: monitoring gripper IDs only; "
                    "startup torque/position writes are disabled"
                )
            else:
                self.get_logger().info(
                    f"Read-only mode: 토크/레지스터 쓰기 없이 모니터링만 — "
                    f"팔 {sorted(ARM_IDS)} + 그리퍼 {self.gripper_ids}"
                )
        elif self.integrated_test_mode:
            # 진단 통합 모드는 같은 버스를 소유하지만 기동 시 쓰기를 수행하지 않는다.
            # 액션은 현재 테스트하는 모터만 활성화한다.
            for dxl_id in ARM_ID_SEQUENCE + list(self.gripper_ids):
                if self.group_sync_read.addParam(dxl_id):
                    self.active_ids.add(dxl_id)
            self.get_logger().info(
                "Integrated test mode: monitoring arm and selected end "
                "effector IDs; all startup writes are disabled")
        else:
            # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for joint_name, config in JOINT_CONFIG.items():
                required_mode = (MODE_EXTENDED_POSITION if config["extended"]
                                 else MODE_POSITION)
                if self._enable_torque(config["id"], joint_name, required_mode):
                    self.group_sync_read.addParam(config["id"])
                    self.active_ids.add(config["id"])
                    self.torque_enabled_ids.add(config["id"])

            # 그리퍼 기동 쓰기에는 별도로 검증한 끝점과 Goal Position이 기대하는 위치
            # 제어 모드가 필요하다. 모드 초기화는 명시적인 시운전 유틸리티가 담당하며,
            # 운영 브리지는 EEPROM/RAM 제어 모드를 변경하지 않는다.
            if self._gripper_startup_torque_allowed():
                for gid in self.gripper_ids:
                    if self._enable_torque(
                            gid, f"gripper(id {gid})",
                            self._required_gripper_mode(gid)):
                        self._write_gripper_goal_pwm(gid)
                        self.group_sync_read.addParam(gid)
                        self.active_ids.add(gid)
                        self.torque_enabled_ids.add(gid)
            else:
                for gid in self.gripper_ids:
                    if self.group_sync_read.addParam(gid):
                        self.active_ids.add(gid)
                self.get_logger().warn(
                    "Gripper startup torque blocked: endpoints or operating "
                    "mode are not commissioned")

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )

        # 벤치 teleop_core의 단일 관절 명령. 메시지는 [motor_id, goal_tick].
        # FSM/MoveIt 경로와 같은 GroupSyncWrite를 사용하되 알려진 팔 ID만 허용한다.
        self.teleop_goal_sub = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.teleop_goal_callback,
            10,
        )

        self._action_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.arm_goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # 그리퍼 액션 서버 (FSM 이 /gripper_controller/follow_joint_trajectory 로 파지/개방 명령)
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_gripper,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.rotate_action_server = ActionServer(
            self,
            EndEffectorRotate,
            "/end_effector/rotate",
            execute_callback=self.execute_rotate,
            goal_callback=self.rotate_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )

        self.arm_test_action_server = ActionServer(
            self,
            ArmTestMove,
            "/arm/test_move",
            execute_callback=self.execute_arm_test_move,
            goal_callback=self.arm_test_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
        )

        self.arm_recorded_path_action_server = ActionServer(
            self,
            ArmRecordedPath,
            "/arm/recorded_path",
            execute_callback=self.execute_arm_recorded_path,
            goal_callback=self.arm_recorded_path_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_group,
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

        self.feedback_timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info(
            f"MoveIt Dynamixel bridge started (arm={list(JOINT_CONFIG)}, "
            f"gripper_type={self.gripper_type}, gripper_ids={self.gripper_ids}, "
            f"gripper_change_mode={self.gripper_change_mode}, "
            f"gripper_disabled={self.gripper_disabled}, "
            f"read_only={self.read_only}, gripper_only_mode={self.gripper_only_mode})"
        )

    # ------------------------------------------------------------------ helpers
    def _write_motion_profile(self, dxl_id, label):
        """Profile Acceleration/Velocity 설정 — 토크 인가 **전에** 호출한다.

        기본값 0(=최고속 즉시 이동)이면 그리퍼가 움직일 때마다 순간 과전류로 토크가
        풀린다(HW-8 실기 검증, 재현율 100%). 팔 축도 같은 이유로 완만하게 둔다.
        """
        for addr, value, field in (
            (ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, "Profile Acceleration"),
            (ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, "Profile Velocity"),
        ):
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, addr, value
            )
            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"{field} 설정 실패: {label}, id={dxl_id}, "
                    f"result={result}, error={error} — 과전류 토크 트립 위험"
                )

    def _write_gripper_goal_pwm(self, dxl_id):
        """그리퍼 토크 상한(Goal PWM) 설정 — 파지 중 Overload 트립 방지.

        물체를 문 채 목표에 도달 못 하면 서보는 무한정 밀어붙이다 Overload 로 토크가
        끊긴다(2026-08-09 실기). 여기서 상한을 걸면 그 힘에서 멈춰 계속 물고 있는다.
        값 근거와 조정 방향은 `gripper_presets.py` 의 `gripper_goal_pwm` 주석 참고.
        """
        if self.gripper_goal_pwm <= 0:
            return
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, ADDR_GOAL_PWM, self.gripper_goal_pwm)
        if result != 0 or error != 0:
            self.get_logger().warn(
                f"Goal PWM 쓰기 실패: id={dxl_id}, result={result}, error={error} — "
                "토크 상한이 안 걸려 파지 중 Overload 트립 가능")
        else:
            self.get_logger().info(
                f"Goal PWM 설정: id={dxl_id} -> {self.gripper_goal_pwm} "
                f"(최대 885, 파지 토크 상한)")

    def _required_gripper_mode(self, dxl_id):
        return self.gripper_required_operating_modes.get(
            dxl_id, self.gripper_required_operating_mode)

    def _enable_torque(self, dxl_id, label, required_mode=None):
        """현재 위치를 goal로 검증 동기화한 뒤에만 해당 ID의 torque를 켠다."""
        try:
            with self._bus_lock:
                torque = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, "startup torque")
                if torque != TORQUE_DISABLE:
                    raise RuntimeError(
                        f"startup requires Torque OFF, readback={torque}")
                if required_mode is not None:
                    mode = self._read_register(
                        dxl_id, ADDR_OPERATING_MODE, 1,
                        "startup operating mode")
                    if mode != required_mode:
                        raise RuntimeError(
                            f"operating mode mismatch: expected "
                            f"{required_mode}, read {mode}; automatic mode "
                            "writes are disabled")
                present = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    "startup present position", signed=True)
                self._write_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    present & 0xFFFFFFFF, "startup synchronize goal")
                goal_readback = self._read_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    "startup goal readback", signed=True)
                # Startup synchronization is a fail-closed safety gate: even a
                # one-tick mismatch means the value written was not read back
                # exactly, so torque must remain disabled.
                if goal_readback != present:
                    raise RuntimeError(
                        f"Present->Goal readback mismatch: "
                        f"present={present}, goal={goal_readback}")
                self._write_motion_profile(dxl_id, label)
                self._write_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                    TORQUE_ENABLE, "startup torque enable")
                torque_readback = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                    "startup torque readback")
                if torque_readback != TORQUE_ENABLE:
                    raise RuntimeError(
                        f"Torque ON readback failed: {torque_readback}")
        except Exception as exc:
            self.get_logger().error(
                f"Torque enable blocked: {label}, id={dxl_id}: {exc}")
            return False
        self.get_logger().info(f"Torque enabled safely: {label} -> id {dxl_id}")
        return True

    def _joint_gear_ratio(self, joint_name):
        """실측으로 덮어쓸 수 있는 기어비(`gear_ratios` 파라미터 > JOINT_CONFIG 기본값)."""
        return self.gear_ratios.get(joint_name, JOINT_CONFIG[joint_name]["gear_ratio"])

    def _read_register(self, dxl_id, address, size, label, signed=False):
        reader = {
            1: self.packet_handler.read1ByteTxRx,
            2: self.packet_handler.read2ByteTxRx,
            4: self.packet_handler.read4ByteTxRx,
        }[size]
        value, result, error = reader(self.port_handler, dxl_id, address)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label} read failed: result={result}, error={error}")
        return to_signed(value, size) if signed else value

    def _write_register(self, dxl_id, address, size, value, label):
        writer = {
            1: self.packet_handler.write1ByteTxRx,
            4: self.packet_handler.write4ByteTxRx,
        }[size]
        result, error = writer(
            self.port_handler, dxl_id, address, value)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label} write failed: result={result}, error={error}")

    def rad_to_tick(self, joint_name, rad):
        """관절 rad → 서보 tick. 안전 리밋 clamp 후 기어비를 곱해 서보축 도메인으로 올린다.

        tick 범위 clamp(아래)는 "서보가 표현할 수 있는 값" 일 뿐 "관절이 안 부딪히는
        범위" 가 아니다 — 그래서 joint_limits 를 **먼저** 적용한다.
        """
        config = JOINT_CONFIG[joint_name]
        # clamp 를 조용히 하면 IK 버그가 "왜 목표에 안 닿지?" 로만 보인다 — 반드시 남긴다.
        rad, was_clamped = joint_limits.clamp(joint_name, rad)
        if was_clamped:
            lower, upper = joint_limits.get_limits(joint_name)
            self.get_logger().warn(
                f"{joint_name}: 목표각이 안전 범위를 벗어나 clamp 됨 "
                f"→ {rad:+.4f} rad (범위 [{lower:+.4f}, {upper:+.4f}])"
            )
        ticks_per_joint_rad = TICKS_PER_RAD * self._joint_gear_ratio(joint_name)
        tick = config["center"] + config["direction"] * rad * ticks_per_joint_rad
        tick = int(round(tick))
        if config["extended"]:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        """서보 tick → 관절 rad. rad_to_tick 의 역변환."""
        config = JOINT_CONFIG[joint_name]
        ticks_per_joint_rad = TICKS_PER_RAD * self._joint_gear_ratio(joint_name)
        return (tick - config["center"]) / (config["direction"] * ticks_per_joint_rad)

    def gripper_pos_to_tick(self, rad):
        self._require_gripper_command_mapping()
        span = self.gripper_open_tick - self.gripper_close_tick
        denom = self.gripper_open_rad - self.gripper_close_rad
        frac = (rad - self.gripper_close_rad) / denom
        tick = int(round(self.gripper_close_tick + frac * span))
        # 다회전 그리퍼는 끝단 tick 이 0~4095 밖으로 나간다(2026-08-07 실측 close=-401)
        # — 단일회전으로 clamp 하면 완전 닫힘이 tick 0 에서 잘려 덜 닫힌 채 멈춘다.
        if self.gripper_extended:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def gripper_pos_to_ratio(self, rad):
        """논리 관절 위치를 닫힘 비율(open=0, close=1)로 변환한다."""
        self._require_gripper_command_mapping()
        span = self.gripper_close_rad - self.gripper_open_rad
        ratio = (rad - self.gripper_open_rad) / span
        return max(0.0, min(1.0, ratio))

    def gripper_goals_for_ratio(self, ratio):
        """논리 닫힘 비율 하나를 개별 캘리브레이션된 모터 목표로 매핑한다."""
        self._require_gripper_command_mapping()
        ratio = max(0.0, min(1.0, float(ratio)))
        if not self.gripper_motor_endpoints:
            return {gid: self.gripper_pos_to_tick(
                self.gripper_open_rad + ratio * (
                    self.gripper_close_rad - self.gripper_open_rad))
                    for gid in self.gripper_ids}
        if set(self.gripper_motor_endpoints) != set(self.gripper_ids):
            raise RuntimeError("gripper endpoint IDs do not match selected IDs")
        return {
            gid: int(round(values["open"] + ratio * (
                values["close"] - values["open"])))
            for gid, values in self.gripper_motor_endpoints.items()
        }

    def gripper_tick_to_pos(self, tick):
        span = self.gripper_open_tick - self.gripper_close_tick
        if span == 0:
            return self.gripper_close_rad
        frac = (tick - self.gripper_close_tick) / span
        return self.gripper_close_rad + frac * (self.gripper_open_rad - self.gripper_close_rad)

    def gripper_velocity_to_rad_s(self, velocity_raw):
        """Present Velocity(raw, 0.229rev/min 단위) → gripper_tick_to_pos 와 같은 논리 rad/s.

        Present Velocity 는 서보축 물리 회전속도(4096tick/rev 고정, 데이터시트 상수)이고
        gripper_tick_to_pos 의 tick→rad 기울기는 open/close 캘리브 span 기반의 별도 계수라
        두 스케일을 직접 연결해야 한다: raw → 물리 tick/s(4096tick/rev 경유) → 캘리브
        기울기(rad/tick)로 환산. 부호/스케일은 실기 검증 전까지 확정 아님(Notion 절차 §2-3).
        """
        span = self.gripper_open_tick - self.gripper_close_tick
        if span == 0:
            return 0.0
        ticks_per_s = velocity_raw * (0.229 / 60.0) * DXL_TICKS_PER_REV
        rad_per_tick = (self.gripper_open_rad - self.gripper_close_rad) / span
        return ticks_per_s * rad_per_tick

    def int_to_little_endian_4bytes(self, value):
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def goal_callback(self, goal_request):
        if self.read_only:
            self.get_logger().warn("Read-only mode: rejecting trajectory goal")
            return GoalResponse.REJECT
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def _gripper_commands_allowed(self):
        """캘리브레이션된 위치 명령 동작일 때만 참을 반환한다."""
        if (not self.gripper_command_calibrated
                or self.gripper_open_rad == self.gripper_close_rad):
            return False
        if self.gripper_required_operating_modes:
            return (set(self.gripper_required_operating_modes)
                    == set(self.gripper_ids)
                    and self.gripper_observed_operating_modes
                    == self.gripper_required_operating_modes)
        return (self.gripper_observed_operating_mode
                == self.gripper_required_operating_mode)

    def _require_gripper_command_mapping(self):
        """안전성이 확인되지 않은 rad→tick 변환을 fail-closed로 차단한다."""
        if not self.gripper_command_calibrated:
            raise RuntimeError("gripper command calibration is not verified")
        if self.gripper_open_rad == self.gripper_close_rad:
            raise RuntimeError("gripper open/close rad endpoints are identical")

    def _gripper_startup_torque_allowed(self):
        """듀얼 그리퍼에만 기존 기동 동작을 허용한다."""
        return self.end_effector_kind == "gripper" \
            and self._gripper_commands_allowed()

    def gripper_goal_callback(self, goal_request):
        """모드와 끝점의 시운전이 끝날 때까지 그리퍼 동작을 거부한다."""
        if (getattr(self, "gripper_disabled", False)
                or self.end_effector_kind != "gripper" or self.read_only
                or not self._gripper_commands_allowed()):
            self.get_logger().warn(
                "Rejecting gripper goal: command calibration/mode gate closed")
            return GoalResponse.REJECT
        return self.goal_callback(goal_request)

    def rotate_goal_callback(self, goal_request):
        """선택한 단일축 회전 프리셋에 대해서만 회전을 수락한다."""
        if (self.read_only or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or not self._gripper_commands_allowed()):
            self.get_logger().warn(
                "Rejecting rotate goal: rotary_id5 preset is not active")
            return GoalResponse.REJECT
        if goal_request.max_abs_current < 0 or goal_request.timeout < 0.0:
            return GoalResponse.REJECT
        if not goal_request.relative and not 0 <= goal_request.ticks <= 4095:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def arm_test_goal_callback(self, goal_request):
        """명시적으로 승인된 4축 tick 시퀀스만 수락한다."""
        requested = tuple(zip(goal_request.motor_ids, goal_request.delta_ticks))
        fixed_sequence = requested == ARM_TEST_SEQUENCE
        random_sequence = (
            bool(goal_request.random_demo)
            and self.random_demo_enabled
            and tuple(goal_request.motor_ids) == tuple(ARM_ID_SEQUENCE)
            and len(goal_request.delta_ticks) == len(ARM_ID_SEQUENCE)
            and all(abs(int(delta)) <= 2 * RANDOM_ARM_RANGES[dxl_id]
                    for dxl_id, delta in requested)
        )
        if (self.read_only or not self.integrated_test_mode
                or self.gripper_only_mode
                or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or not (fixed_sequence and not goal_request.random_demo
                        or random_sequence)):
            self.get_logger().warn(
                "Rejecting arm test goal: mode/preset/sequence gate closed")
            return GoalResponse.REJECT
        if (goal_request.max_abs_current < 0
                or goal_request.stall_timeout < 0.0
                or goal_request.step_timeout < 0.0):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def split_recorded_path_request(goal_request):
        """평탄화된 signed 기록 경로 요청을 검증하고 분리한다."""
        motor_ids = tuple(int(v) for v in goal_request.motor_ids)
        counts = tuple(int(v) for v in goal_request.waypoint_counts)
        flat = tuple(int(v) for v in goal_request.signed_waypoints)
        if motor_ids != RECORDED_PATH_IDS:
            raise ValueError(
                f"motor_ids must be exactly {list(RECORDED_PATH_IDS)}")
        if len(counts) != len(motor_ids) or any(count <= 0 for count in counts):
            raise ValueError("one positive waypoint_count is required per motor")
        if sum(counts) != len(flat):
            raise ValueError("waypoint_counts do not match signed_waypoints")

        paths = []
        offset = 0
        for dxl_id, count in zip(motor_ids, counts):
            waypoints = flat[offset:offset + count]
            offset += count
            if dxl_id == 12 and any(not 0 <= value <= 4095
                                    for value in waypoints):
                raise ValueError("ID 12 Mode 3 waypoint outside [0, 4095]")
            deltas = [b - a for a, b in zip(waypoints, waypoints[1:])]
            if any(delta == 0 or abs(delta) > RECORDED_PATH_MAX_WAYPOINT_STEP
                   for delta in deltas):
                raise ValueError("waypoint step must be in [1, 50] ticks")
            signs = {1 if delta > 0 else -1 for delta in deltas}
            if len(signs) > 1:
                raise ValueError(f"ID {dxl_id} waypoint direction reverses")
            paths.append((dxl_id, waypoints))
        return paths

    def arm_recorded_path_goal_callback(self, goal_request):
        """명시적인 3축 signed 기록 경로 액션만 수락한다."""
        try:
            self.split_recorded_path_request(goal_request)
        except ValueError as exc:
            self.get_logger().warn(f"Rejecting recorded path: {exc}")
            return GoalResponse.REJECT
        if (self.read_only or not self.integrated_test_mode
                or self.gripper_only_mode
                or self.end_effector_kind != "rotary"
                or self.gripper_ids != [5]
                or int(goal_request.max_abs_current) <= 0
                or float(goal_request.stall_timeout) <= 0.0
                or float(goal_request.step_timeout) <= 0.0
                or not 1 <= int(goal_request.goal_tolerance) <= 10):
            self.get_logger().warn(
                "Rejecting recorded path: mode/preset/safety gate closed")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def arm_goal_callback(self, goal_request):
        if getattr(self, "integrated_test_mode", False):
            self.get_logger().error(
                "Integrated test mode: rejecting normal arm trajectory")
            return GoalResponse.REJECT
        if self.gripper_only_mode:
            self.get_logger().error(
                "Gripper-only mode: rejecting arm FollowJointTrajectory goal")
            return GoalResponse.REJECT
        return self.goal_callback(goal_request)

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ arm
    def teleop_goal_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Teleop goal must be [motor_id, goal_tick]")
            return

        dxl_id, goal_tick = (int(msg.data[0]), int(msg.data[1]))
        if dxl_id not in ARM_IDS:
            self.get_logger().warn(f"Unknown arm motor ID from teleop: {dxl_id}")
            return
        if (self.gripper_only_mode
                or getattr(self, "integrated_test_mode", False)):
            self.get_logger().error(
                f"Diagnostic mode: rejecting arm teleop command id={dxl_id}")
            return
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring teleop goal")
            return
        if dxl_id not in self.active_ids:
            self.get_logger().error(f"Inactive arm motor ID from teleop: {dxl_id}")
            return

        # 다회전(Extended Position) 축을 0~4095 로 clamp 하면 감속기 축이 한 바퀴
        # 넘는 순간 명령이 끝단에 박혀 더는 안 움직인다 — 축별 범위로 clamp 한다.
        if any(c["id"] == dxl_id and c["extended"] for c in JOINT_CONFIG.values()):
            goal_tick = max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, goal_tick))
        else:
            goal_tick = max(DXL_MINIMUM_POSITION_VALUE,
                            min(DXL_MAXIMUM_POSITION_VALUE, goal_tick))
        self.group_sync_write.clearParam()
        if not self.group_sync_write.addParam(
                dxl_id, self.int_to_little_endian_4bytes(goal_tick)):
            self.get_logger().warn(f"Failed to add teleop sync write param: id={dxl_id}")
            return
        result = self.group_sync_write.txPacket()
        self.group_sync_write.clearParam()
        if result != 0:
            self.get_logger().warn(f"Teleop GroupSyncWrite failed: result={result}")
            return
        self.get_logger().info(f"teleop -> id {dxl_id}: tick {goal_tick}")

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
        if (self.gripper_only_mode
                or getattr(self, "integrated_test_mode", False)):
            self.get_logger().error(
                "Diagnostic mode: rejecting arm trajectory topic command")
            return
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring arm trajectory")
            return
        if not msg.points:
            return

        point = msg.points[-1]

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn("JointTrajectory names/positions length mismatch")
            return

        self.group_sync_write.clearParam()
        added_any = False

        for joint_name, rad in zip(msg.joint_names, point.positions):
            if joint_name not in JOINT_CONFIG:
                self.get_logger().warn(f"Unknown joint from MoveIt: {joint_name}")
                continue

            dxl_id = JOINT_CONFIG[joint_name]["id"]
            if dxl_id not in self.active_ids:
                self.get_logger().error(
                    f"Inactive arm motor from trajectory: {joint_name}, id={dxl_id}"
                )
                continue
            goal_tick = self.rad_to_tick(joint_name, rad)
            param_goal_position = self.int_to_little_endian_4bytes(goal_tick)

            ok = self.group_sync_write.addParam(dxl_id, param_goal_position)
            if not ok:
                self.get_logger().warn(f"Failed to add sync write param: id={dxl_id}")
                continue
            added_any = True

            self.get_logger().info(
                f"{joint_name} -> id {dxl_id}: {rad:.3f} rad -> {goal_tick}"
            )

        if not added_any:
            self.get_logger().error("Trajectory contains no active arm motors")
            self.group_sync_write.clearParam()
            return

        result = self.group_sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(f"GroupSyncWrite failed: result={result}")

        self.group_sync_write.clearParam()

    # ------------------------------------------------------------------ gripper
    @staticmethod
    def arm_test_goal_reached(position, goal, velocity, tolerance):
        return abs(position - goal) <= tolerance and velocity == 0

    @staticmethod
    def recorded_direction_violation(delta, expected_direction, state):
        """짧은 반동은 허용하면서 지속되는 역방향 구간을 추적한다."""
        if delta * expected_direction < -2:
            state["samples"] += 1
            state["ticks"] += abs(delta)
            return state["samples"] >= 3 or state["ticks"] > 10
        if delta * expected_direction >= 0:
            state["samples"] = 0
            state["ticks"] = 0
        return False

    def execute_arm_test_move(self, goal_handle):
        """승인된 4축 tick 시퀀스를 모터 하나씩 실행한다."""
        request = goal_handle.request
        result = ArmTestMove.Result()
        completed = 0
        succeeded = False
        canceled = False
        reason = "unknown failure"
        selected = (
            self.integrated_test_mode
            and not self.read_only
            and not self.gripper_only_mode
            and self.end_effector_kind == "rotary"
            and self.gripper_ids == [5]
        )
        sequence = tuple(zip(request.motor_ids, request.delta_ticks))
        random_demo = bool(request.random_demo)
        current_limit = int(request.max_abs_current) \
            or self.arm_test_max_abs_current
        stall_timeout = float(request.stall_timeout) \
            or self.arm_test_stall_timeout
        step_timeout = float(request.step_timeout) \
            or self.arm_test_step_timeout

        try:
            if not selected:
                raise RuntimeError("integrated_test_mode + rotary_id5 required")
            if random_demo:
                if (not self.random_demo_enabled
                        or tuple(request.motor_ids) != tuple(ARM_ID_SEQUENCE)):
                    raise RuntimeError("random arm demo gate/ID sequence invalid")
            elif sequence != ARM_TEST_SEQUENCE:
                raise RuntimeError(
                    f"arm test sequence must be {ARM_TEST_SEQUENCE}")

            # 승인된 모든 팔 축이 torque-free 및 fault-free로 시작하지 않으면 안전
            # 측으로 실패한다. 이 액션은 의도적으로 ID 5를 건드리지 않는다.
            with self._bus_lock:
                starts = {}
                for dxl_id in ARM_ID_SEQUENCE:
                    torque = self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1, "arm torque")
                    hardware_error = self._read_register(
                        dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "arm hardware error")
                    starts[dxl_id] = self._read_register(
                        dxl_id, ADDR_PRESENT_POSITION, 4,
                        "arm starting position", signed=True)
                    if torque != TORQUE_DISABLE:
                        raise RuntimeError(
                            f"ID {dxl_id} Torque must start OFF")
                    if hardware_error != 0:
                        raise RuntimeError(
                            f"ID {dxl_id} hardware error "
                            f"0x{hardware_error:02X}")
                if random_demo:
                    id5_torque = self._read_register(
                        5, ADDR_TORQUE_ENABLE, 1, "ID5 torque")
                    id5_error = self._read_register(
                        5, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "ID5 hardware error")
                    if id5_torque != TORQUE_DISABLE or id5_error != 0:
                        raise RuntimeError(
                            "ID 5 must start Torque OFF with Hardware Error 0")
            if random_demo and self._random_arm_baseline is None:
                self._random_arm_baseline = starts

            for dxl_id, delta in sequence:
                if goal_handle.is_cancel_requested:
                    canceled = True
                    reason = "canceled before next arm step"
                    break
                with self._bus_lock:
                    mode = self._read_register(
                        dxl_id, ADDR_OPERATING_MODE, 1, "operating mode")
                    torque = self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1, "torque")
                    hardware_error = self._read_register(
                        dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "hardware error")
                    start = self._read_register(
                        dxl_id, ADDR_PRESENT_POSITION, 4,
                        "present position", signed=True)
                if mode not in (3, 4):
                    raise RuntimeError(
                        f"ID {dxl_id} position mode must be 3 or 4, got {mode}")
                if torque != TORQUE_DISABLE:
                    raise RuntimeError(f"ID {dxl_id} Torque is not OFF")
                if hardware_error != 0:
                    raise RuntimeError(
                        f"ID {dxl_id} hardware error 0x{hardware_error:02X}")
                goal = start + int(delta)
                if random_demo:
                    baseline = self._random_arm_baseline[dxl_id]
                    offset = goal - baseline
                    if (abs(offset) < 5
                            or abs(offset) > RANDOM_ARM_RANGES[dxl_id]):
                        raise RuntimeError(
                            f"ID {dxl_id} random target offset {offset} "
                            f"outside approved range")
                if mode == 3 and not 0 <= goal <= 4095:
                    raise RuntimeError(
                        f"ID {dxl_id} goal {goal} outside [0, 4095]")
                encoded_goal = goal & 0xFFFFFFFF

                try:
                    with self._bus_lock:
                        self._write_register(
                            dxl_id, ADDR_GOAL_POSITION, 4,
                            start & 0xFFFFFFFF, "synchronize arm goal")
                        synced_goal = self._read_register(
                            dxl_id, ADDR_GOAL_POSITION, 4,
                            "synchronized arm goal readback", signed=True)
                        if synced_goal != start:
                            raise RuntimeError(
                                f"ID {dxl_id} Present->Goal readback mismatch: "
                                f"present={start}, goal={synced_goal}")
                        self._write_register(
                            dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_ENABLE,
                            "arm test torque enable")
                        self.torque_enabled_ids.add(dxl_id)
                        self._write_register(
                            dxl_id, ADDR_GOAL_POSITION, 4, encoded_goal,
                            "arm test goal")

                    deadline = time.monotonic() + step_timeout
                    last_progress_time = time.monotonic()
                    last_progress_position = start
                    step_complete = False
                    while time.monotonic() < deadline:
                        if goal_handle.is_cancel_requested:
                            canceled = True
                            reason = f"canceled during ID {dxl_id}"
                            break
                        with self._bus_lock:
                            position = self._read_register(
                                dxl_id, ADDR_PRESENT_POSITION, 4,
                                "present position", signed=True)
                            velocity = self._read_register(
                                dxl_id, ADDR_PRESENT_VELOCITY, 4,
                                "present velocity", signed=True)
                            current = self._read_register(
                                dxl_id, ADDR_PRESENT_LOAD, 2,
                                "present current", signed=True)
                            moving_status = self._read_register(
                                dxl_id, ADDR_MOVING_STATUS, 1,
                                "moving status")
                            hardware_error = self._read_register(
                                dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                                "hardware error")
                        feedback = ArmTestMove.Feedback()
                        feedback.motor_id = dxl_id
                        feedback.goal_position = goal
                        feedback.present_position = position
                        feedback.present_velocity = velocity
                        feedback.present_current = current
                        feedback.moving_status = moving_status
                        feedback.hardware_error = hardware_error
                        goal_handle.publish_feedback(feedback)
                        if hardware_error:
                            raise RuntimeError(
                                f"ID {dxl_id} hardware error "
                                f"0x{hardware_error:02X}")
                        if abs(current) >= current_limit:
                            raise RuntimeError(
                                f"ID {dxl_id} abs(current) {abs(current)} "
                                f"reached limit {current_limit}")
                        if self.arm_test_goal_reached(
                                position, goal, velocity,
                                self.arm_test_goal_tolerance_ticks):
                            step_complete = True
                            break
                        if abs(position - last_progress_position) >= 2:
                            last_progress_position = position
                            last_progress_time = time.monotonic()
                        elif (time.monotonic() - last_progress_time
                                >= stall_timeout):
                            raise RuntimeError(f"ID {dxl_id} position stalled")
                        time.sleep(0.05)
                    if canceled:
                        break
                    if not step_complete:
                        raise RuntimeError(f"ID {dxl_id} step timeout")
                finally:
                    with self._bus_lock:
                        self._write_register(
                            dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLE,
                            "arm step torque disable")
                        torque_readback = self._read_register(
                            dxl_id, ADDR_TORQUE_ENABLE, 1,
                            "arm step torque readback")
                    self.torque_enabled_ids.discard(dxl_id)
                    if torque_readback != TORQUE_DISABLE:
                        raise RuntimeError(
                            f"ID {dxl_id} Torque OFF readback failed: "
                            f"{torque_readback}")
                completed += 1

            if not canceled and completed == len(sequence):
                succeeded = True
                reason = "approved arm test sequence completed"
        except Exception as exc:
            reason = str(exc)
        finally:
            # 이 진단 모드를 선택하면 아직 도달하지 않은 축을 포함해 모든 종료
            # 경로에서 4개 ID를 명시적으로 비활성화하고 확인한다.
            if selected:
                final_errors = []
                for dxl_id in ARM_ID_SEQUENCE:
                    try:
                        with self._bus_lock:
                            self._write_register(
                                dxl_id, ADDR_TORQUE_ENABLE, 1,
                                TORQUE_DISABLE, "final arm torque disable")
                            torque = self._read_register(
                                dxl_id, ADDR_TORQUE_ENABLE, 1,
                                "final arm torque readback")
                        self.torque_enabled_ids.discard(dxl_id)
                        if torque != TORQUE_DISABLE:
                            final_errors.append(
                                f"ID {dxl_id} torque readback={torque}")
                    except Exception as exc:
                        final_errors.append(f"ID {dxl_id}: {exc}")
                if random_demo:
                    try:
                        with self._bus_lock:
                            self._write_register(
                                5, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLE,
                                "final random demo ID5 torque disable")
                            id5_torque = self._read_register(
                                5, ADDR_TORQUE_ENABLE, 1,
                                "final random demo ID5 torque readback")
                        self.torque_enabled_ids.discard(5)
                        if id5_torque != TORQUE_DISABLE:
                            final_errors.append(
                                f"ID 5 torque readback={id5_torque}")
                    except Exception as exc:
                        final_errors.append(f"ID 5: {exc}")
                if final_errors:
                    succeeded = False
                    reason = "final Torque OFF failed: " + "; ".join(final_errors)

        result.success = succeeded
        result.completed_steps = completed
        result.reason = reason
        if canceled:
            goal_handle.canceled()
        elif succeeded:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def execute_arm_recorded_path(self, goal_handle):
        """ID 16이나 ID 5에 쓰지 않고 검증된 signed 경로를 재생한다."""
        request = goal_handle.request
        result = ArmRecordedPath.Result()
        completed = 0
        succeeded = False
        canceled = False
        reason = "unknown failure"
        selected = (
            self.integrated_test_mode
            and not self.read_only
            and not self.gripper_only_mode
            and self.end_effector_kind == "rotary"
            and self.gripper_ids == [5]
        )
        try:
            if not selected:
                raise RuntimeError("integrated_test_mode + rotary_id5 required")
            paths = self.split_recorded_path_request(request)
            current_limit = int(request.max_abs_current)
            stall_timeout = float(request.stall_timeout)
            step_timeout = float(request.step_timeout)
            tolerance = int(request.goal_tolerance)

            # 안전 상태 5개를 모두 읽되 ID 16과 ID 5에는 절대 쓰지 않는다.
            starts = {}
            with self._bus_lock:
                for dxl_id in (*RECORDED_PATH_IDS, 16, 5):
                    torque = self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1, "recorded path torque")
                    hardware_error = self._read_register(
                        dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "recorded path hardware error")
                    starts[dxl_id] = self._read_register(
                        dxl_id, ADDR_PRESENT_POSITION, 4,
                        "recorded path starting position", signed=True)
                    if torque != TORQUE_DISABLE:
                        raise RuntimeError(f"ID {dxl_id} Torque must start OFF")
                    if hardware_error != 0:
                        raise RuntimeError(
                            f"ID {dxl_id} hardware error 0x{hardware_error:02X}")
                for dxl_id in RECORDED_PATH_IDS:
                    mode = self._read_register(
                        dxl_id, ADDR_OPERATING_MODE, 1, "operating mode")
                    required_mode = 4 if dxl_id in (14, 13) else 3
                    if mode != required_mode:
                        raise RuntimeError(
                            f"ID {dxl_id} mode must be {required_mode}, got {mode}")

            for dxl_id, waypoints in paths:
                start_error = abs(starts[dxl_id] - waypoints[0])
                allowed = RECORDED_PATH_START_TOLERANCE[dxl_id]
                if start_error > allowed:
                    raise RuntimeError(
                        f"ID {dxl_id} start error {start_error} exceeds {allowed}")

            for dxl_id, waypoints in paths:
                # 앞 관절이 torque-free 상태의 뒤 관절을 기계적으로 움직일 수 있으므로
                # 각 축 실행 직전에 시작 게이트를 다시 확인한다.
                with self._bus_lock:
                    phase_start = self._read_register(
                        dxl_id, ADDR_PRESENT_POSITION, 4,
                        "recorded axis starting position", signed=True)
                    phase_torque = self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1,
                        "recorded axis starting torque")
                    phase_error = self._read_register(
                        dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "recorded axis starting hardware error")
                allowed = RECORDED_PATH_START_TOLERANCE[dxl_id]
                if abs(phase_start - waypoints[0]) > allowed:
                    raise RuntimeError(
                        f"ID {dxl_id} phase start error "
                        f"{abs(phase_start - waypoints[0])} exceeds {allowed}")
                if phase_torque != TORQUE_DISABLE or phase_error != 0:
                    raise RuntimeError(
                        f"ID {dxl_id} unsafe phase start torque={phase_torque} "
                        f"error={phase_error}")
                try:
                    # 한 번 동기화한 뒤 중력 반동을 막도록 이 축의 모든 waypoint에서
                    # 토크를 유지한다.
                    with self._bus_lock:
                        self._write_register(
                            dxl_id, ADDR_GOAL_POSITION, 4,
                            phase_start & 0xFFFFFFFF,
                            "synchronize recorded axis goal")
                        synced_goal = self._read_register(
                            dxl_id, ADDR_GOAL_POSITION, 4,
                            "synchronized recorded goal readback", signed=True)
                        if synced_goal != phase_start:
                            raise RuntimeError(
                                f"ID {dxl_id} Present->Goal readback mismatch: "
                                f"present={phase_start}, goal={synced_goal}")
                        self._write_register(
                            dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_ENABLE,
                            "recorded path axis torque enable")
                    self.torque_enabled_ids.add(dxl_id)

                    for waypoint_index, goal in enumerate(waypoints):
                        if goal_handle.is_cancel_requested:
                            canceled = True
                            reason = "canceled before next recorded waypoint"
                            break
                        with self._bus_lock:
                            hardware_error = self._read_register(
                                dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                                "hardware error")
                            start = self._read_register(
                                dxl_id, ADDR_PRESENT_POSITION, 4,
                                "present position", signed=True)
                        if hardware_error != 0:
                            raise RuntimeError(
                                f"ID {dxl_id} hardware error "
                                f"0x{hardware_error:02X}")
                        if abs(start - goal) <= tolerance:
                            completed += 1
                            continue
                        expected_direction = 1 if goal > start else -1
                        encoded_goal = goal & 0xFFFFFFFF
                        with self._bus_lock:
                            self._write_register(
                                dxl_id, ADDR_GOAL_POSITION, 4, encoded_goal,
                                "recorded path goal")

                        deadline = time.monotonic() + step_timeout
                        last_progress_time = time.monotonic()
                        last_progress_position = start
                        last_observed_position = start
                        reverse_state = {"samples": 0, "ticks": 0}
                        step_complete = False
                        while time.monotonic() < deadline:
                            if goal_handle.is_cancel_requested:
                                canceled = True
                                reason = f"canceled during ID {dxl_id}"
                                break
                            with self._bus_lock:
                                position = self._read_register(
                                    dxl_id, ADDR_PRESENT_POSITION, 4,
                                    "present position", signed=True)
                                velocity = self._read_register(
                                    dxl_id, ADDR_PRESENT_VELOCITY, 4,
                                    "present velocity", signed=True)
                                current = self._read_register(
                                    dxl_id, ADDR_PRESENT_LOAD, 2,
                                    "present current", signed=True)
                                moving_status = self._read_register(
                                    dxl_id, ADDR_MOVING_STATUS, 1,
                                    "moving status")
                                hardware_error = self._read_register(
                                    dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                                    "hardware error")
                            feedback = ArmRecordedPath.Feedback()
                            feedback.motor_id = dxl_id
                            feedback.waypoint_index = waypoint_index
                            feedback.goal_position = goal
                            feedback.present_position = position
                            feedback.present_velocity = velocity
                            feedback.present_current = current
                            feedback.moving_status = moving_status
                            feedback.hardware_error = hardware_error
                            goal_handle.publish_feedback(feedback)
                            if hardware_error:
                                raise RuntimeError(
                                    f"ID {dxl_id} hardware error "
                                    f"0x{hardware_error:02X}")
                            if abs(current) >= current_limit:
                                raise RuntimeError(
                                    f"ID {dxl_id} abs(current) {abs(current)} "
                                    f"reached limit {current_limit}")
                            observed_delta = position - last_observed_position
                            if self.recorded_direction_violation(
                                    observed_delta, expected_direction,
                                    reverse_state):
                                raise RuntimeError(
                                    f"ID {dxl_id} sustained opposite movement "
                                    f"samples={reverse_state['samples']} "
                                    f"ticks={reverse_state['ticks']}")
                            last_observed_position = position
                            if self.arm_test_goal_reached(
                                    position, goal, velocity, tolerance):
                                step_complete = True
                                break
                            if abs(position - last_progress_position) >= 2:
                                last_progress_position = position
                                last_progress_time = time.monotonic()
                            elif time.monotonic() - last_progress_time >= stall_timeout:
                                raise RuntimeError(f"ID {dxl_id} position stalled")
                            time.sleep(0.05)
                        if canceled:
                            break
                        if not step_complete:
                            raise RuntimeError(
                                f"ID {dxl_id} waypoint {waypoint_index} timeout")
                        completed += 1
                finally:
                    with self._bus_lock:
                        self._write_register(
                            dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLE,
                            "recorded axis torque disable")
                        torque_readback = self._read_register(
                            dxl_id, ADDR_TORQUE_ENABLE, 1,
                            "recorded axis torque readback")
                    if torque_readback != TORQUE_DISABLE:
                        raise RuntimeError(
                            f"ID {dxl_id} Torque OFF readback failed")
                    self.torque_enabled_ids.discard(dxl_id)
                if canceled:
                    break

            if not canceled and completed == sum(request.waypoint_counts):
                succeeded = True
                reason = "recorded path completed"
        except Exception as exc:
            reason = str(exc)
        finally:
            final_errors = []
            if selected:
                # 제어 대상 팔 ID에만 안전 쓰기를 수행한다. ID 16과 ID 5는 이 액션
                # 전체에서 의도적으로 읽기 전용이다.
                for dxl_id in RECORDED_PATH_IDS:
                    try:
                        with self._bus_lock:
                            torque = self._read_register(
                                dxl_id, ADDR_TORQUE_ENABLE, 1,
                                "final recorded path torque readback")
                            if torque != TORQUE_DISABLE:
                                self._write_register(
                                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                                    TORQUE_DISABLE,
                                    "final recorded path torque disable")
                                torque = self._read_register(
                                    dxl_id, ADDR_TORQUE_ENABLE, 1,
                                    "final recorded path torque re-readback")
                            hardware_error = self._read_register(
                                dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                                "final recorded path hardware error")
                        self.torque_enabled_ids.discard(dxl_id)
                        if torque != TORQUE_DISABLE or hardware_error != 0:
                            final_errors.append(
                                f"ID {dxl_id} torque={torque} "
                                f"error={hardware_error}")
                    except Exception as exc:
                        final_errors.append(f"ID {dxl_id}: {exc}")
                for dxl_id in (16, 5):
                    try:
                        with self._bus_lock:
                            torque = self._read_register(
                                dxl_id, ADDR_TORQUE_ENABLE, 1,
                                "untouched final torque readback")
                            hardware_error = self._read_register(
                                dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                                "untouched final hardware error")
                        if torque != TORQUE_DISABLE or hardware_error != 0:
                            final_errors.append(
                                f"ID {dxl_id} torque={torque} error={hardware_error}")
                    except Exception as exc:
                        final_errors.append(f"ID {dxl_id}: {exc}")
            if final_errors:
                succeeded = False
                reason = "final safety readback failed: " + "; ".join(final_errors)

        result.success = succeeded
        result.completed_waypoints = completed
        result.reason = reason
        if canceled:
            goal_handle.canceled()
        elif succeeded:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def rotation_goal(start, relative, ticks):
        """wraparound 문제 없이 tick 공간의 회전 요청을 계산한다."""
        goal = start + int(ticks) if relative else int(ticks)
        if not DXL_MINIMUM_POSITION_VALUE <= goal <= DXL_MAXIMUM_POSITION_VALUE:
            raise ValueError(
                f"goal {goal} outside Position Mode limits [0, 4095]")
        return goal

    def execute_rotate(self, goal_handle):
        """회전 ID 5 프리셋에 대해 보호된 tick 공간 이동을 한 번 실행한다."""
        request = goal_handle.request
        result = EndEffectorRotate.Result()
        dxl_id = 5
        start = None
        final = None
        maximum_current = 0
        reason = "unknown failure"
        succeeded = False
        canceled = False
        selected = False
        current_limit = int(request.max_abs_current) \
            or self.end_effector_max_abs_current
        timeout = float(request.timeout) or self.end_effector_motion_timeout
        try:
            if (self.end_effector_kind != "rotary"
                    or self.gripper_ids != [dxl_id]):
                raise RuntimeError("rotary_id5 preset is not active")
            selected = True
            with self._bus_lock:
                mode = self._read_register(
                    dxl_id, ADDR_OPERATING_MODE, 1, "operating mode")
                torque = self._read_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, "torque")
                hardware_error = self._read_register(
                    dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                    "hardware error")
                start_signed = self._read_register(
                    dxl_id, ADDR_PRESENT_POSITION, 4,
                    "present position", signed=True)
                start = start_signed % 4096
            if mode != 3:
                raise RuntimeError(f"Operating Mode must be 3, got {mode}")
            if torque != 0:
                raise RuntimeError("Torque must be OFF before rotate action")
            if hardware_error != 0:
                raise RuntimeError(
                    f"hardware error 0x{hardware_error:02X}")
            goal = self.rotation_goal(start, request.relative, request.ticks)

            with self._bus_lock:
                self._write_register(
                    dxl_id, ADDR_GOAL_POSITION, 4, start,
                    "synchronize goal")
                synced_goal = self._read_register(
                    dxl_id, ADDR_GOAL_POSITION, 4,
                    "synchronized goal readback", signed=True)
                if synced_goal != start:
                    raise RuntimeError(
                        f"ID {dxl_id} Present->Goal readback mismatch: "
                        f"present={start}, goal={synced_goal}")
                self._write_register(
                    dxl_id, ADDR_PROFILE_ACCELERATION, 4,
                    self.end_effector_profile_acceleration,
                    "profile acceleration")
                self._write_register(
                    dxl_id, ADDR_PROFILE_VELOCITY, 4,
                    self.end_effector_profile_velocity,
                    "profile velocity")
                self._write_register(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_ENABLE,
                    "torque enable")
                self.torque_enabled_ids.add(dxl_id)
                self._write_register(
                    dxl_id, ADDR_GOAL_POSITION, 4, goal,
                    "rotate goal")

            deadline = time.monotonic() + timeout
            last_progress_time = time.monotonic()
            last_progress_position = start
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    canceled = True
                    reason = "canceled"
                    break
                with self._bus_lock:
                    position_signed = self._read_register(
                        dxl_id, ADDR_PRESENT_POSITION, 4,
                        "present position", signed=True)
                    velocity = self._read_register(
                        dxl_id, ADDR_PRESENT_VELOCITY, 4,
                        "present velocity", signed=True)
                    current = self._read_register(
                        dxl_id, ADDR_PRESENT_LOAD, 2,
                        "present current", signed=True)
                    moving_status = self._read_register(
                        dxl_id, ADDR_MOVING_STATUS, 1, "moving status")
                    hardware_error = self._read_register(
                        dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "hardware error")
                final = position_signed % 4096
                maximum_current = max(maximum_current, abs(current))
                feedback = EndEffectorRotate.Feedback()
                feedback.goal_position = goal
                feedback.present_position = final
                feedback.present_velocity = velocity
                feedback.present_current = current
                feedback.moving_status = moving_status
                feedback.hardware_error = hardware_error
                goal_handle.publish_feedback(feedback)
                if hardware_error:
                    reason = f"hardware error 0x{hardware_error:02X}"
                    break
                if abs(current) >= current_limit:
                    reason = (
                        f"abs(current) {abs(current)} reached limit "
                        f"{current_limit}")
                    break
                if abs(final - goal) <= self.end_effector_goal_tolerance_ticks \
                        and velocity == 0:
                    succeeded = True
                    reason = "goal reached"
                    break
                if abs(final - last_progress_position) >= 2:
                    last_progress_position = final
                    last_progress_time = time.monotonic()
                elif time.monotonic() - last_progress_time \
                        >= self.end_effector_stall_timeout:
                    reason = "position stalled"
                    break
                time.sleep(0.1)
            else:
                reason = "motion timeout"
        except Exception as exc:
            reason = str(exc)
        finally:
            try:
                # 다른 프리셋을 선택했을 때는 ID 5를 절대 건드리지 않는다.
                # rotary_id5 선택 시 모든 종료 경로는 반드시 토크 OFF로 끝나야 한다.
                if not selected:
                    raise RuntimeError(
                        "no Torque write: rotary_id5 preset is not active")
                with self._bus_lock:
                    self._write_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLE,
                        "final torque disable")
                    torque_readback = self._read_register(
                        dxl_id, ADDR_TORQUE_ENABLE, 1,
                        "final torque readback")
                self.torque_enabled_ids.discard(dxl_id)
                if torque_readback != TORQUE_DISABLE:
                    succeeded = False
                    reason = f"Torque OFF readback failed: {torque_readback}"
            except Exception as exc:
                succeeded = False
                if selected:
                    reason = f"final Torque OFF failed: {exc}"

        if final is None:
            final = start if start is not None else 0
        result.success = succeeded
        result.final_position = int(final)
        result.actual_delta = int(final - start) if start is not None else 0
        result.max_abs_current = int(maximum_current)
        result.reason = reason
        if canceled:
            goal_handle.canceled()
        elif succeeded:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def execute_gripper(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        result = FollowJointTrajectory.Result()

        if (getattr(self, "gripper_disabled", False)
                or self.end_effector_kind != "gripper"
                or not self._gripper_commands_allowed()):
            self.get_logger().error(
                "Gripper execution blocked: command calibration/mode gate closed")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Gripper command calibration or mode is not verified"
            return result

        if not self.gripper_ids:
            self.get_logger().warn("Gripper goal received but gripper_ids is empty — ignored")
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        if trajectory.points:
            point = trajectory.points[-1]
            name_to_pos = dict(zip(trajectory.joint_names, point.positions))
            # Preset의 논리 구동 조인트 하나를 실제 gripper_ids 전체에 매핑한다.
            target_rad = None
            for jn in self.gripper_joints:
                if jn in name_to_pos:
                    target_rad = name_to_pos[jn]
                    break
            if target_rad is not None:
                if not self._write_gripper(target_rad):
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "Dual gripper motion failed; both motors torqued off"
                    return result
            else:
                self.get_logger().warn(
                    f"Gripper goal has no known finger joint {self.gripper_joints}"
                )

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Gripper command sent to Dynamixel"
        return result

    def _torque_off_gripper(self, reason):
        """최선 노력 방식의 동시 정지: 한 모터의 fault도 두 모터를 모두 끈다."""
        failures = []
        for gid in self.gripper_ids:
            try:
                self._write_register(
                    gid, ADDR_TORQUE_ENABLE, 1, TORQUE_DISABLE,
                    f"dual gripper emergency stop ({reason})")
                self.torque_enabled_ids.discard(gid)
            except Exception as exc:
                failures.append(f"ID {gid}: {exc}")
        if failures:
            self.get_logger().error(
                f"Dual gripper Torque OFF incomplete ({reason}): "
                + "; ".join(failures))
            return False
        self.get_logger().error(
            f"Dual gripper Torque OFF: IDs {self.gripper_ids} ({reason})")
        return True

    def _write_gripper(self, rad):
        if (getattr(self, "gripper_disabled", False)
                or self.end_effector_kind != "gripper" or self.read_only
                or not self._gripper_commands_allowed()):
            self.get_logger().warn(
                "Ignoring gripper command: read-only/calibration/mode gate closed")
            return False

        ratio = self.gripper_pos_to_ratio(rad)
        goals = self.gripper_goals_for_ratio(ratio)
        positions = {}
        last_progress = {}
        try:
            with self._bus_lock:
                for gid in self.gripper_ids:
                    mode = self._read_register(
                        gid, ADDR_OPERATING_MODE, 1, "gripper operating mode")
                    required = self._required_gripper_mode(gid)
                    if mode != required:
                        raise RuntimeError(
                            f"ID {gid} mode {mode}, expected {required}")
                    hardware_error = self._read_register(
                        gid, ADDR_HARDWARE_ERROR_STATUS, 1,
                        "gripper hardware error")
                    if hardware_error:
                        raise RuntimeError(
                            f"ID {gid} hardware error 0x{hardware_error:02X}")
                    positions[gid] = self._read_register(
                        gid, ADDR_PRESENT_POSITION, 4,
                        "gripper start position", signed=True)
                for gid, goal in goals.items():
                    self._write_register(
                        gid, ADDR_GOAL_POSITION, 4, goal & 0xFFFFFFFF,
                        f"gripper ratio {ratio:.4f} goal")

            now = time.monotonic()
            last_progress = {gid: now for gid in self.gripper_ids}
            deadline = now + self.end_effector_motion_timeout
            while time.monotonic() < deadline:
                reached = True
                with self._bus_lock:
                    for gid in self.gripper_ids:
                        position = self._read_register(
                            gid, ADDR_PRESENT_POSITION, 4,
                            "gripper present position", signed=True)
                        current = self._read_register(
                            gid, ADDR_PRESENT_LOAD, 2,
                            "gripper present current", signed=True)
                        hardware_error = self._read_register(
                            gid, ADDR_HARDWARE_ERROR_STATUS, 1,
                            "gripper hardware error")
                        if hardware_error:
                            raise RuntimeError(
                                f"ID {gid} hardware error 0x{hardware_error:02X}")
                        if abs(current) >= self.end_effector_max_abs_current:
                            raise RuntimeError(
                                f"ID {gid} abs(current) {abs(current)} reached "
                                f"limit {self.end_effector_max_abs_current}")
                        error_ticks = abs(goals[gid] - position)
                        if error_ticks > self.end_effector_goal_tolerance_ticks:
                            reached = False
                            if abs(position - positions[gid]) >= 2:
                                positions[gid] = position
                                last_progress[gid] = time.monotonic()
                            elif (time.monotonic() - last_progress[gid]
                                  >= self.end_effector_stall_timeout):
                                raise RuntimeError(
                                    f"ID {gid} position stalled")
                if reached:
                    self.get_logger().info(
                        f"gripper ratio={ratio:.4f}, goals={goals} reached")
                    return True
                time.sleep(0.05)
            raise RuntimeError("dual gripper motion timeout")
        except Exception as exc:
            self._torque_off_gripper(str(exc))
            return False

    # ------------------------------------------------------------------ feedback
    def publish_joint_states(self):
        self.group_sync_read.txRxPacket()
        # 일부 ID가 버스에 없어도 응답받은 ID만 처리 (result 무시)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # controller fault 집계 — SyncRead 에 등록된(토크 ON 성공) ID 중 하나라도
        # Hardware Error Status != 0 이거나 이번 tick 응답이 없으면 fault=True.
        # 응답 없음도 fault 로 보는 이유: 활성 등록된 서보가 갑자기 무응답이면 버스/전원
        # 이상일 수 있어 "정상"으로 오인하면 안 됨(안전 측 기본값).
        fault = False if self.gripper_only_mode else not ARM_IDS.issubset(
            self.active_ids)

        # 모터 없는 관절(STATIC_JOINTS)을 먼저 실어 TF 트리가 끊기지 않게 한다.
        # gripper-only 모드에선 팔을 아예 다루지 않으므로 제외.
        if not self.gripper_only_mode:
            for joint_name, fixed_rad in STATIC_JOINTS.items():
                msg.name.append(joint_name)
                msg.position.append(fixed_rad)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)

        # 팔 관절: position(rad) + address-126 feedback(raw signed).
        # 주소 126의 의미는 실제 장착 모터 control table로 확인해야 한다.
        if not self.gripper_only_mode:
            for joint_name, config in JOINT_CONFIG.items():
                dxl_id = config["id"]
                if dxl_id not in self.active_ids:
                    continue
                sample = self._read_sample(dxl_id)
                if sample is None:
                    fault = True
                    continue
                feedback_raw, tick, hw_error, velocity_raw = sample
                if hw_error != 0:
                    fault = True
                # Extended Position 축은 tick 이 4095 를 넘고 음수로도 내려가므로
                # 부호 있는 정수로 해석해야 한다 — 안 하면 다회전 축이 한 바퀴 넘는
                # 순간 위치가 갑자기 +2^32 쪽으로 튄다.
                signed_tick = to_signed(tick, LEN_PRESENT_POSITION)
                msg.name.append(joint_name)
                msg.position.append(self.tick_to_rad(joint_name, signed_tick))
                # 속도도 위치와 같은 관절 도메인으로 맞춘다 — 기어비만큼 관절이 느리다.
                msg.velocity.append(
                    velocity_raw * VELOCITY_LSB_TO_RAD_S
                    / (config["direction"] * self._joint_gear_ratio(joint_name))
                )
                msg.effort.append(float(feedback_raw))

        # XL430-W250 그리퍼: 주소 126은 signed Present Load(0.1% 추정 부하)다.
        # 랙피니언 2모터(ID 3,4)를 함께 읽어 하나의 논리 조인트(gripper_left_pinion_joint)로
        # 보고한다 — position(rad)=대표(첫 응답) 모터 tick, effort=가장 큰 abs(load).
        # 한 모터라도 부하가 크면 파지로 보는 보수적(안전 측) 집계이며, FSM 이 이 effort 로
        # 파지/DROP 을 판정한다.
        gripper_samples = []
        gripper_fault_reason = None
        for gid in self.gripper_ids:
            if gid not in self.active_ids:
                fault = True
                continue
            sample = self._read_sample(gid)
            if sample is None:
                fault = True
                continue
            load_raw, tick, hw_error, velocity_raw = sample
            if hw_error != 0:
                fault = True
                gripper_fault_reason = (
                    f"ID {gid} hardware error 0x{hw_error:02X}")
            if abs(load_raw) >= self.end_effector_max_abs_current:
                fault = True
                gripper_fault_reason = (
                    f"ID {gid} abs(current) {abs(load_raw)} reached "
                    f"limit {self.end_effector_max_abs_current}")
            gripper_samples.append((load_raw, to_signed(tick, LEN_PRESENT_POSITION), velocity_raw))

        if (gripper_fault_reason
                and any(gid in self.torque_enabled_ids
                        for gid in self.gripper_ids)):
            with self._bus_lock:
                self._torque_off_gripper(gripper_fault_reason)

        if len(gripper_samples) == len(self.gripper_ids) and gripper_samples:
            representative_tick = gripper_samples[0][1]
            representative_velocity_raw = gripper_samples[0][2]
            max_abs_load = max(abs(sample[0]) for sample in gripper_samples)
            if self.end_effector_kind == "rotary":
                finger_rad = representative_tick / TICKS_PER_RAD
                finger_vel = representative_velocity_raw * VELOCITY_LSB_TO_RAD_S
            else:
                finger_rad = self.gripper_tick_to_pos(representative_tick)
                finger_vel = self.gripper_velocity_to_rad_s(
                    representative_velocity_raw)
            for jn in self.gripper_joints:
                msg.name.append(jn)
                msg.position.append(finger_rad)
                msg.velocity.append(finger_vel)
                msg.effort.append(float(max_abs_load))

        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """SyncRead 블록에서 (signed address-126 feedback, position, hw error, velocity) 추출.

        PRESENT_VELOCITY(128,4)는 SyncRead 범위(70~135) 안에 이미 포함돼 있어 별도 버스
        요청 없이 같은 블록에서 꺼낸다. 미수신 시 None.
        """
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            return None
        hw_error = self.group_sync_read.getData(
            dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)
        feedback_raw = to_signed(
            self.group_sync_read.getData(dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD),
            LEN_PRESENT_LOAD,
        )
        velocity_raw = to_signed(
            self.group_sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY),
            LEN_PRESENT_VELOCITY,
        )
        tick = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return feedback_raw, tick, hw_error, velocity_raw

    def destroy_node(self):
        for dxl_id in self.torque_enabled_ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )

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
