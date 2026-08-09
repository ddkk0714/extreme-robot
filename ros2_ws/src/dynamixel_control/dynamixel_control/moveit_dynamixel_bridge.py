#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from control_msgs.action import FollowJointTrajectory
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead

from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
from dynamixel_control import joint_limits


ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_POSITION = 116
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
# ⚠️ **직결(1:1) 축인 arm_joint_4/5 는 신뢰도가 한 단계 낮다.** 감속기 축(9:1, 4:1)은
#    역구동이 안 돼 손으로 세운 자세가 그대로 유지되지만, 직결 축은 토크를 끄면
#    중력으로 흘러내린다(arm_joint_4 는 손을 떼면 늘 같은 자리로 처지고, arm_joint_5 는
#    붙잡고 있어도 +1.5°/s 로 미끄러졌다). 2026-08-07 캘리브 대비 편차도 감속기 축은
#    1.7~2.0° 인데 이 두 축은 6.0°/7.5° 다. 파지가 손목 기울기 방향으로 어긋나면
#    기어비보다 먼저 여기를 의심할 것.
JOINT_CONFIG = {
    # 2026-08-07 실측: 9.034:1 (관절 90° 회전 기준)
    "arm_joint_2": {"id": 14, "center": 1448, "direction": -1,
                    "gear_ratio": 9.034, "extended": True},
    # 2026-08-07 실측: 4.040:1 — arm_joint_2 와 다른 감속기다(오타 아님)
    "arm_joint_3": {"id": 13, "center": 278, "direction": 1,
                    "gear_ratio": 4.040, "extended": True},
    "arm_joint_4": {"id": 12, "center": 2495, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    "arm_joint_5": {"id": 16, "center": 1034, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
}
ARM_IDS = {config["id"] for config in JOINT_CONFIG.values()}

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

        # --- 그리퍼 파라미터 (랙피니언 2모터 동일방향 구동, ID 3/4) ---
        # gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter("gripper_type", DEFAULT_GRIPPER)
        self.gripper_type = self.get_parameter("gripper_type").value
        preset = get_preset(self.gripper_type, self.get_logger())

        self.declare_parameter("gripper_joints", preset["gripper_joints"])
        self.declare_parameter("gripper_ids", preset["gripper_ids"])  # 빈 배열이면 그리퍼 비활성
        self.declare_parameter("gripper_open_rad", preset["gripper_open_rad"])
        self.declare_parameter("gripper_close_rad", preset["gripper_close_rad"])
        self.declare_parameter("gripper_open_tick", preset["gripper_open_tick"])
        self.declare_parameter("gripper_close_tick", preset["gripper_close_tick"])
        # 다회전 그리퍼 여부. preset 에 없으면 단일회전으로 본다(보수적 — 다회전을
        # 잘못 켜면 tick 이 wrap 없이 계속 나가 랙 끝단을 밀어붙인다).
        self.declare_parameter("gripper_extended", bool(preset.get("extended", False)))
        self.declare_parameter("read_only", False)
        self.declare_parameter("gripper_only_mode", False)

        self.gripper_joints = list(self.get_parameter("gripper_joints").value)
        self.gripper_ids = list(self.get_parameter("gripper_ids").value)
        self.gripper_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.gripper_open_tick = int(self.get_parameter("gripper_open_tick").value)
        self.gripper_close_tick = int(self.get_parameter("gripper_close_tick").value)
        self.gripper_extended = bool(self.get_parameter("gripper_extended").value)
        self.read_only = bool(self.get_parameter("read_only").value)
        self.gripper_only_mode = bool(
            self.get_parameter("gripper_only_mode").value)

        # 기어비 실측 반영용 — JOINT_CONFIG 의 gear_ratio 기본값을 런타임에 덮어쓴다.
        # "<joint>:<ratio>" 문자열 배열로 받는다(예: ["arm_joint_2:9.8"]). rclpy 는
        # dict 타입 파라미터가 없어서 이 형태를 쓴다.
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
            if ratio <= 0.0:
                self.get_logger().warn(f"gear_ratios: '{entry}' 은 양수여야 함 — 무시")
                continue
            self.gear_ratios[name] = ratio
            self.get_logger().info(f"gear_ratio 덮어쓰기: {name} = {ratio}")

        geared = {n: self._joint_gear_ratio(n) for n, c in JOINT_CONFIG.items()
                  if c["extended"]}
        if geared:
            self.get_logger().info(
                "감속기 축 기어비: "
                + ", ".join(f"{n}={r:.3f}:1" for n, r in geared.items())
                + " (2026-08-07 실측 — 파지 위치가 계통적으로 어긋나면 여기부터 의심)"
            )

        self.get_logger().info(
            "관절 안전 리밋: "
            + ", ".join(
                f"{n}=[{joint_limits.get_limits(n)[0]:+.3f},{joint_limits.get_limits(n)[1]:+.3f}]"
                for n in JOINT_CONFIG if joint_limits.get_limits(n) is not None
            )
        )
        unregistered = [n for n in JOINT_CONFIG if joint_limits.get_limits(n) is None]
        if unregistered:
            self.get_logger().warn(
                f"joint_limits 에 없는 축 {unregistered} — **리밋 없이 그대로 나간다.** "
                "joint_limits.py 에 추가할 것."
            )
        provisional = [n for n in joint_limits.provisional_joints() if n in JOINT_CONFIG]
        if provisional:
            self.get_logger().warn(
                f"관절 {provisional} 은 가동범위 실측이 없어 보수적으로 좁혀둔 상태다"
                f"(±{joint_limits.PROVISIONAL_HALF_RANGE} rad). 이 축이 거의 안 움직이면 "
                "리밋 탓이다 — scripts/measure_joint_limits.py 로 실측할 것."
            )

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
        else:
            # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for joint_name, config in JOINT_CONFIG.items():
                if self._enable_torque(config["id"], joint_name):
                    self.group_sync_read.addParam(config["id"])
                    self.active_ids.add(config["id"])
                    self.torque_enabled_ids.add(config["id"])

            # 그리퍼 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for gid in self.gripper_ids:
                if self._enable_torque(gid, f"gripper(id {gid})"):
                    self.group_sync_read.addParam(gid)
                    self.active_ids.add(gid)
                    self.torque_enabled_ids.add(gid)

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
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
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

    def _enable_torque(self, dxl_id, label):
        # 토크 인가 전에 모션 프로파일부터 넣는다(급가속 트립 방지).
        self._write_motion_profile(dxl_id, label)
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

    def _joint_gear_ratio(self, joint_name):
        """실측으로 덮어쓸 수 있는 기어비(`gear_ratios` 파라미터 > JOINT_CONFIG 기본값)."""
        return self.gear_ratios.get(joint_name, JOINT_CONFIG[joint_name]["gear_ratio"])

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
        span = self.gripper_open_tick - self.gripper_close_tick
        denom = self.gripper_open_rad - self.gripper_close_rad
        frac = 0.0 if denom == 0.0 else (rad - self.gripper_close_rad) / denom
        tick = int(round(self.gripper_close_tick + frac * span))
        # 다회전 그리퍼는 끝단 tick 이 0~4095 밖으로 나간다(2026-08-07 실측 close=-401)
        # — 단일회전으로 clamp 하면 완전 닫힘이 tick 0 에서 잘려 덜 닫힌 채 멈춘다.
        if self.gripper_extended:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

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

    def arm_goal_callback(self, goal_request):
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
        if self.gripper_only_mode:
            self.get_logger().error(
                f"Gripper-only mode: rejecting arm teleop command id={dxl_id}")
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
        if self.gripper_only_mode:
            self.get_logger().error(
                "Gripper-only mode: rejecting arm trajectory topic command")
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
    def execute_gripper(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        result = FollowJointTrajectory.Result()

        if not self.gripper_ids:
            self.get_logger().warn("Gripper goal received but gripper_ids is empty — ignored")
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        if trajectory.points:
            point = trajectory.points[-1]
            name_to_pos = dict(zip(trajectory.joint_names, point.positions))
            # 단일 구동 조인트(gripper_left_pinion_joint)만 사용 — 나머지 3개(우 피니언·좌우 랙)는
            # URDF <mimic> 으로 종속된다. 두 서보(id 3,4)에는 같은 goal_tick 을 보낸다.
            target_rad = None
            for jn in self.gripper_joints:
                if jn in name_to_pos:
                    target_rad = name_to_pos[jn]
                    break
            if target_rad is not None:
                self._write_gripper(target_rad)
            else:
                self.get_logger().warn(
                    f"Gripper goal has no known finger joint {self.gripper_joints}"
                )

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Gripper command sent to Dynamixel"
        return result

    def _write_gripper(self, rad):
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring gripper command")
            return
        goal_tick = self.gripper_pos_to_tick(rad)
        for gid in self.gripper_ids:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, gid, ADDR_GOAL_POSITION, goal_tick
            )
            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"Gripper write failed: id={gid}, result={result}, error={error}"
                )
        self.get_logger().info(
            f"gripper -> {rad:.4f} rad -> tick {goal_tick} "
            f"(ids {self.gripper_ids})"
        )

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
            gripper_samples.append((load_raw, to_signed(tick, LEN_PRESENT_POSITION), velocity_raw))

        if len(gripper_samples) == len(self.gripper_ids) and gripper_samples:
            representative_tick = gripper_samples[0][1]
            representative_velocity_raw = gripper_samples[0][2]
            max_abs_load = max(abs(sample[0]) for sample in gripper_samples)
            finger_rad = self.gripper_tick_to_pos(representative_tick)
            finger_vel = self.gripper_velocity_to_rad_s(representative_velocity_raw)
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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
