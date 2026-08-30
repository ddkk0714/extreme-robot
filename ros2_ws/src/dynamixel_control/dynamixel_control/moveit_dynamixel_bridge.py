#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32MultiArray
from control_msgs.action import FollowJointTrajectory
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead

from rcl_interfaces.msg import SetParametersResult

from dynamixel_control import bus_lock
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset
from dynamixel_control import calib_math
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

# tick 상수와 캘리브 측정식의 단일 출처는 calib_math 다(ROS 비의존 → pytest 로 고정).
# 여기서 재노출하는 이유는 `scripts/measure_*.py` 와 외부 코드가 예전부터 이 모듈에서
# 가져다 쓰고 있어서다 — import 경로를 깨지 않으면서 정의는 한 곳으로 모은다.
DXL_MINIMUM_POSITION_VALUE = calib_math.DXL_MINIMUM_POSITION_VALUE
DXL_MAXIMUM_POSITION_VALUE = calib_math.DXL_MAXIMUM_POSITION_VALUE
DXL_CENTER_POSITION = calib_math.DXL_CENTER_POSITION

TICKS_PER_RAD = calib_math.TICKS_PER_RAD
DXL_TICKS_PER_REV = calib_math.DXL_TICKS_PER_REV  # Present Velocity 환산용


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
#    `center`(영점)도 같은 방식으로 `centers` 파라미터가 덮어쓴다
#    (`scripts/measure_zero_offset.py` 또는 관제 GUI 의 영점 마법사 결과를 바로 시험).
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
# 🔁 **2026-08-19 3차 재측정 (현재 값).** 팔 전체 재조립 → 전 축 영점 무효.
#    같은 회차에 arm_joint_1(ID 11, XM540-W270)이 새로 실장된 것도 확인됐다.
#    ⚠️ extended 축(arm_joint_2/3)의 아래 center 는 **이 전원 세션 안에서만** 유효하다.
#
# 🔁 **2026-08-09 2차 재측정** (이력). 1차 측정 후 실기 구동 중 손목(arm_joint_5)
#    결합이 물리적으로 빠져 재조립했고, 서보 전원도 내려갔다 — 둘 다 영점 무효 사유다.
#    재조립 후 측정에서는 붙잡은 상태의 드리프트가 8초간 arm_joint_2/5 **정확히 0.000°**
#    로 나왔다.
#    ⚠️ 1차 측정 때 arm_joint_5 가 붙잡고 있는데도 +1.5°/s 로 미끄러졌던 것은 중력이
#       아니라 **결합이 이미 헐거웠다는 신호**였다(수리 후 그 드리프트가 완전히 사라진
#       것으로 확인). 어떤 축이 "잡아도 계속 흐르면" 측정을 계속하지 말고 결합부터
#       점검할 것 — 그대로 두면 구동 중 빠진다.
JOINT_CONFIG = {
    # 🆕 **2026-08-19 신설.** 이 축은 그동안 "모터가 물리적으로 없다"는 전제로
    # STATIC_JOINTS 에 0.0 고정 발행돼 있었는데, 재조립 후 버스 스캔에서 ID 11
    # (XM540-W270)이 정상 응답했다. 사용자 확인 결과 기구에도 물려 있다.
    # gear_ratio 1.0 = **감속기 없는 직결**이다(2026-08-19 사용자 확인) — 측정으로
    # 나온 값이 아니라 기구가 그렇다. center 는 같은 날 zero_offset 실측값.
    "arm_joint_1": {"id": 11, "center": 2081, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    # 2026-08-07 실측: 9.034:1 (관절 90° 회전 기준)
    "arm_joint_2": {"id": 14, "center": 506, "direction": -1,
                    "gear_ratio": 9.034, "extended": True},
    # 2026-08-07 실측: 4.040:1 — arm_joint_2 와 다른 감속기다(오타 아님)
    "arm_joint_3": {"id": 13, "center": 1855, "direction": 1,
                    "gear_ratio": 4.040, "extended": True},
    "arm_joint_4": {"id": 12, "center": 1184, "direction": 1,
                    "gear_ratio": 1.0, "extended": False},
    "arm_joint_5": {"id": 16, "center": 675, "direction": 1,
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
# 값의 근거:
#   arm_joint_2/3/4 는 축이 서로 평행해서 이 팔은 **평면 로봇**이고, 그 평면의 방위를
#   정하는 유일한 관절이 arm_joint_1 이다(축 0 0 1, 회전중심은 base_link 원점 위
#   z=0.0465). 즉 이 값이 틀리면 팔이 향하는 방향 전체가 틀린다.
#
# 🔁 **2026-08-12 정정: 1.405 → 0.0.**
#   이 값은 그동안 +1.405 rad 였다. 근거는 "0.0 이면 FK 가 그리퍼를 방위각 -80.5°
#   (거의 정오른쪽)에 놓는데, 실기의 팔은 정면(+x)을 향하므로 방위각을 0 으로 돌려야
#   한다" 는 것이었다 — 그 **전제가 틀렸다. 팔은 실제로 오른쪽으로 틀어져 있다**
#   (2026-08-12 사용자 확인).
#
#   확인 방법(같은 방식으로 재확인 가능): 카메라 TF 캘리브를 마친 상태에서 박스를
#   그리퍼 정면에 놓고 RViz 를 본다. joint_1=0 인 모델의 tip(link_043)은 방위각
#   -80.7°/반경 15.5cm 에 서고, 카메라가 본 박스는 -90.7°/46.1cm 에 찍혔다 —
#   **두 방향이 10° 안쪽으로 일치**했고 화면상으로도 박스가 그리퍼 앞이었다.
#   1.405 를 쓰면 이 관계가 통째로 80° 어긋난다.
#
#   ⚠️ 그 오차는 조용하다: 브릿지가 떠 있는 동안에만 TF 가 80° 돌아가므로, RViz 만
#      띄워 보면(jsp_gui 가 0 을 발행) 멀쩡해 보이고 arm_fsm 을 붙였을 때만 목표가
#      틀어진다. "인식·캘리브는 맞는데 팔이 엉뚱한 데로 간다" 면 여기를 볼 것.
#
# ⚠️ 이 축은 모터가 없다. **기구적으로 고정돼 있다는 전제**이며, 만약 자유회전
#    상태라면 팔의 평면이 운용 중 돌아가고 이 값은 무의미해진다 — 그 경우 IK 목표가
#    조용히 틀어지므로, 물리적으로 고정돼 있는지 반드시 확인할 것.
#    팔을 재장착했다면 위 확인 절차를 다시 밟을 것.
# 🔁 **2026-08-19 비움.** arm_joint_1 은 ID 11 서보가 실재하는 것이 확인돼
#    JOINT_CONFIG 로 옮겨졌다(위 참고). 이제 실측값이 /joint_states 로 나가므로
#    고정 발행이 필요 없다 — 고정값을 남겨두면 실제 서보각과 충돌한다.
#    아래 dict 와 발행 경로는 다음에 모터 없는 축이 생길 때를 위해 남겨둔다.
STATIC_JOINTS = {}

# X 시리즈 Extended Position Control Mode 의 raw tick 한계(약 ±256회전).
DXL_EXTENDED_MIN_TICK = calib_math.DXL_EXTENDED_MIN_TICK
DXL_EXTENDED_MAX_TICK = calib_math.DXL_EXTENDED_MAX_TICK


#: 캘리브 파라미터의 "비어 있음" 기본값.
#:
#: ⚠️ **`[]` 을 쓰면 안 된다.** rclpy 는 빈 리스트에서 타입을 추론하지 못해
#: `BYTE_ARRAY` 로 선언해 버리고, 그러면 런타임 `set_parameters` 가
#: *"Wrong parameter type, expected 'Type.BYTE_ARRAY' got 'Type.STRING_ARRAY'"* 로
#: **거절된다**(2026-08-12 실기 확인). CLI `-p gear_ratios:=` 는 선언 시점에 값을
#: 덮어써서 멀쩡히 동작하므로, **런타임에 처음 바꿔 볼 때까지 드러나지 않는다.**
#: `ParameterDescriptor(type=...)` 로도 추론을 못 바꾼다 — 빈 문자열 하나가 답이다.
#: 파서가 이름 없는 항목을 건너뛰므로 의미상으로는 "없음" 그대로다.
EMPTY_STR_ARRAY = [""]


# 캘리브 파라미터(`gear_ratios`·`centers`)의 파서. rclpy 에 dict 타입 파라미터가 없어
# "<joint>:<값>" 문자열 배열로 받는다. 기동 시와 런타임 변경(파라미터 콜백)이 **같은
# 검증**을 쓰도록 함수로 뺐다 — 한쪽만 느슨하면 기동은 되는데 변경은 거절되는 식이 된다.
def _parse_gear_ratios(entries):
    """`["arm_joint_2:9.034", …]` → `({이름: 비}, [오류 사유])`."""
    out, errors = {}, []
    for entry in entries or []:
        name, _, value = str(entry).partition(":")
        if not name:
            continue                       # 빈 문자열은 "없음" 으로 본다(기본값 [""])
        if name not in JOINT_CONFIG:
            errors.append(f"모르는 관절 '{name}'")
            continue
        try:
            ratio = float(value)
        except ValueError:
            errors.append(f"'{entry}' 파싱 실패")
            continue
        if ratio <= 0.0:
            errors.append(f"'{entry}' 은 양수여야 함")
            continue
        out[name] = ratio
    return out, errors


def _parse_centers(entries):
    """`["arm_joint_2:1627", …]` → `({이름: tick}, [오류 사유])`."""
    out, errors = {}, []
    for entry in entries or []:
        name, _, value = str(entry).partition(":")
        if not name:
            continue
        if name not in JOINT_CONFIG:
            errors.append(f"모르는 관절 '{name}'")
            continue
        try:
            center = int(round(float(value)))
        except ValueError:
            errors.append(f"'{entry}' 파싱 실패")
            continue
        reason = calib_math.center_out_of_range(center, JOINT_CONFIG[name]["extended"])
        if reason is not None:
            errors.append(f"{name}: {reason}")
            continue
        out[name] = center
    return out, errors


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
        # 0 이면 쓰지 않는다(서보 기본 885=100% 유지). preset 주석에 값 근거 있음.
        self.declare_parameter("gripper_goal_pwm", int(preset.get("gripper_goal_pwm", 0)))
        # 캘리브 범위 밖으로 미끄러진 그리퍼 자동 복구(_recover_gripper_range 참고).
        # 종료 시 토크가 풀리면 그리퍼가 닫힘 끝단을 지나쳐 미끄러지는데, 그 상태에서는
        # gripper_goal_pwm 의 힘으로 못 빠져나온다 — 재기동마다 재발하므로 기본 활성.
        # 모션 프로파일. 단위는 데이터시트 기준 Profile Velocity = 0.229 rev/min,
        # Profile Acceleration = 214.577 rev/min^2.
        # ⚠️ 팔 속도는 **여기서만** 정해진다(_write_motion_profile 주석 참고).
        #    2026-08-12: 40(절반)으로 낮췄다가 **80 으로 되돌렸다.** 느리게 하면
        #    arm_fsm 의 모션 완료 판정과 어긋난다 — `_publish_joint_trajectory` 가
        #    `arm_move_speed`(0.5 rad/s)로 duration 을 추정해 그만큼만 기다리는데,
        #    서보가 그보다 느려지면 **도착 전에 다음 상태로 넘어간다**(하강 도중
        #    파지 등). 속도를 정말 낮추려면 arm_fsm 의 `arm_move_speed` 를 같은
        #    비율로 낮춰 둘을 함께 맞춰야 한다.
        self.declare_parameter("arm_profile_velocity", 80)
        self.declare_parameter("gripper_profile_velocity", 80)
        self.declare_parameter("profile_acceleration", 25)
        self.declare_parameter("gripper_auto_recover", True)
        # ⚠️ 885(최대)로 두지 말 것. 2026-08-12 에 885 로 열림 끝단까지 밀어붙였다가
        # **랙이 피니언에서 미끄러진** 것으로 보인다(직후 재캘리브에서 오프셋이 통째로
        # ~1880 tick 이동). 실측상 500 이면 범위 밖에서 끌어내는 데 충분하다
        # (PWM 500 으로 -938 → -434 를 1초). 끝단을 때리지 않는 것이 더 중요하다.
        self.declare_parameter("gripper_recover_pwm", 500)
        self.declare_parameter("gripper_recover_timeout", 6.0)
        self.declare_parameter("read_only", False)
        self.declare_parameter("gripper_only_mode", False)

        self.gripper_joints = list(self.get_parameter("gripper_joints").value)
        self.gripper_ids = list(self.get_parameter("gripper_ids").value)
        self.gripper_open_rad = float(self.get_parameter("gripper_open_rad").value)
        self.gripper_close_rad = float(self.get_parameter("gripper_close_rad").value)
        self.gripper_open_tick = int(self.get_parameter("gripper_open_tick").value)
        self.gripper_close_tick = int(self.get_parameter("gripper_close_tick").value)
        self.gripper_extended = bool(self.get_parameter("gripper_extended").value)
        self.gripper_goal_pwm = int(self.get_parameter("gripper_goal_pwm").value)
        self.arm_profile_velocity = int(
            self.get_parameter("arm_profile_velocity").value)
        self.gripper_profile_velocity = int(
            self.get_parameter("gripper_profile_velocity").value)
        self.profile_acceleration = int(
            self.get_parameter("profile_acceleration").value)
        self.gripper_auto_recover = bool(
            self.get_parameter("gripper_auto_recover").value)
        self.gripper_recover_pwm = int(
            self.get_parameter("gripper_recover_pwm").value)
        self.gripper_recover_timeout = float(
            self.get_parameter("gripper_recover_timeout").value)
        self.read_only = bool(self.get_parameter("read_only").value)
        self.gripper_only_mode = bool(
            self.get_parameter("gripper_only_mode").value)

        # 기어비 실측 반영용 — JOINT_CONFIG 의 gear_ratio 기본값을 런타임에 덮어쓴다.
        # "<joint>:<ratio>" 문자열 배열로 받는다(예: ["arm_joint_2:9.8"]). rclpy 는
        # dict 타입 파라미터가 없어서 이 형태를 쓴다.
        self.declare_parameter("gear_ratios", EMPTY_STR_ARRAY)
        self.gear_ratios, errors = _parse_gear_ratios(
            self.get_parameter("gear_ratios").value)
        for reason in errors:
            self.get_logger().warn(f"gear_ratios: {reason} — 무시")
        for name, ratio in self.gear_ratios.items():
            self.get_logger().info(f"gear_ratio 덮어쓰기: {name} = {ratio}")

        # 영점(center tick) 실측 반영용 — gear_ratios 와 **완전히 대칭**이다.
        # 이게 없어서 `measure_zero_offset.py` 결과는 소스를 고쳐 재빌드해야만 반영됐다
        # (기어비·그리퍼 끝단은 파라미터로 바로 넣을 수 있는데 영점만 없었다).
        # ⚠️ 여기 값은 **관절각 도메인**의 center 다 — teleop_core 의 DEFAULT_CENTERS
        #    (서보축 도메인)와 숫자가 다르며 서로 복사하면 안 된다.
        self.declare_parameter("centers", EMPTY_STR_ARRAY)
        self.centers, errors = _parse_centers(self.get_parameter("centers").value)
        for reason in errors:
            self.get_logger().warn(f"centers: {reason} — 무시")
        for name, center in self.centers.items():
            self.get_logger().info(f"center 덮어쓰기: {name} = {center} tick")

        # ⚠️ 이 콜백이 없으면 `set_parameters` 는 **값만** 바꾸고 브릿지는 기동 시
        #    파싱해 둔 dict 를 계속 쓴다 — 호출자에게는 성공으로 보이는데 실제
        #    변환식은 그대로다. 캘리브 결과를 재빌드 없이 시험하려면 여기서 받아야 한다.
        self.add_on_set_parameters_callback(self._on_set_parameters)

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

        # ⚠️ 포트를 열기 **전에** 배타 잠금. 이 브릿지와 position_node 는 같은
        # /dev/ttyUSB0 을 잡으므로 "동시에 띄우지 말 것"이 계약인데, 지금까지는
        # 규율로만 지켜졌고 어기면 축 하나만 조용히 빠지는 형태로 망가졌다
        # (bus_lock 모듈 docstring 참고). fd 는 살려둬야 잠금이 유지된다.
        self._bus_lock_fd = bus_lock.acquire(DEVICENAME, self.get_logger())

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
            #
            # ⚠️ **응답하는 ID만 등록한다.** SyncRead 는 한 번의 브로드캐스트라, 등록해
            #    둔 ID 하나가 무응답이면 그 자리에서 응답 수신이 어긋나 **나머지 서보의
            #    데이터까지 못 쓰게 된다.** 2026-08-12 실기에서 ID 14 하나가 죽은 채로
            #    이 모드를 띄웠더니 /joint_states 에 팔 관절이 **하나도** 안 실렸고,
            #    로그에는 아무 경고도 없어서 "브릿지가 안 떴나?" 로 보였다.
            #    구동 경로(else 절)는 토크 인가 성공 여부로 이미 같은 필터를 갖고 있는데,
            #    정작 **캘리브에 쓰는 이 모드에만** 그게 없었다.
            if not self.gripper_only_mode:
                for joint_name, config in JOINT_CONFIG.items():
                    if not self._ping(config["id"], joint_name):
                        continue
                    if self.group_sync_read.addParam(config["id"]):
                        self.active_ids.add(config["id"])
            for gid in self.gripper_ids:
                if not self._ping(gid, f"gripper(id {gid})"):
                    continue
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
                if self._enable_torque(config["id"], joint_name, config["extended"]):
                    self.group_sync_read.addParam(config["id"])
                    self.active_ids.add(config["id"])
                    self.torque_enabled_ids.add(config["id"])

            # 그리퍼 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for gid in self.gripper_ids:
                if self._enable_torque(gid, f"gripper(id {gid})", self.gripper_extended,
                                       self.gripper_profile_velocity):
                    # Operating Mode 변경이 일부 RAM 값을 초기화하므로 모드·토크가 확정된
                    # **뒤에** 쓴다.
                    self._write_gripper_goal_pwm(gid)
                    self._check_gripper_in_calibrated_range(gid)
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
        # 토크 on/off 요청 — `position_node` 와 **같은 토픽·같은 포맷**을 쓴다
        # (`[enable, id...]`). 벤치 텔레옵 쪽에만 있던 인터페이스라 브릿지 경로에서는
        # 팔을 손으로 만지려면 스택을 통째로 내리는 수밖에 없었다. 어휘를 새로 만들지
        # 않고 기존 것을 그대로 받아, `teleop_core` 의 stop/freedrive 나 관제 GUI 버튼이
        # 어느 런타임에서든 같은 뜻을 갖게 한다.
        # 확장 한 가지: id 목록을 생략하면(`[enable]`) **등록된 전 축**에 적용한다 —
        # 요청자가 서보 ID 를 몰라도 되게 하기 위함이다(mission_console 이 이걸 쓴다).
        self.torque_request_sub = self.create_subscription(
            Int32MultiArray, "/dynamixel/torque_request",
            self.torque_request_callback, 10)

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
    def _write_motion_profile(self, dxl_id, label, velocity=None):
        """Profile Acceleration/Velocity 설정 — 토크 인가 **전에** 호출한다.

        기본값 0(=최고속 즉시 이동)이면 그리퍼가 움직일 때마다 순간 과전류로 토크가
        풀린다(HW-8 실기 검증, 재현율 100%). 팔 축도 같은 이유로 완만하게 둔다.

        ⚠️ **여기가 팔의 실제 속도를 정하는 유일한 곳이다.** `trajectory_callback` 은
        `time_from_start` 를 쓰지 않고 goal tick 만 SyncWrite 하므로, 궤적의 duration
        (arm_fsm 의 `arm_move_speed`)은 FSM 내부 타임아웃 추정에만 쓰이고 서보 속도에는
        영향이 없다. 속도를 바꾸려면 `arm_profile_velocity` 를 조정할 것.

        ⚠️ 그리퍼는 팔과 **따로** 둔다(`gripper_profile_velocity`). 그리퍼 속도를 낮추면
        완전 개폐 시간이 늘어나는데, `gripper_presets.gripper_action_time`(2.5s)은 그
        시간을 넘겨야 파지 effort 를 제대로 읽는다 — 같이 낮추면 "닫히는 도중에 판정"
        해서 grasp effort 가 0 으로 읽히는 알려진 실패로 돌아간다.
        """
        if velocity is None:
            velocity = self.arm_profile_velocity
        for addr, value, field in (
            (ADDR_PROFILE_ACCELERATION, self.profile_acceleration, "Profile Acceleration"),
            (ADDR_PROFILE_VELOCITY, velocity, "Profile Velocity"),
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

    def _warn_if_torque_off(self):
        """토크가 꺼진 채 모션 명령이 들어오면 크게 알린다.

        ⚠️ 2026-08-12 실기: 콘솔이 종료하며 토크를 풀어둔 상태에서 픽을 돌렸더니
        FSM 은 PERCEIVE→…→GRASP 전 구간을 정상 수행하고 브릿지도 goal 을 다 썼는데
        **서보가 전부 무시**해서 팔이 한 tick 도 안 움직였다. 어디에도 에러가 없어
        "프로그램은 도는데 안 움직인다" 로만 보인다 — 이 저장소가 반복해서 밟는
        조용한 실패다. 여기서 한 번은 말해준다.

        자동으로 토크를 켜지는 **않는다**. 사람이 팔을 만지려고 일부러 푼 것일 수
        있고, 그때 명령 하나에 팔이 다시 잠기면 손을 다친다.
        """
        off = sorted(self.active_ids - self.torque_enabled_ids)
        if not off:
            return
        self.get_logger().error(
            f"모션 명령을 받았지만 ID {off} 의 토크가 꺼져 있습니다 — 서보가 무시하므로 "
            "팔은 움직이지 않습니다(에러 없이 조용히). 켜려면 mission_console 의 "
            "'torque on' 또는 /dynamixel/torque_request 에 [1] 발행.")

    def torque_request_callback(self, msg):
        """`[enable, id...]` → 해당 ID 토크 on/off. id 생략 시 등록된 전 축.

        ⚠️ 끄면 팔이 중력으로 처진다. 그래서 "요청받았으니 끈다" 이상은 하지 않는다 —
        여기서 자세를 미리 접거나 하는 배려를 넣으면, 정작 급히 끊고 싶을 때 그 동작이
        먼저 나가버린다(안전 게이트에 부가 동작을 넣지 않는다는 이 저장소의 원칙).
        """
        data = list(msg.data)
        if not data:
            self.get_logger().error("torque_request: [enable, id...] 형식이어야 합니다")
            return
        if self.read_only:
            self.get_logger().warn("torque_request 무시 — read_only 모드는 레지스터를 쓰지 않습니다")
            return

        enable = 1 if data[0] else 0
        ids = data[1:] or sorted(self.active_ids)
        applied, failed = [], []
        for dxl_id in ids:
            if dxl_id not in self.active_ids:
                self.get_logger().warn(f"torque_request 무시 — 등록 안 된 ID {dxl_id}")
                continue
            if enable:
                # ⚠️ 토크를 켜기 **전에** Goal Position 을 현재 위치로 덮어쓴다.
                # 토크가 꺼진 동안 팔은 중력으로 처지는데 Goal 레지스터에는 마지막
                # 명령값이 그대로 남아 있다 — 그냥 켜면 서보가 그 옛 목표로 **튄다**
                # (teleop_core 의 resume 이 _sync_goal_to_measured 를 하는 것과 같은 이유).
                pos, res, err = self.packet_handler.read4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
                if res == 0 and err == 0:
                    self.packet_handler.write4ByteTxRx(
                        self.port_handler, dxl_id, ADDR_GOAL_POSITION, pos)
                else:
                    self.get_logger().error(
                        f"ID {dxl_id} 현재 위치를 못 읽어 goal 동기화를 건너뜁니다 — "
                        "토크 인가 시 팔이 옛 목표로 튈 수 있습니다")
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE if enable else TORQUE_DISABLE)
            if result != 0 or error != 0:
                failed.append(dxl_id)
                continue
            applied.append(dxl_id)
            if enable:
                self.torque_enabled_ids.add(dxl_id)
            else:
                self.torque_enabled_ids.discard(dxl_id)

        word = "인가" if enable else "해제"
        if applied:
            self.get_logger().warn(f"토크 {word}: ID {applied}")
        if failed:
            self.get_logger().error(f"토크 {word} 실패: ID {failed}")

    def _check_gripper_in_calibrated_range(self, dxl_id):
        """그리퍼가 캘리브 tick 범위 **밖**에 있으면 크게 경고한다.

        ⚠️ 2026-08-12 실기: 토크를 끄고 팔을 손으로 다루는 동안 그리퍼가 닫힘 끝단
        (-401)보다 786 tick 아래(-1187)까지 밀려 들어갔다. 그 영역에서는
        `gripper_goal_pwm`(280, 파지 토크 상한)의 힘으로 **되돌아 나올 수 없다** —
        실측으로 tick -890 부근에서 전류 316 을 뽑으며 스톨했고, 양방향 모두 막혔다.
        정상 범위 안에서는 같은 PWM 280 으로 전 구간을 2.5초에 여닫는다(실측).

        증상이 지독하다: 그리퍼가 "안 닫히고", `/joint_states` effort 는 스톨 전류
        316 을 계속 보고해 `grasp_effort_thresh`(250)를 넘으므로 FSM 은 **빈손인데
        파지 성공으로 판정**한다. 어느 로그에도 에러가 안 뜬다.

        복구는 Goal PWM 을 일시적으로 올려(500 이상) 범위 안으로 끌어낸 뒤 되돌리는
        것이다. 자동으로 하지 않는 이유는 그 상한이 Overload 트립을 막는 안전장치라,
        올릴지는 사람이 상황을 보고 정해야 하기 때문이다.
        """
        pos, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if result != 0 or error != 0:
            return
        tick = pos - (1 << 32) if pos >= (1 << 31) else pos
        lo = min(self.gripper_close_tick, self.gripper_open_tick)
        hi = max(self.gripper_close_tick, self.gripper_open_tick)
        margin = max(1, int(0.05 * (hi - lo)))
        if lo - margin <= tick <= hi + margin:
            return
        self.get_logger().error(
            f"그리퍼(id={dxl_id})가 캘리브 범위 밖입니다: tick={tick} "
            f"(정상 {lo}~{hi}). 이 상태에서는 Goal PWM {self.gripper_goal_pwm} 의 힘으로 "
            "빠져나오지 못해 '안 닫히는' 것처럼 보이고, 스톨 전류가 파지 임계를 넘어 "
            "**빈손인데 파지 성공으로 오판**합니다.")
        if self.gripper_auto_recover:
            self._recover_gripper_range(dxl_id, tick)
        else:
            self.get_logger().error(
                "gripper_auto_recover=false 이므로 자동 복구하지 않습니다 — Goal PWM 을 "
                "일시적으로 500 이상으로 올려 범위 안으로 되돌린 뒤 다시 시작하세요.")

    def _recover_gripper_range(self, dxl_id, tick):
        """캘리브 범위 밖으로 미끄러진 그리퍼를 열림 끝단으로 끌어낸다.

        ⚠️ 왜 매번 필요한가: `destroy_node()` 가 종료 시 전 ID 토크를 해제하는데,
        그리퍼는 힘을 잃으면 닫힘 방향으로 미끄러져 **끝단을 지나쳐 버린다**(2026-08-12
        실측: +1070 → -1259). 즉 스택을 재기동할 때마다 재발한다. 사람이 매번 손으로
        PWM 을 올려 빼내는 건 현실적이지 않아 자동화했다.

        복구는 파지 토크 상한(`gripper_goal_pwm`)을 **일시적으로** 올려서 한다 — 그
        상한은 물체를 문 채 무한정 미는 걸 막는 장치지, 빈 그리퍼를 옮기는 데 필요한
        힘까지 제한하려던 게 아니다. 실측으로 PWM 885 에서 1.5초 만에 끝나고 움직이는
        중 전류는 40~90(무부하 수준)까지 떨어진다. 끝나면 반드시 원래 값으로 되돌린다.
        """
        # 끝단(open_tick) 자체를 겨냥하지 않는다 — 거기는 기구적 스토퍼라 밀어붙이면
        # 랙이 미끄러진다(2026-08-12, 그때 오프셋이 통째로 ~1880 tick 이동했다).
        # 범위 안쪽 15% 지점이면 "밖에서 안으로" 라는 목적은 그대로 달성하면서
        # 스토퍼를 때리지 않는다.
        span = self.gripper_open_tick - self.gripper_close_tick
        target = int(self.gripper_open_tick - 0.15 * span)
        self.get_logger().warn(
            f"자동 복구 시도: Goal PWM {self.gripper_goal_pwm} → "
            f"{self.gripper_recover_pwm} 로 일시 상향, tick {tick} → {target} 로 이동")
        try:
            self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, ADDR_GOAL_PWM, self.gripper_recover_pwm)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, ADDR_GOAL_POSITION, target & 0xFFFFFFFF)
            deadline = time.time() + self.gripper_recover_timeout
            reached = False
            while time.time() < deadline:
                time.sleep(0.2)
                pos, result, error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
                if result != 0 or error != 0:
                    continue
                now = pos - (1 << 32) if pos >= (1 << 31) else pos
                if abs(now - target) <= 40:
                    reached = True
                    break
        finally:
            # 성공하든 실패하든 상한을 되돌린다 — 높은 PWM 을 켠 채 파지에 들어가면
            # Overload 트립 위험이 그대로 돌아온다.
            self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, ADDR_GOAL_PWM, self.gripper_goal_pwm)
        if reached:
            self.get_logger().info(
                f"자동 복구 성공 — 그리퍼가 범위 안({target})으로 복귀. "
                f"Goal PWM {self.gripper_goal_pwm} 복원됨.")
        else:
            self.get_logger().error(
                "자동 복구 실패 — 그리퍼가 여전히 범위 밖입니다. 기구적 걸림일 수 "
                "있으니 손으로 확인하세요(파지 판정을 신뢰하지 말 것).")

    def _ensure_operating_mode(self, dxl_id, label, extended):
        """Operating Mode 를 이 축이 요구하는 값으로 맞춘다 (토크 인가 **전에** 호출).

        ⚠️ 2026-08-09 실기: 그리퍼(id 3)가 **Velocity 모드(1)** 로 남아 있어 파지가 계속
           실패했다. Velocity 모드에서는 Goal Position(116) 이 **통째로 무시된다** — 브릿지가
           tick 을 써넣는 것도 성공하고(레지스터에 -401 이 그대로 들어가 있었다), 토크도
           켜져 있고, Hardware Error 도 0 인데, 서보는 Goal Velocity(=0) 만 따르므로
           **한 tick 도 움직이지 않는다.** 에러가 아무데도 안 나서 원인을 찾기 어렵다.

           증상: 그리퍼 position 이 세션 내내 고정값, effort 가 정확히 0.0(빈손이어도
           62~119 는 나와야 한다), FSM 은 `grasp effort 0.0 below threshold` 로 실패.

        그 전까지 이 브릿지는 모드를 **한 번도 쓰지 않고** 다른 도구(레거시
        `dynamixel_position_node`, `gripper_calibration` 등)가 남긴 값을 그대로 물려받았다.
        그래서 그 도구들을 돌린 뒤 모드가 바뀌어 있으면 조용히 깨졌다.
        """
        desired = MODE_EXTENDED_POSITION if extended else MODE_POSITION
        current, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE)
        if result != 0:
            self.get_logger().warn(
                f"Operating Mode 조회 실패: {label}, id={dxl_id}, result={result}")
            return False
        if current == desired:
            return True

        # 주소 11 은 EEPROM 이라 토크가 걸린 채로는 안 써진다 — 반드시 먼저 끈다.
        self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE, desired)
        readback, _, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, dxl_id, ADDR_OPERATING_MODE)
        if result != 0 or readback != desired:
            self.get_logger().error(
                f"Operating Mode 설정 실패: {label}, id={dxl_id}, "
                f"{current} → {desired} 시도했으나 현재 {readback} "
                f"(result={result}, error={error}) — 이 축은 명령이 무시된다")
            return False
        self.get_logger().warn(
            f"Operating Mode 교정: {label}, id={dxl_id}, {current} → {desired} "
            f"({'extended position' if extended else 'position'}). "
            "다른 도구가 모드를 바꿔놓은 상태였다.")
        return True

    def _ping(self, dxl_id, label):
        """버스에 실제로 응답하는 서보인지 확인. 없으면 사유를 남기고 False.

        읽기 전용/그리퍼 전용 모드가 SyncRead 등록 전에 쓴다. 구동 경로는 토크 인가
        성공 여부가 같은 역할을 한다(모터가 없으면 인가가 실패한다).
        """
        _model, result, error = self.packet_handler.ping(self.port_handler, dxl_id)
        if result != 0:
            self.get_logger().warn(
                f"서보 무응답 — {label} (id {dxl_id}): "
                f"{self.packet_handler.getTxRxResult(result)}. "
                "이 ID 는 읽기 대상에서 제외한다(등록해 두면 나머지 서보의 응답까지 "
                "못 쓰게 된다). 전원·케이블·ID 를 확인할 것.")
            return False
        if error != 0:
            self.get_logger().warn(
                f"{label} (id {dxl_id}) 응답에 에러 플래그: {error}")
        return True

    def _enable_torque(self, dxl_id, label, extended=False, velocity=None):
        # 모드가 틀리면 Goal Position 이 무시되므로 토크보다 먼저 맞춘다(EEPROM = 토크 OFF 필요).
        if not self._ensure_operating_mode(dxl_id, label, extended):
            return False
        # 토크 인가 전에 모션 프로파일부터 넣는다(급가속 트립 방지).
        self._write_motion_profile(dxl_id, label, velocity)
        # ⚠️ 그리고 Goal Position 을 **현재 위치로 덮어쓴다.** 서보의 Goal 레지스터에는
        # 지난 세션의 마지막 명령값이 그대로 남아 있는데, 그 사이 토크가 꺼져 팔이
        # 중력으로 처졌다면 토크를 켜는 순간 옛 목표로 **튄다**. mission_console 이
        # 종료할 때마다 토크를 풀어 팔을 늘어뜨리므로 이 경로는 기동마다 걸린다.
        # (torque_request_callback 의 enable 경로도 같은 이유로 같은 처리를 한다.)
        pos, res, err = self.packet_handler.read4ByteTxRx(
            self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
        if res == 0 and err == 0:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, ADDR_GOAL_POSITION, pos)
        else:
            self.get_logger().error(
                f"{label}(id={dxl_id}) 현재 위치를 못 읽어 goal 동기화를 건너뜁니다 — "
                "토크 인가 시 팔이 옛 목표로 튈 수 있습니다")
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

    def _joint_center(self, joint_name):
        """실측으로 덮어쓸 수 있는 영점(`centers` 파라미터 > JOINT_CONFIG 기본값)."""
        return self.centers.get(joint_name, JOINT_CONFIG[joint_name]["center"])

    def _on_set_parameters(self, params):
        """캘리브 값(영점·기어비·그리퍼 끝단)의 **런타임 반영**.

        캘리브 도구(`scripts/measure_*.py`, 관제 GUI 마법사)가 잰 값을 재빌드 없이
        그 자리에서 시험할 수 있어야 한다. 검증에 실패하면 이유와 함께 거절한다 —
        조용히 무시하면 "적용했는데 왜 그대로지?" 가 된다.

        ⚠️ 원자적 설정(`set_parameters_atomically`)으로 보내야 한다. 비원자 설정은
        파라미터를 하나씩 넘겨서, 그리퍼 개폐 tick 처럼 **짝으로만 의미가 있는 값**이
        중간 상태로 검증된다.

        ⚠️ 이 값이 바뀌면 rad↔tick 변환이 통째로 달라진다. 측정은 `read_only:=true`
        (토크 OFF)에서 하는 것이 전제이고, 토크가 살아 있는 상태에서 바꾸면 다음
        명령부터 팔이 다른 위치를 목표로 삼는다 — 그 경우 경고를 남긴다.
        """
        centers, ratios = None, None
        gripper = {}
        for param in params:
            if param.name == "centers":
                centers, errors = _parse_centers(param.value)
                if errors:
                    return SetParametersResult(
                        successful=False, reason=f"centers: {'; '.join(errors)}")
            elif param.name == "gear_ratios":
                ratios, errors = _parse_gear_ratios(param.value)
                if errors:
                    return SetParametersResult(
                        successful=False, reason=f"gear_ratios: {'; '.join(errors)}")
            elif param.name in ("gripper_open_tick", "gripper_close_tick"):
                gripper[param.name] = int(param.value)

        if gripper:
            open_tick = gripper.get("gripper_open_tick", self.gripper_open_tick)
            close_tick = gripper.get("gripper_close_tick", self.gripper_close_tick)
            if abs(open_tick - close_tick) < calib_math.MIN_GRIPPER_SPAN_TICK:
                return SetParametersResult(
                    successful=False,
                    reason=(f"그리퍼 개폐 tick 차이가 {abs(open_tick - close_tick)} "
                            "밖에 안 됩니다 — 잘못 측정된 값입니다"))

        changed = []
        if centers is not None:
            self.centers = centers
            changed.append(f"centers={centers}")
        if ratios is not None:
            self.gear_ratios = ratios
            changed.append(f"gear_ratios={ratios}")
        for name, value in gripper.items():
            setattr(self, name, value)
            changed.append(f"{name}={value}")

        if changed:
            self.get_logger().info("캘리브 런타임 반영: " + ", ".join(changed))
            if not self.read_only:
                self.get_logger().warn(
                    "⚠️ 토크가 살아 있는 상태에서 캘리브가 바뀌었다 — 다음 명령부터 "
                    "rad↔tick 변환이 달라진다. 측정은 read_only:=true 에서 할 것.")
        return SetParametersResult(successful=True)

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
        tick = int(round(calib_math.rad_to_tick(
            self._joint_center(joint_name), config["direction"],
            self._joint_gear_ratio(joint_name), rad)))
        if config["extended"]:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        """서보 tick → 관절 rad. rad_to_tick 의 역변환."""
        return calib_math.tick_to_rad(
            self._joint_center(joint_name), JOINT_CONFIG[joint_name]["direction"],
            self._joint_gear_ratio(joint_name), tick)

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
        self._warn_if_torque_off()

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

    # position_node 와 같은 이유로 traceback 대신 한 줄로 죽는다(그쪽 main 참고).
    try:
        node = MoveItDynamixelBridge()
    except bus_lock.BusInUseError as exc:
        print(f"[moveit_dynamixel_bridge] 기동 거부: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

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
