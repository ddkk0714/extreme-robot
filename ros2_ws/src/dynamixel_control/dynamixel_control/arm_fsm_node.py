"""로봇팔 미션 FSM 노드 (Phase 3, 구간2 구호물자 운반 중심).

설계 문서: `project_docs/PHASE3_FSM_설계.md` §4 상태표 / §5 핸드셰이크.
구현 방식(결정 '가', 2026-06-29): **MoveIt 단일 경로**.
  - 팔 모션: MoveIt `move_action`(MoveGroup)에 목표 pose 전송 → IK·경로계획은
    MoveIt이 수행 → MoveIt이 `arm_controller` FollowJointTrajectory로 실행 →
    upstream `moveit_dynamixel_bridge`가 실제 다이나믹셀 구동.
  - 그리퍼: `/gripper_controller/follow_joint_trajectory`(FollowJointTrajectory)로
    열림/닫힘 명령(그리퍼도 Dynamixel, 결정 B).
  - 전류 피드백: `/joint_states`의 effort 필드에서 그리퍼 관절 전류 → 파지/DROP 판정.

  ⚠️ 전제(브릿지 측 선행 작업, PHASE3_FSM_설계.md §6 → 결정 '가'로 이관):
    1) `moveit_dynamixel_bridge`가 `/joint_states`에 **effort(전류)** 를 채워야 함
       (현재는 position만 발행) — 안 그러면 파지/DROP 판정 불가.
    2) 그리퍼 관절 + `gripper_controller` 실행 경로를 브릿지/컨트롤러에 추가해야 함.
    3) 카메라 frame → planning frame(base_link) **TF**가 있어야 MoveIt이 목표를 변환.

2026-07-13 파워트레인 "계약 v2"(Notion `2026 국방로봇 자율주행 SW 전체 개발계획` §5.1/5.2)
반영 — status/mode 문자열과 트리거 조건을 아래처럼 갱신:
  - 작업 개시/하역 모두 **`/chassis_mode == MISSION_STOP` AND 같은 mission_id의
    `/arrival_status`** 를 순서 무관하게 둘 다 받아야 전이(`_try_advance`). 미인식
    mode·stale/미래/역행 stamp는 default-deny(상태 변경 없음).
  - `DONE`은 더 이상 완료 권위가 아님 — 픽업 완료는 `CARRYING_LOCKED`, 하역 완료는
    `STOWED_LOCKED`가 최종 권위. `RELEASE` 뒤 `STOWING` 경유해서 `STOWED_LOCKED` 도달.
  - `CARRY` 중 DROP 감지 시 기존 자동 재파지 루프(REGRASP↔PERCEIVE) 대신 `GRIP_LOST`로
    **완전 래치**(자동 재시도 없음) — 새 MISSION_STOP+ArrivalStatus(ARRIVED_PICKUP,
    같은 mission_id 재발행 가능) conjunction이 다시 와야만 PERCEIVE 재진입.
  - `STOW_REQUEST` mode 수신 시 진행 중인 작업을 강제로 RELEASE→STOWING 경로로 유도.
  - mission_id 멱등성: 이미 `STOWED_LOCKED`까지 완료한 mission_id의 ArrivalStatus
    재수신은 무시(중복 재실행 방지).

2026-07-14 자체 결정(우리 쪽에서 결정 가능한 항목은 회의 전에 확정·구현):
  - **`STOW_REQUEST` 범위 확장**: `GRIP_LOST` 전용이었던 것을 `STOW_ABORTABLE_STATES`
    (진행 중인 모든 작업 상태 + `CARRY` + `LOCKED`)로 확장.
  - **`WORK_READY` vs `STOWED_LOCKED` 역할 확정**: `WORK_READY`=MISSION_STOP+ArrivalStatus
    conjunction 수락 순간의 1회성 ack, `STOWED_LOCKED`=그 외 평상시(빈손) 상시 하트비트.
  - **`_is_settled()`**: locked 하트비트(`CARRYING_LOCKED`/`STOWED_LOCKED`) 발행 전 실제
    확인 — TF(`base_frame`←`tip_link`) tip 위치가 `locked_pos_tol` 이내로 안정되고,
    관절각 유한차분 속도가 `locked_vel_tol` 이내인 상태가 `locked_dwell`초 이상
    지속돼야 True. 브릿지가 `/joint_states`에 velocity를 안 실어도 위치 유한차분으로
    자체 계산하므로 문제 없음.

2026-07-15 — origin/main 재합류 + 실제 STOWING 모션 구현:
  - PR #17("파워트레인 DDS 통신 복구 + arm_status 10Hz heartbeat")이 이 파일을 이
    브랜치와 무관하게 독립적으로 다시 손대(계약 v2 상태/게이트 로직 없는 이전 버전 위에
    `contract.py`/`qos_profiles.py` 단일 출처 + heartbeat 전용 타이머·MultiThreadedExecutor
    를 추가) `main`에 먼저 병합됨. 이 세션에서 그 인프라(heartbeat 아키텍처·QoS·contract
    상수 단일 출처) 위에 위 계약 v2 FSM 로직(conjunction 게이트·GRIP_LOST 래치·
    STOW_ABORTABLE_STATES·`_is_settled()`)을 재적용.
  - **LOCK_MODES를 `contract.py` 것으로 통일**(기존엔 이 파일이 로컬로 `DRIVING`을 제외한
    부분집합을 따로 들고 있었음) — `contract.py`(파워트레인 contract.py와 짝, 단일 출처)는
    `DRIVING`도 LOCK_MODES에 포함한다. 즉 PERCEIVE~LIFT 중 `DRIVING` 수신 시에도 이제
    `_enter_locked()`가 걸린다("MISSION_STOP만 허가, 나머지 전부 잠금"을 문자 그대로 적용).
    LOCKED 상태에서 `DRIVING`으로 자동 언락되는 옛 버그(PR #17이 미수정으로 지적)는
    애초에 이 파일에 그런 분기가 없으므로 해당 없음 — `_try_advance()`의
    MISSION_STOP+ArrivalStatus conjunction으로만 탈출.
  - **`STOWING` 실제 접이 모션 구현**(`_begin_stow_move`) — 이전까지는 스켈레톤이라 현재
    자세 그대로 `_is_settled()`만 확인했음(모션 자체가 없어 접힘 자세 검증이 아니라
    "멈춰있나" 검증에 불과했음). 이제 `stow_joint_positions` 파라미터가 정의하는 목표
    관절각으로 `/arm_controller/joint_trajectory`에 직접 궤적을 발행 → 완료 후 `_is_settled()`
    게이트를 거쳐 `STOWED_LOCKED`.
    ⚠️ **`stow_joint_positions` 기본값은 CAD 미검증 placeholder다.** 계약상 all-zero
    home을 접힘 자세로 쓰는 것은 금지(PR #17 회신) — 그래서 0이 아닌 임의값을 넣어뒀지만
    실제 팔이 안전하게 접히는 각도인지는 실기 검증 전까지 모른다. **실기 테스트 없이
    이 기본값으로 실제 서보를 구동하지 말 것.**

2026-07-15 — 파워트레인 §5.1 잔여 합의 2건 해결(`project_docs/파워트레인_계약_충돌점검.md`
항목 1·2 대응):
  - **`_near_stow_posture()` 추가** — `LOCKED`(지형/주행 이벤트로 작업 중단) 경유로 도달한
    임의 자세를 예전엔 정지만 확인되면 바로 `STOWED_LOCKED`로 근사했음. 이제 관절각이
    `stow_joint_positions` 근처(`stow_pos_tol_rad`)인지 확인한 경우에만 `STOWED_LOCKED`를
    발행하고, 아니면 `EXECUTING`을 유지해 파워트레인 쪽 motion hold를 받는다(거짓 주행
    허가 방지).
  - **`LOWER_RELEASE` 상태 신설** — `PAYLOAD_ALOFT_STATES`(`LIFT`/`CARRY`, 화물을 든 채
    공중일 수 있는 상태)에서 `STOW_REQUEST`로 중단되면, 예전엔 바로 `RELEASE`(그리퍼
    오픈)로 갔는데 이는 화물이 공중에서 그대로 낙하하는 경로였음. 이제 `RELEASE` 전에
    `lift_height`만큼 먼저 내려(`_lower_pose`/`_lower_target_xyz`, `_carry_pose`/
    `_lift_target_xyz`의 역) grasp 당시 높이 근처로 되돌아간 뒤에만 그리퍼를 연다.
  - **controller fault 게이트 추가** — `moveit_dynamixel_bridge`가 Hardware Error
    Status(주소 70)를 기존 current/position SyncRead 블록에 합쳐 읽어
    `/dynamixel/controller_fault`(Bool, 내부용 — DDS 경계 안 넘음)로 발행하도록
    확장. `_is_settled()`가 이 값이 True(등록된 서보 중 하나라도 에러 또는 무응답)
    이면 즉시 미확인 처리 — 이전엔 이 필드 자체가 없어 검사 불가였던 항목(§5.1
    잔여 항목 3).

상태 흐름: IDLE → PERCEIVE → PLAN → DESCEND → GRASP_CHECK → LIFT → CARRY
  → (ARRIVED_DROP) RELEASE → STOWING → STOWED_LOCKED → IDLE
  CARRY 중 DROP 감지 → GRIP_LOST(래치) → (재발행 conjunction) PERCEIVE
  PERCEIVE~LIFT 중 지형/주행 이벤트 → LOCKED(래치, 하트비트 유지) → (재발행 conjunction) PERCEIVE
  LIFT/CARRY(또는 LIFT 중 LOCKED) 중 STOW_REQUEST → LOWER_RELEASE → RELEASE → STOWING
  그 외 상태 중 STOW_REQUEST → RELEASE → STOWING (이미 grasp 높이 근처라 낙하 낙차 없음)
  LOCKED/GRIP_LOST 모두: 진행 중 모션 취소 + 현재 자세 홀드, MISSION_STOP conjunction으로만 탈출.

⚠️ 스켈레톤: MoveGroup pose goal / 그리퍼 액션 / effort 판정 / FSM 골격은 구현.
   LIFT·CARRY 목표 pose, 임계값 캘리브, TF 연결은 구현됨. STOWING 모션은 위 참고
   (목표 관절각 실측 필요). CARRYING_LOCKED/STOWED_LOCKED 발행 전 controller fault
   확인은 `moveit_dynamixel_bridge`의 `/dynamixel/controller_fault`(Hardware Error
   Status 집계)를 `_is_settled()`에서 게이트로 사용해 구현 완료(2026-07-15).
"""
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume, RobotState)
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from robot_arm_msgs.msg import ArrivalStatus, ChassisMode, ArmStatus, DetectedObject
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset


# 2026-07-15 Isaac Sim 기반 재export(robotarm_urdf_20260711.urdf) 기준 — URDF 자체는
# 팔 5축(arm_joint_1~5)을 전부 반영하지만, analytic IK(FK+수치 자코비안)는 아직 앞의
# 3관절만 풀도록 남겨둠(HW-7 당시 6DOF pose goal이 NO_IK_SOLUTION이던 문제 회피용으로
# 도입된 3DOF 위치전용 IK — URDF가 3축만 있어서가 아니라 solver를 아직 5DOF로 확장 안
# 해서임, 방향은 여전히 무시). MoveGroup 경로(§6 결정 '가')는 남겨두되 ik_mode:='moveit'로
# 전환 가능하게만 유지.
ARM_JOINT_NAMES = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']


# ──────────────────────────────────────────────
# status / mode 문자열 — 단일 출처는 contract.py (파워트레인 contract.py 와 짝).
# 여기서 상수를 새로 정의하지 말 것. 어휘 변경은 양 팀 합의 사항이다.
# ──────────────────────────────────────────────
from dynamixel_control.contract import (       # noqa: E402
    ARRIVED_PICKUP, ARRIVED_DROP,
    ARM_PERCEIVING, ARM_PLANNING, ARM_EXECUTING, ARM_FAILED,
    ARM_WORK_READY, ARM_STOWING, ARM_STOWED_LOCKED, ARM_CARRYING_LOCKED, ARM_GRIP_LOST,
    LOCK_MODES, MODE_MISSION_STOP, MODE_STOW_REQUEST, HEARTBEAT_RATE_HZ,
)
from dynamixel_control.qos_profiles import HEARTBEAT_QOS, ARRIVAL_QOS   # noqa: E402

# contract.py의 LOCK_MODES는 DRIVING을 포함한다("MISSION_STOP만 허가, 나머지 전부 잠금").
RECOGNIZED_MODES = LOCK_MODES | {MODE_MISSION_STOP, MODE_STOW_REQUEST}

# stamp freshness — 미래/역행 판정 허용오차 [s] (계약 §5.1 age 0~0.5s 기준)
STAMP_FUTURE_TOL = 0.5

# moveit_msgs/MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1


class State(Enum):
    IDLE = auto()
    PERCEIVE = auto()
    PLAN = auto()
    DESCEND = auto()
    GRASP_CHECK = auto()
    LIFT = auto()
    CARRY = auto()
    GRIP_LOST = auto()
    LOWER_RELEASE = auto()
    RELEASE = auto()
    STOWING = auto()
    STOWED_LOCKED = auto()
    LOCKED = auto()


# 지형/주행 이벤트(LOCK_MODES)로 preempt 대상이 되는 상태 — 실제 모션/그리퍼 동작이 진행
# 중일 수 있는 상태만. CARRY는 이미 정지-유지 상태(그리퍼 effort 감시만)라 preempt
# 불필요 — 계약 v2 하트비트를 CARRY 자체 루프가 계속 발행해야 하므로 굳이 LOCKED로 빼지 않음.
PREEMPTIBLE_STATES = (State.PERCEIVE, State.PLAN, State.DESCEND, State.GRASP_CHECK, State.LIFT)

# STOW_REQUEST(운영자 포기·재정렬 유도)로 즉시 RELEASE(or LOWER_RELEASE)→STOWING 강제
# 진입 가능한 상태 — 작업이 진행 중이거나 래치된 모든 상태(2026-07-14 결정: GRIP_LOST
# 전용이었던 것을 확장). IDLE/LOWER_RELEASE/RELEASE/STOWING/STOWED_LOCKED는 이미
# 정지/포기 진행 중이라 대상에서 제외.
STOW_ABORTABLE_STATES = PREEMPTIBLE_STATES + (State.CARRY, State.GRIP_LOST, State.LOCKED)

# STOW_REQUEST로 중단될 때 화물을 든 채 공중에 있을 수 있는 상태 — 그리퍼를 바로 열면
# 낙하 위험(파워트레인 §5.1 잔여 합의 ①). 이 상태들에서만 RELEASE 전에 파지 높이까지
# 먼저 내리는 LOWER_RELEASE를 경유한다. 그 외(PERCEIVE/PLAN/DESCEND/GRASP_CHECK)는 이미
# grasp 높이 근처라 낙하 낙차가 없어 바로 RELEASE해도 안전.
PAYLOAD_ALOFT_STATES = (State.LIFT, State.CARRY)


class ArmFsmNode(Node):
    def __init__(self):
        super().__init__('arm_fsm_node')

        # ── 파라미터 ──────────────────────────────
        # MoveIt
        self.declare_parameter('planning_group', 'arm')          # SRDF group
        # tip_link: arm_joint_5 이후 고정 조인트 체인의 마지막 링크(link_051) — 여기서
        # gripper_drive_joint(구동)와 gripper_linkage_base_fixed(그리퍼 고정 베이스)가 갈라짐.
        # 2026-07-15 Isaac Sim 재export(robotarm_urdf_20260711.urdf) 기준.
        self.declare_parameter('tip_link', 'link_051')            # 그리퍼 부모 링크
        self.declare_parameter('base_frame', 'base_link')        # planning frame (리프트 기준)
        self.declare_parameter('lift_height', 0.10)              # LIFT 시 base_link +Z [m]
        self.declare_parameter('pick_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pos_tolerance', 0.01)            # [m]
        self.declare_parameter('orient_tolerance', 0.1)          # [rad]
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('vel_scale', 0.1)                 # 저속(파지 안전)
        self.declare_parameter('acc_scale', 0.1)
        # 'analytic'(기본, URDF 3관절 한정 수치 IK) | 'moveit'(URDF 5축 완성 후 전환)
        self.declare_parameter('ik_mode', 'analytic')
        self.declare_parameter('ik_max_iters', 8)
        self.declare_parameter('ik_tol', 0.01)          # [m] 위치 수렴 허용오차
        self.declare_parameter('ik_accept_tol', 0.03)   # [m] 최종 실패 판정 기준
        self.declare_parameter('arm_move_speed', 0.5)   # [rad/s] 직접명령 시 소요시간 추정용
        # 그리퍼 — gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter('gripper_type', DEFAULT_GRIPPER)
        gripper_type = self.get_parameter('gripper_type').value
        gpreset = get_preset(gripper_type, self.get_logger())

        self.declare_parameter('gripper_joints', gpreset['gripper_joints'])
        self.declare_parameter('gripper_open', gpreset['gripper_open_m'])
        self.declare_parameter('gripper_close', gpreset['gripper_close_m'])
        # 전류(effort) 임계 — moveit_dynamixel_bridge 가 /joint_states.effort 에
        # raw signed PRESENT_CURRENT(XL430 기준 1단위≈2.69mA)를 발행. preset 값은 placeholder,
        # 실측 캘리브 필요(TODO): 무부하 파지 전류/낙하 시 전류를 측정해 임계값 설정.
        self.declare_parameter('grasp_effort_thresh', gpreset['grasp_effort_thresh'])
        self.declare_parameter('drop_effort_thresh', gpreset['drop_effort_thresh'])
        # 동작 제어
        # max_regrasp(origin/main의 옛 자동 재파지 루프 파라미터)는 도입 안 함 — 계약 v2는
        # CARRY 중 DROP 감지 시 REGRASP↔PERCEIVE 루프 대신 GRIP_LOST 완전 래치로 대체함
        # (자동 재시도 없음, 새 MISSION_STOP+ArrivalStatus conjunction으로만 재개).
        self.declare_parameter('gripper_action_time', gpreset['gripper_action_time'])  # [s]
        self.declare_parameter('tick_rate', 10.0)
        # chassis_mode 수신 끊김 워치독 — §5.1 "수신 끊김 = default-deny(잠금 유지)"
        self.declare_parameter('chassis_mode_timeout', 1.0)      # [s]
        # locked 하트비트(CARRYING_LOCKED/STOWED_LOCKED) 발행 전 실제 확인 조건 — §5.1
        # "문자열만 바꾸는 게 아니라 자세 오차·관절 속도·유지 시간을 확인한 뒤 발행".
        # 수치는 실측 캘리브 전 placeholder(TODO). 컨트롤러 fault 확인은 브릿지에 아직
        # 해당 필드가 없어 미포함(별도 후속 작업).
        self.declare_parameter('locked_pos_tol', 0.005)   # [m] tip 위치 흔들림 허용치
        self.declare_parameter('locked_vel_tol', 0.05)    # [rad/s] 관절 속도(유한차분) 허용치
        self.declare_parameter('locked_dwell', 0.5)       # [s] 안정 유지 시간
        # STOWING 목표 관절각(ARM_JOINT_NAMES 순서) — ⚠️ CAD 미검증 placeholder. all-zero
        # home은 계약상 접힘 자세로 금지(구조상 충돌·역구동 위험) — 실기 검증 전까지 이
        # 기본값으로 실제 서보를 구동하지 말 것. 실측 후 이 파라미터로 덮어쓸 것.
        self.declare_parameter('stow_joint_positions', [0.0, -0.6, 1.2])
        # STOWED_LOCKED 발행 전 "실제로 접힌 자세인지" 확인용 관절각 허용오차 — §5.1 잔여
        # 합의 ②(정지 안정성만 검사하고 접힘 자세 근접은 미확인) 대응. LOCKED 경유(지형/주행
        # 이벤트로 작업 중단)로 도달한 임의 자세를 STOWED_LOCKED로 착칭하지 않기 위함.
        self.declare_parameter('stow_pos_tol_rad', 0.1)   # [rad] ≈5.7도, placeholder

        g = self.get_parameter
        self.planning_group = g('planning_group').value
        self.tip_link = g('tip_link').value
        self.base_frame = g('base_frame').value
        self.lift_height = g('lift_height').value
        self.pick_frame_id = g('pick_frame_id').value
        self.pos_tol = g('pos_tolerance').value
        self.orient_tol = g('orient_tolerance').value
        self.planning_time = g('planning_time').value
        self.vel_scale = g('vel_scale').value
        self.acc_scale = g('acc_scale').value
        self.ik_mode = g('ik_mode').value
        self.ik_max_iters = int(g('ik_max_iters').value)
        self.ik_tol = g('ik_tol').value
        self.ik_accept_tol = g('ik_accept_tol').value
        self.arm_move_speed = g('arm_move_speed').value
        self.gripper_type = gripper_type
        self.gripper_joints = list(g('gripper_joints').value)
        self.gripper_open = g('gripper_open').value
        self.gripper_close = g('gripper_close').value
        self.grasp_thresh = g('grasp_effort_thresh').value
        self.drop_thresh = g('drop_effort_thresh').value
        self.gripper_action_time = g('gripper_action_time').value
        self.chassis_mode_timeout = g('chassis_mode_timeout').value
        self.locked_pos_tol = g('locked_pos_tol').value
        self.locked_vel_tol = g('locked_vel_tol').value
        self.locked_dwell = g('locked_dwell').value
        self.stow_pos_tol_rad = g('stow_pos_tol_rad').value
        self.stow_joint_positions = list(g('stow_joint_positions').value)
        if len(self.stow_joint_positions) != len(ARM_JOINT_NAMES):
            self.get_logger().error(
                f'stow_joint_positions 길이({len(self.stow_joint_positions)})가 '
                f'ARM_JOINT_NAMES({len(ARM_JOINT_NAMES)})와 다름 — STOWING 모션 비활성')
            self.stow_joint_positions = None
        else:
            self.get_logger().warn(
                'stow_joint_positions는 CAD 미검증 placeholder다 — 실기 검증 없이 '
                '이 기본값으로 실제 서보를 구동하지 말 것.')

        # ── 토픽/액션 I/O ─────────────────────────
        # QoS 는 계약(contract.py/qos_profiles.py) 기준. heartbeat 계열을 depth 10 으로
        # 두면 낡은 샘플이 큐에 쌓여 파워트레인의 age(신선도) 판정이 어긋난다.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DetectedObject, '/pick_target', self._on_pick_target, latched)
        self.create_subscription(ArrivalStatus, '/arrival_status', self._on_arrival, ARRIVAL_QOS)
        self.create_subscription(ChassisMode, '/chassis_mode', self._on_chassis_mode, HEARTBEAT_QOS)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        # 계약 §5.1 "locked heartbeat는 ... controller fault 0 ... 을 실제 확인한다" —
        # moveit_dynamixel_bridge가 Hardware Error Status를 집계해 발행(내부용 토픽,
        # 파워트레인 DDS 경계를 넘지 않음). _is_settled()에서 게이트로 사용.
        self.create_subscription(Bool, '/dynamixel/controller_fault',
                                  self._on_controller_fault, 10)

        self.pub_status = self.create_publisher(ArmStatus, '/arm_status', HEARTBEAT_QOS)

        # TF: base_link ← tip_link 조회용 (LIFT 시 현재 TCP 기준 수직 리프트 계산)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._move = ActionClient(self, MoveGroup, 'move_action')          # MoveIt (ik_mode=='moveit')
        self._grip = ActionClient(self, FollowJointTrajectory,
                                  '/gripper_controller/follow_joint_trajectory')

        # analytic IK 경로 (ik_mode=='analytic', 기본): FK 서비스 + 직접 관절궤적 publish
        # ⚠️ FK 호출은 _tick(타이머 콜백) 안에서 블로킹 대기함 — self 를 spin하면 이미
        # 실행 중인 콜백을 재진입 spin 하게 되어 응답을 못 받고 타임아웃(실측 확인:
        # 독립 스크립트로는 2회 반복만에 수렴하는데 노드 내부에서는 즉시 실패).
        # 별도 헬퍼 노드/이그제큐터로 분리해서 우회.
        self._fk_node = rclpy.create_node('arm_fsm_fk_client')
        self._fk_client = self._fk_node.create_client(GetPositionFK, '/compute_fk')
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._joint_position = {}          # joint_name -> position(rad), /joint_states 에서 갱신
        self._arm_move_deadline = None      # analytic 이동 완료 예상 시각

        # ── 내부 상태 ─────────────────────────────
        self.state = State.IDLE
        self._state_enter_t = self.get_clock().now()
        self._prev_state = None
        self.locked = False
        self.pick_target = None
        self.mission_id = 0
        self._joint_effort = {}            # joint_name -> effort
        # 브릿지 controller fault 게이트 — 첫 샘플 받기 전엔 알 수 없으니 보수적으로 True
        # (TF 미가용 시 _is_settled()가 False를 리턴하는 것과 같은 안전 측 기본값).
        self._controller_fault = True
        # 계약 v2 — MISSION_STOP + ArrivalStatus conjunction 게이트 (순서 무관)
        self._mission_stop_active = False
        self._pending_arrival = None        # 아직 소비 안 한 최신 ArrivalStatus
        self._last_arrival_stamp = None
        self._last_chassis_stamp = None
        self._last_chassis_recv_wall = None     # chassis_mode 마지막 수신 시각(워치독용)
        self._last_completed_mission_id = None  # STOWED_LOCKED 도달한 mission_id (중복 재실행 방지)
        # locked-heartbeat 실측 확인용 (_is_settled)
        self._settle_start = None          # 안정 유지 시작 시각(불안정 감지 시 리셋)
        self._last_settle_pos = None       # 직전 tick의 tip 위치(np.array)
        self._last_settle_time = None
        self._last_settle_joints = {}      # 직전 tick의 관절각 스냅샷(유한차분 속도용)
        # 팔 모션(MoveIt/직접궤적) 진행 추적
        self._motion_state = 'idle'        # 'idle' | 'active' | 'done'
        self._motion_ok = False
        self._arm_goal_handle = None
        self._grip_sent = False            # 상태 진입 시 _transition에서 리셋
        self._stow_move_sent = False       # 상태 진입 시 _transition에서 리셋

        # ── heartbeat ─────────────────────────────
        # 계약: 현재 상태를 10Hz 로 끊임없이 발행한다. 0.5초 넘게 끊기면 파워트레인이
        # arm_status_stale 로 차를 세운다.
        #
        # 발행 경로는 **반드시 이 타이머 하나뿐**이어야 한다. 상태 핸들러가 각자
        # publish 하면 stamp 가 뒤섞여 나갈 수 있는데, 파워트레인은 stamp 가 0.5초 이상
        # 역행하면 **영구 latch**(프로세스 재시작 전까지 해제 불가)를 건다.
        # 그래서 핸들러는 _set_status() 로 값만 바꾸고, 실제 발행은 여기서만 한다.
        #
        # 별도 콜백그룹인 이유: _tick 은 analytic IK 의 FK 호출에서 블로킹 대기한다.
        # 같은 그룹이면 IK 도는 동안 heartbeat 가 굶어 stale 판정을 맞는다.
        # → main() 이 MultiThreadedExecutor 로 띄운다.
        self._status = ARM_STOWED_LOCKED
        self._hb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / HEARTBEAT_RATE_HZ, self._publish_heartbeat,
                          callback_group=self._hb_group)

        period = 1.0 / g('tick_rate').value
        self._tick_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(period, self._tick, callback_group=self._tick_group)
        self.get_logger().info(
            f'arm_fsm_node started (MoveIt 경로, state=IDLE, gripper_type={self.gripper_type}, '
            f'heartbeat={HEARTBEAT_RATE_HZ}Hz)'
        )

    # ── 콜백 ───────────────────────────────────

    def _on_pick_target(self, msg):
        self.pick_target = msg

    def _on_arrival(self, msg):
        if not self._stamp_is_fresh(msg.header.stamp, self._last_arrival_stamp):
            self.get_logger().warn('ArrivalStatus stamp 무효(0/미래/역행) — 무시')
            return
        self._last_arrival_stamp = msg.header.stamp
        self._pending_arrival = msg
        self._try_advance()

    def _on_chassis_mode(self, msg):
        if not self._stamp_is_fresh(msg.header.stamp, self._last_chassis_stamp):
            return
        self._last_chassis_stamp = msg.header.stamp
        self._last_chassis_recv_wall = self.get_clock().now()

        if msg.mode not in RECOGNIZED_MODES:
            # 미인식 mode — default-deny, 상태 변경 없음(락 유지)
            self.get_logger().warn(f'미인식 chassis_mode={msg.mode!r} — 무시')
            return

        # 계약 v2: MISSION_STOP만이 유일한 언락·작업 허가. DRIVING 포함 나머지는 전부
        # 잠금 유지 — LOCKED 탈출은 _try_advance()의 MISSION_STOP+ArrivalStatus
        # conjunction으로만 가능(자동 언락 분기 없음).
        self._mission_stop_active = (msg.mode == MODE_MISSION_STOP)

        if msg.mode in LOCK_MODES:      # contract.py 기준 DRIVING 포함
            self._enter_locked()
        elif msg.mode == MODE_STOW_REQUEST and self.state in STOW_ABORTABLE_STATES:
            # 운영자 포기/재정렬 유도 — 진행 중인 작업(또는 GRIP_LOST/LOCKED 래치) 중단하고
            # 접어 잠금. 2026-07-14: GRIP_LOST 전용이었던 범위를 작업 중 모든 상태로 확장.
            # 화물을 든 채 공중일 수 있으면(PAYLOAD_ALOFT_STATES, 또는 LIFT 중 LOCKED로
            # 중단된 경우) 그리퍼를 바로 열지 않고 LOWER_RELEASE로 먼저 내린다 — §5.1
            # 잔여 합의 ①(무조건 RELEASE 전이 = 화물 낙하 경로) 대응.
            aloft = (self.state in PAYLOAD_ALOFT_STATES
                     or (self.state == State.LOCKED and self._prev_state == State.LIFT))
            self._cancel_arm_motion()
            self.locked = False
            self._transition(State.LOWER_RELEASE if aloft else State.RELEASE)

        self._try_advance()

    def _enter_locked(self):
        """PREEMPTIBLE_STATES 중이면 모션 취소 후 LOCKED 진입. 이미 락이면 no-op."""
        self.locked = True
        if self.state in PREEMPTIBLE_STATES:
            self._prev_state = self.state
            self._cancel_arm_motion()
            self._transition(State.LOCKED)

    def _try_advance(self):
        """MISSION_STOP + ArrivalStatus conjunction(순서 무관) 충족 시에만 전이.

        픽업 개시: IDLE/GRIP_LOST 에서 ARRIVED_PICKUP 수신 시 PERCEIVE.
        지형 중단 복귀: LOCKED(같은 mission_id) 에서도 동일 conjunction으로 PERCEIVE 재진입
        (중단 시점 재개 대신 PERCEIVE부터 다시 — 중단 중 타겟이 변했을 수 있어 더 안전).
        하역: CARRY 에서 ARRIVED_DROP(같은 mission_id) 수신 시 RELEASE.
        이미 STOWED_LOCKED 까지 끝난 mission_id 의 재발행(중복)은 무시.
        """
        msg = self._pending_arrival
        if msg is None or not self._mission_stop_active:
            return

        if self.state in (State.IDLE, State.GRIP_LOST) and msg.status == ARRIVED_PICKUP:
            if msg.mission_id == self._last_completed_mission_id:
                return  # 이미 완료된 mission_id 재발행 — 재실행 금지
            self.mission_id = msg.mission_id
            self._pending_arrival = None
            self._set_status(ARM_WORK_READY)
            self._transition(State.PERCEIVE)
        elif (self.state == State.LOCKED and msg.status == ARRIVED_PICKUP
                and msg.mission_id == self.mission_id):
            self._pending_arrival = None
            self.locked = False
            self._set_status(ARM_WORK_READY)
            self._transition(State.PERCEIVE)
        elif (self.state == State.CARRY and msg.status == ARRIVED_DROP
                and msg.mission_id == self.mission_id):
            self._pending_arrival = None
            self._transition(State.RELEASE)

    def _stamp_is_fresh(self, stamp, prev_stamp):
        """0/미래/동일·역행 stamp 거부 (계약 §5.1 heartbeat freshness 기준)."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        if t > now + STAMP_FUTURE_TOL:
            return False
        if prev_stamp is not None:
            pt = prev_stamp.sec + prev_stamp.nanosec * 1e-9
            if t <= pt:
                return False
        return True

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.effort):
                self._joint_effort[name] = msg.effort[i]
            if i < len(msg.position):
                self._joint_position[name] = msg.position[i]

    def _on_controller_fault(self, msg):
        self._controller_fault = bool(msg.data)

    # ── FSM tick ───────────────────────────────

    def _tick(self):
        self._check_chassis_mode_watchdog()
        if (self._motion_state == 'active' and self._arm_move_deadline is not None
                and self.get_clock().now() >= self._arm_move_deadline):
            self._motion_state = 'done'
            self._arm_move_deadline = None
        handler = getattr(self, f'_do_{self.state.name.lower()}', None)
        if handler:
            handler()

    def _check_chassis_mode_watchdog(self):
        """chassis_mode 수신 끊김 = default-deny(잠금 유지, §5.1)."""
        if self._last_chassis_recv_wall is None:
            return  # 아직 한 번도 못 받음 — IDLE 기본값(안 움직임)으로 이미 안전
        age = (self.get_clock().now() - self._last_chassis_recv_wall).nanoseconds * 1e-9
        if age > self.chassis_mode_timeout:
            self._mission_stop_active = False
            self._enter_locked()

    def _is_settled(self):
        """locked 하트비트(CARRYING_LOCKED/STOWED_LOCKED) 발행 전 실제 확인 — §5.1.

        tip pose(TF, base_frame←tip_link)가 연속 tick 사이 `locked_pos_tol` 이내로
        유지되고, 관절각 유한차분 속도가 `locked_vel_tol` 이내인 상태가 `locked_dwell`
        초 이상 지속돼야 True. TF 조회 실패·불안정 감지 시 dwell 타이머 리셋(안전 측
        기본값 = 미확인). controller fault(`/dynamixel/controller_fault`, 브릿지가
        Hardware Error Status 집계) 가 True 이면 즉시 미확인 처리(2026-07-15 추가 —
        이전엔 브릿지에 해당 필드가 없어 미포함이었음).
        """
        if self._controller_fault:
            self._settle_start = None
            return False
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException:
            self._settle_start = None
            return False
        t = tf.transform.translation
        pos = np.array([t.x, t.y, t.z])
        now = self.get_clock().now()

        stable = False
        if self._last_settle_pos is not None and self._last_settle_time is not None:
            dt = (now - self._last_settle_time).nanoseconds * 1e-9
            pos_delta = float(np.linalg.norm(pos - self._last_settle_pos))
            max_joint_vel = 0.0
            if dt > 1e-6:
                for name in ARM_JOINT_NAMES:
                    prev = self._last_settle_joints.get(name)
                    cur = self._joint_position.get(name)
                    if prev is not None and cur is not None:
                        max_joint_vel = max(max_joint_vel, abs(cur - prev) / dt)
            stable = (pos_delta <= self.locked_pos_tol
                      and max_joint_vel <= self.locked_vel_tol)

        self._last_settle_pos = pos
        self._last_settle_time = now
        self._last_settle_joints = dict(self._joint_position)

        if not stable:
            self._settle_start = None
            return False
        if self._settle_start is None:
            self._settle_start = now
        return (now - self._settle_start).nanoseconds * 1e-9 >= self.locked_dwell

    def _do_idle(self):
        # 계약 v2: 평상시(빈손) 주행 중 상시 하트비트 — WORK_READY 아님, STOWED_LOCKED.
        # WORK_READY는 MISSION_STOP+ArrivalStatus conjunction 수락 순간(_try_advance)의
        # 1회성 ack로 재배치됨. (STOWING에서 settle 확인 후 넘어온 상태라 여기선 재확인 안 함)
        self._set_status(ARM_STOWED_LOCKED)

    def _do_perceive(self):
        self._set_status(ARM_PERCEIVING)
        if self.pick_target is None:
            return
        if self.pick_target.pose.position.z == 0.0:   # depth 무효 (Phase 2 require_depth 기준)
            self.get_logger().warn('pick_target depth 무효 — 대기')
            return
        self._transition(State.PLAN)

    def _do_plan(self):
        """파지 목표로 이동 시작. 디스패치만 하고 DESCEND에서 대기."""
        self._set_status(ARM_PLANNING)
        if self.ik_mode == 'moveit':
            grasp_pose = self._grasp_pose()
            if grasp_pose is None:
                self._set_status(ARM_FAILED)
                self._transition(State.IDLE)
                return
            self._begin_arm_move(grasp_pose)
            self._transition(State.DESCEND)
            return

        target = self._grasp_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self._set_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        self._transition(State.DESCEND)

    def _do_descend(self):
        """MoveIt 모션 결과 대기 (저속 실행 = 하강 포함). TODO: 접촉 시 arm effort 감시."""
        self._set_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        ok = self._motion_ok
        self._motion_state = 'idle'
        self._transition(State.GRASP_CHECK if ok else State.IDLE)
        if not ok:
            self._set_status(ARM_FAILED)

    def _do_grasp_check(self):
        """그리퍼 닫고 effort(전류)로 파지 판정."""
        self._set_status(ARM_EXECUTING)
        if not self._grip_sent:
            self._send_gripper(self.gripper_close)
            self._grip_sent = True
            return
        if self._elapsed() < self.gripper_action_time:
            return
        if self._gripper_effort() >= self.grasp_thresh:
            self._transition(State.LIFT)
        else:
            self.get_logger().warn('파지 실패(effort 미달) → 재계획')
            self._transition(State.PLAN)

    def _do_lift(self):
        """수직 리프트 → 운반 자세 (base_link +Z, 현재 tip TF 기준)."""
        self._set_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            self._motion_state = 'idle'
            self._transition(State.CARRY)
            return

        # 'idle' — 리프트 모션 시작
        if self.ik_mode == 'moveit':
            lift_pose = self._carry_pose()
            if lift_pose is None:
                self.get_logger().warn('carry pose 미구현/TF 실패 — 스킵하고 CARRY 진입')
                self._transition(State.CARRY)
                return
            self._begin_arm_move(lift_pose)
            return

        target = self._lift_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('LIFT 목표 계산/이동 실패 — 스킵하고 CARRY 진입')
            self._transition(State.CARRY)

    def _do_carry(self):
        """운반 중 그리퍼 effort 감시 → 급감 시 GRIP_LOST 래치.

        `_is_settled()`(pose·관절속도·dwell 실측) 충족 전엔 `CARRYING_LOCKED` 대신
        `EXECUTING`을 발행 — 문자열만 바꾸지 않고 실제 정지 확인 후에만 locked 하트비트.
        """
        self._set_status(ARM_CARRYING_LOCKED if self._is_settled() else ARM_EXECUTING)
        if self._elapsed() < 0.5:          # 진입 직후 dwell (DROP 오탐 방지, settle과 별개)
            return
        if self._gripper_effort() < self.drop_thresh:
            self.get_logger().warn('DROP 감지 (effort 급감) → GRIP_LOST 래치')
            self._cancel_arm_motion()
            self._transition(State.GRIP_LOST)
        # ARRIVED_DROP 은 _try_advance(MISSION_STOP conjunction)에서 RELEASE로 전이

    def _do_grip_lost(self):
        """supervisor-latched hold — 자동 재시도 없음(§5.1).

        새 MISSION_STOP+ArrivalStatus(ARRIVED_PICKUP) conjunction이 다시 와야만
        `_try_advance()`가 PERCEIVE로 재진입시킴. STOW_REQUEST 수신 시
        `_on_chassis_mode`에서 RELEASE로 강제 전이(운영자 포기, `STOW_ABORTABLE_STATES`).
        """
        self._set_status(ARM_GRIP_LOST)

    def _do_lower_release(self):
        """RELEASE(그리퍼 오픈) 전에 파지 높이까지 내린다 — 화물 낙하 방지(§5.1 잔여 합의 ①).

        `PAYLOAD_ALOFT_STATES`(LIFT/CARRY)에서 STOW_REQUEST로 중단됐을 때만 경유.
        `_carry_pose`/`_lift_target_xyz`가 더한 `lift_height`만큼 되돌려 내려
        grasp 당시 높이 근처로 되돌아간 뒤에만 RELEASE로 넘어간다. TF/IK 실패 시엔
        무한정 대기하지 않고 바로 RELEASE(기존 동작, 낙하 위험을 감수)로 폴백한다 —
        높이를 못 낮추는 것보다 그리퍼를 계속 닫아 매달아두는 쪽이 더 위험할 수 있다.
        """
        self._set_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            self._motion_state = 'idle'
            self._transition(State.RELEASE)
            return

        if self.ik_mode == 'moveit':
            pose = self._lower_pose()
            if pose is None:
                self.get_logger().warn('lower pose 계산 실패 — 바로 RELEASE(낙하 위험 감수)')
                self._transition(State.RELEASE)
                return
            self._begin_arm_move(pose)
            return

        target = self._lower_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('내림 이동 실패 — 바로 RELEASE(낙하 위험 감수)')
            self._transition(State.RELEASE)

    def _lower_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 -Z(lift_height만큼) 내린 자세. `_carry_pose` 역."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'lower_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = max(0.0, t.z - self.lift_height)
        ps.pose.orientation = tf.transform.rotation
        return ps

    def _lower_target_xyz(self):
        """현재 tip 위치(base_frame)에서 -Z lift_height 만큼 내린 목표. `_lift_target_xyz` 역."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lower target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, max(0.0, t.z - self.lift_height))

    def _do_release(self):
        self._set_status(ARM_EXECUTING)
        if not self._grip_sent:
            self._send_gripper(self.gripper_open)
            self._grip_sent = True
            return
        if self._elapsed() > self.gripper_action_time:
            self._transition(State.STOWING)

    def _do_stowing(self):
        """접힘 자세로 이동 → `_is_settled()` 확인 후 `STOWED_LOCKED`.

        `stow_joint_positions`(파라미터, CAD 미검증 placeholder)로 직접 관절궤적을
        딱 한 번만(`_stow_move_sent`, 상태 진입당 1회) 발행한다. 모션 완료 전에
        `_is_settled()`를 확인하면 CARRY 종료 시점의 자세가 우연히 안정적이어서 실제로
        접히기도 전에 통과해버릴 수 있으므로, 반드시 `_motion_state=='active'`가 아닐
        때만 settle 게이트를 본다. ⚠️ `_motion_state`를 'idle'로 되돌리면 다음 tick에
        재발행 분기를 다시 타 dwell 누적 없이 궤적을 계속 재전송하는 버그가 있었음
        (2026-07-15 발견·수정) — `_stow_move_sent`는 상태를 벗어날 때(`_transition`)만
        리셋되므로 매 tick 재발행되지 않는다.
        """
        self._set_status(ARM_STOWING)
        if self.stow_joint_positions is None:
            # 목표 미설정(파라미터 길이 오류) — 접이 모션 없이 현재 자세 유지로 폴백.
            if self._is_settled():
                self._transition(State.STOWED_LOCKED)
            return
        if not self._stow_move_sent:
            self._begin_stow_move()
            self._stow_move_sent = True
            return
        if self._motion_state == 'active':
            return
        if self._is_settled():
            self._transition(State.STOWED_LOCKED)

    def _do_stowed_locked(self):
        """하역 완료 최종 권위. ⚠️ 실제 locked-heartbeat 검증(dwell 등)은 TODO."""
        self._set_status(ARM_STOWED_LOCKED)
        self._last_completed_mission_id = self.mission_id
        self.pick_target = None
        self._transition(State.IDLE)

    def _do_locked(self):
        """현재 자세 홀드: MoveIt에 새 goal 안 보냄 → 브릿지가 torque로 마지막 위치 유지.

        계약 v2 10Hz 하트비트 요구 — LOCKED 중에도 발행을 멈추면 안 됨. `_is_settled()`
        충족 전엔 `EXECUTING`(아직 정지 미확인). 충족 후 `_prev_state==LIFT`(파지 확정 뒤
        중단, 화물을 든 상태)면 `CARRYING_LOCKED`.

        그 외(PERCEIVE/PLAN/DESCEND/GRASP_CHECK 중단, 빈손)는 예전엔 정지만 확인되면
        바로 `STOWED_LOCKED`로 근사했으나, LOCKED는 새 goal을 안 보내(모션 없이 그 자리에서
        홀드) 실제로는 임의의 안 접힌 자세일 수 있다 — 파워트레인은 `STOWED_LOCKED`를
        "차가 출발해도 되는" 근거로 그대로 신뢰하므로 이는 안전 gap이었다(§5.1 잔여 합의 ②).
        `_near_stow_posture()`로 관절각이 실제 `stow_joint_positions` 근처인지 확인한
        경우에만 `STOWED_LOCKED`를 발행하고, 아니면 정직하게 `EXECUTING`을 유지해 파워트레인
        쪽 motion hold를 받는다(거짓 주행 허가보다 안전).
        """
        if not self._is_settled():
            self._set_status(ARM_EXECUTING)
            return
        if self._prev_state == State.LIFT:
            self._set_status(ARM_CARRYING_LOCKED)
            return
        self._set_status(ARM_STOWED_LOCKED if self._near_stow_posture() else ARM_EXECUTING)

    def _near_stow_posture(self):
        """현재 관절각이 `stow_joint_positions` 근처(±`stow_pos_tol_rad`)인지 확인.

        `stow_joint_positions`가 비활성(파라미터 길이 오류)이면 검증 불가하므로 기존
        동작(정지만 확인)으로 폴백한다 — 이미 그 경우엔 시작 시점에 에러 로그를 남긴다.
        """
        if self.stow_joint_positions is None:
            return True
        for name, target in zip(ARM_JOINT_NAMES, self.stow_joint_positions):
            cur = self._joint_position.get(name)
            if cur is None or abs(cur - target) > self.stow_pos_tol_rad:
                return False
        return True

    # ── MoveIt 팔 모션 ─────────────────────────

    def _begin_arm_move(self, pose_stamped):
        self._motion_state = 'active'
        self._motion_ok = False
        if not self._move.server_is_ready():
            self.get_logger().warn('move_action 서버 미준비 — MoveIt(move_group) 실행 확인')
        goal = self._build_move_group_goal(pose_stamped)
        self._move.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('MoveGroup goal 거부됨')
            self._motion_state = 'done'
            self._motion_ok = False
            return
        self._arm_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        result = future.result().result
        self._motion_ok = (result.error_code.val == MOVEIT_SUCCESS)
        self._motion_state = 'done'
        self._arm_goal_handle = None

    def _cancel_arm_motion(self):
        if self._arm_goal_handle is not None:
            self._arm_goal_handle.cancel_goal_async()
            self._arm_goal_handle = None
        self._arm_move_deadline = None
        self._motion_state = 'idle'

    def _build_move_group_goal(self, pose_stamped):
        """목표 pose → MoveGroup goal (plan & execute). tip_link를 pose로 이동."""
        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = self.tip_link
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.pos_tol]
        region.primitives.append(sphere)
        region.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = region
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = self.tip_link
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = self.orient_tol
        oc.absolute_y_axis_tolerance = self.orient_tol
        oc.absolute_z_axis_tolerance = self.orient_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        # planning_options 기본값: plan_only=False → 계획 후 컨트롤러로 실행
        return goal

    def _grasp_pose(self):
        """/pick_target(DetectedObject, frame 정보 없음) → PoseStamped(pick_frame_id)."""
        if self.pick_target is None:
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.pick_frame_id    # DetectedObject엔 header 없음 → 파라미터 사용
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = self.pick_target.pose
        return ps

    def _carry_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 +Z 로 들어올린 운반 자세.

        파지 직후의 실제 말단 자세를 TF(base_frame←tip_link)로 조회 → z 에 lift_height
        를 더하고 orientation 은 유지(박스 자세 보존). base_frame 이 planning frame 이라
        MoveIt 이 바로 계획 가능. TF 미가용 시 None → 호출부(_do_lift)가 LIFT 스킵.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'carry_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = t.z + self.lift_height
        ps.pose.orientation = tf.transform.rotation
        return ps

    # ── analytic IK (ik_mode=='analytic', URDF 3관절 한정) ─────

    def _current_arm_joint_positions(self):
        return [self._joint_position.get(j, 0.0) for j in ARM_JOINT_NAMES]

    def _grasp_target_xyz(self):
        """/pick_target(카메라 프레임) → base_frame 기준 (x,y,z). 방향은 무시(3DOF 한계)."""
        if self.pick_target is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.pick_frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'grasp target TF 조회 실패: {e}')
            return None
        out = do_transform_pose(self.pick_target.pose, tf)
        return (out.position.x, out.position.y, out.position.z)

    def _lift_target_xyz(self):
        """현재 tip 위치(base_frame)에서 +Z lift_height 만큼 든 목표."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lift target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z + self.lift_height)

    def _fk_tip(self, q):
        """3관절 각도 q=[j1,j2,j3] → tip_link 위치(base_frame) np.array, 실패 시 None."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.tip_link]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if not self._fk_client.service_is_ready():
            return None
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self._fk_node, future, timeout_sec=1.0)
        res = future.result()
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def _solve_position_ik(self, target_xyz, q_init):
        """FK + 수치 자코비안(finite-difference) 로 위치만 맞추는 3DOF IK.

        ARM_JOINT_NAMES가 앞 3관절(arm_joint_1~3)만 써서 MoveIt 6DOF pose IK 대신 이 방식을
        기본으로 씀(HW-7 실측 확인, compute_ik가 현재 실제 tip pose에도 NO_IK_SOLUTION 반환하던
        문제 회피 — URDF 자체는 5축 다 있음, solver가 아직 5DOF로 확장 안 됨). 방향은 포기하고
        위치만 댐핑 최소자승(Levenberg-Marquardt 유사)으로 반복 수렴.
        """
        q = np.array(q_init, dtype=float)
        target = np.array(target_xyz, dtype=float)
        eps = 0.05
        lam = 0.01
        max_step = 0.4

        p = self._fk_tip(q)
        if p is None:
            return None

        for _ in range(self.ik_max_iters):
            err = target - p
            if np.linalg.norm(err) < self.ik_tol:
                return q.tolist()
            J = np.zeros((3, 3))
            for i in range(3):
                dq = np.zeros(3)
                dq[i] = eps
                p2 = self._fk_tip(q + dq)
                if p2 is None:
                    return None
                J[:, i] = (p2 - p) / eps
            delta = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
            norm = np.linalg.norm(delta)
            if norm > max_step:
                delta *= max_step / norm
            q = q + delta
            p = self._fk_tip(q)
            if p is None:
                return None

        if np.linalg.norm(target - p) < self.ik_accept_tol:
            return q.tolist()
        return None

    def _move_to_xyz(self, target_xyz):
        """target_xyz(base_frame) 로 analytic IK 계산 → /arm_controller/joint_trajectory 직접 발행."""
        q_current = self._current_arm_joint_positions()
        solution = self._solve_position_ik(target_xyz, q_current)
        if solution is None:
            self.get_logger().warn(f'analytic IK 실패 — 목표 도달 불가: {target_xyz}')
            return False
        self._publish_joint_trajectory(solution, q_current)
        self.get_logger().info(
            f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동')
        return True

    def _begin_stow_move(self):
        """`stow_joint_positions` 목표로 직접 관절궤적 발행 (MoveIt 미경유).

        접힘 자세는 충돌 회피가 필요 없는 known-safe 설정값이라 가정하므로(실측 후
        캘리브 전제), ik_mode와 무관하게 항상 `_arm_traj_pub`로 직접 명령한다 —
        analytic IK를 거칠 필요가 없다(목표가 이미 관절각이지 xyz가 아님).
        """
        q_current = self._current_arm_joint_positions()
        self._publish_joint_trajectory(self.stow_joint_positions, q_current)

    def _publish_joint_trajectory(self, target_positions, q_current):
        """목표 관절각으로 단일 포인트 궤적 발행 + 모션 진행 상태 갱신 (공통 헬퍼)."""
        delta = max(abs(a - b) for a, b in zip(target_positions, q_current))
        duration = max(1.0, min(5.0, delta / max(self.arm_move_speed, 0.05)))

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target_positions]
        pt.time_from_start = Duration(sec=int(duration))
        traj.points.append(pt)
        self._arm_traj_pub.publish(traj)

        self._motion_state = 'active'
        self._motion_ok = True
        self._arm_move_deadline = self.get_clock().now() + RclpyDuration(
            seconds=duration + 0.5)

    # ── 그리퍼 ─────────────────────────────────

    def _send_gripper(self, position):
        """gripper_controller에 FollowJointTrajectory 단일 점 전송 (fire-and-forget)."""
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)] * len(self.gripper_joints)
        pt.time_from_start = Duration(sec=int(self.gripper_action_time))
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if not self._grip.server_is_ready():
            self.get_logger().warn('gripper_controller 액션 서버 미준비')
        self._grip.send_goal_async(goal)

    def _gripper_effort(self):
        """그리퍼 effort(전류) 절댓값 — /joint_states 의 gripper_drive_joint effort.

        랙피니언 2모터(ID 3,4)는 브릿지가 두 전류의 max-abs 를 이 관절 effort 로 이미
        집계해 보고하므로, 여기서는 대표 관절 하나만 읽으면 된다.
        """
        return abs(self._joint_effort.get(self.gripper_joints[0], 0.0))

    # ── 공통 ───────────────────────────────────

    def _elapsed(self):
        return (self.get_clock().now() - self._state_enter_t).nanoseconds * 1e-9

    def _transition(self, new_state):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state = new_state
        self._state_enter_t = self.get_clock().now()
        self._grip_sent = False
        self._stow_move_sent = False

    def _set_status(self, status):
        """발행할 현재 상태를 갱신한다. 실제 발행은 _publish_heartbeat 가 전담한다.

        여기서 직접 publish 하지 말 것 — 발행 경로가 둘이 되면 stamp 순서가 뒤집힐 수
        있고, 파워트레인은 stamp 역행을 영구 latch 로 처벌한다(contract.py 참고).
        """
        self._status = status

    def _publish_heartbeat(self):
        """계약 heartbeat — 현재 상태를 10Hz 로 발행. **유일한 /arm_status 발행 지점.**"""
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self.mission_id
        msg.status = self._status
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFsmNode()
    # heartbeat 타이머가 _tick(analytic IK 의 FK 블로킹 대기)에 굶지 않도록 멀티스레드.
    executor = MultiThreadedExecutor()
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


if __name__ == '__main__':
    main()
