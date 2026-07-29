#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead

from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset


ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
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

        # --- 그리퍼 파라미터 (단일 서보 양 핑거 미러링) ---
        # gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter("gripper_type", DEFAULT_GRIPPER)
        self.gripper_type = self.get_parameter("gripper_type").value
        preset = get_preset(self.gripper_type, self.get_logger())

        self.declare_parameter("gripper_joints", preset["gripper_joints"])
        self.declare_parameter("gripper_ids", preset["gripper_ids"])  # 빈 배열이면 그리퍼 비활성
        self.declare_parameter("gripper_open_m", preset["gripper_open_m"])
        self.declare_parameter("gripper_close_m", preset["gripper_close_m"])
        self.declare_parameter("gripper_open_tick", preset["gripper_open_tick"])
        self.declare_parameter("gripper_close_tick", preset["gripper_close_tick"])
        self.declare_parameter("read_only", False)

        self.gripper_joints = list(self.get_parameter("gripper_joints").value)
        self.gripper_ids = list(self.get_parameter("gripper_ids").value)
        self.gripper_open_m = float(self.get_parameter("gripper_open_m").value)
        self.gripper_close_m = float(self.get_parameter("gripper_close_m").value)
        self.gripper_open_tick = int(self.get_parameter("gripper_open_tick").value)
        self.gripper_close_tick = int(self.get_parameter("gripper_close_tick").value)
        self.read_only = bool(self.get_parameter("read_only").value)

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

        # 토크 ON에 성공해 SyncRead 에 실제로 등록된 ID만 추적 — 이후 매 tick 이 ID들의
        # 응답 유무/Hardware Error Status 로 controller fault 를 판정한다(등록 안 된 ID는
        # 애초에 버스에 없거나 비활성화된 것으로 간주해 fault 판정에서 제외).
        self.active_ids = set()

        if self.read_only:
            # 진단용: register write 없이 CLI로 지정한 그리퍼 ID만 읽는다.
            for gid in self.gripper_ids:
                if self.group_sync_read.addParam(gid):
                    self.active_ids.add(gid)
        else:
            # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for joint_name, config in JOINT_CONFIG.items():
                if self._enable_torque(config["id"], joint_name):
                    self.group_sync_read.addParam(config["id"])
                    self.active_ids.add(config["id"])

            # 그리퍼 서보: 토크 ON 성공한 ID만 SyncRead 등록
            for gid in self.gripper_ids:
                if self._enable_torque(gid, f"gripper(id {gid})"):
                    self.group_sync_read.addParam(gid)
                    self.active_ids.add(gid)

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
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
            f"read_only={self.read_only})"
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

    def rad_to_tick(self, joint_name, rad):
        config = JOINT_CONFIG[joint_name]
        tick = config["center"] + config["direction"] * rad * TICKS_PER_RAD
        tick = int(round(tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        config = JOINT_CONFIG[joint_name]
        return (tick - config["center"]) / (config["direction"] * TICKS_PER_RAD)

    def gripper_m_to_tick(self, meters):
        span = self.gripper_open_tick - self.gripper_close_tick
        denom = self.gripper_open_m - self.gripper_close_m
        frac = 0.0 if denom == 0.0 else (meters - self.gripper_close_m) / denom
        tick = int(round(self.gripper_close_tick + frac * span))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def gripper_tick_to_m(self, tick):
        span = self.gripper_open_tick - self.gripper_close_tick
        if span == 0:
            return self.gripper_close_m
        frac = (tick - self.gripper_close_tick) / span
        return self.gripper_close_m + frac * (self.gripper_open_m - self.gripper_close_m)

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

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

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
            # 단일 구동 조인트(gripper_drive_joint)만 사용 — 나머지 8개는 URDF mimic으로 종속
            target_m = None
            for jn in self.gripper_joints:
                if jn in name_to_pos:
                    target_m = name_to_pos[jn]
                    break
            if target_m is not None:
                self._write_gripper(target_m)
            else:
                self.get_logger().warn(
                    f"Gripper goal has no known finger joint {self.gripper_joints}"
                )

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Gripper command sent to Dynamixel"
        return result

    def _write_gripper(self, meters):
        if self.read_only:
            self.get_logger().warn("Read-only mode: ignoring gripper command")
            return
        goal_tick = self.gripper_m_to_tick(meters)
        for gid in self.gripper_ids:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, gid, ADDR_GOAL_POSITION, goal_tick
            )
            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"Gripper write failed: id={gid}, result={result}, error={error}"
                )
        self.get_logger().info(
            f"gripper -> {meters:.4f} m -> tick {goal_tick} "
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

        # XL430-W250 그리퍼: 주소 126은 signed Present Load(0.1% 추정 부하)다.
        # 모든 그리퍼 ID를 검사하고 가장 큰 abs(load)를 논리 조인트 effort로 보고한다.
        gripper_samples = []
        for gid in self.gripper_ids:
            if gid not in self.active_ids:
                fault = True
                continue
            sample = self._read_sample(gid)
            if sample is None:
                fault = True
                continue
            load_raw, tick, hw_error = sample
            if hw_error != 0:
                fault = True
            gripper_samples.append((load_raw, to_signed(tick, LEN_PRESENT_POSITION)))

        if gripper_samples:
            representative_tick = gripper_samples[0][1]
            max_abs_load = max(abs(sample[0]) for sample in gripper_samples)
            finger_m = self.gripper_tick_to_m(representative_tick)
            for jn in self.gripper_joints:
                msg.name.append(jn)
                msg.position.append(finger_m)
                msg.effort.append(float(max_abs_load))

        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """SyncRead 블록에서 (signed address-126 feedback, position, hw error) 추출.

        미수신 시 None.
        """
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_LOAD, LEN_PRESENT_LOAD):
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
        tick = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return feedback_raw, tick, hw_error

    def destroy_node(self):
        if not self.read_only:
            for config in JOINT_CONFIG.values():
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, config["id"], ADDR_TORQUE_ENABLE, TORQUE_DISABLE
                )
            for gid in self.gripper_ids:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, gid, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
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
