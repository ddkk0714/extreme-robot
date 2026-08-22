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

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_VELOCITY = 104
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_GOAL_VELOCITY = 4
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_POSITION = 4

# HARDWARE_ERROR_STATUS(70,1) ~ PRESENT_POSITION(132,4) 은 X-시리즈 컨트롤 테이블에서
# 연속 주소 범위라, 70부터 66바이트를 한 번의 SyncRead 로 받아 fault/current/position 을
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

        # 털털이 ZIP에 모터 ID/방향/속도가 없어 모두 fail-closed 기본값이다.
        self.declare_parameter("cleaning_actuator_joint", "")
        self.declare_parameter("cleaning_actuator_id", -1)
        self.declare_parameter("cleaning_direction", 0)
        self.declare_parameter("cleaning_velocity_raw", 0)

        self.cleaning_actuator_joint = self.get_parameter("cleaning_actuator_joint").value
        self.cleaning_actuator_id = int(self.get_parameter("cleaning_actuator_id").value)
        self.cleaning_direction = int(self.get_parameter("cleaning_direction").value)
        self.cleaning_velocity_raw = int(self.get_parameter("cleaning_velocity_raw").value)
        self.cleaning_configured = (
            bool(self.cleaning_actuator_joint) and self.cleaning_actuator_id >= 0
            and self.cleaning_direction in (-1, 1) and self.cleaning_velocity_raw > 0
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

        # 토크 ON에 성공해 SyncRead 에 실제로 등록된 ID만 추적 — 이후 매 tick 이 ID들의
        # 응답 유무/Hardware Error Status 로 controller fault 를 판정한다(등록 안 된 ID는
        # 애초에 버스에 없거나 비활성화된 것으로 간주해 fault 판정에서 제외).
        self.active_ids = set()

        # 팔 서보: 토크 ON 성공한 ID만 SyncRead 등록
        for joint_name, config in JOINT_CONFIG.items():
            if self._enable_torque(config["id"], joint_name):
                self.group_sync_read.addParam(config["id"])
                self.active_ids.add(config["id"])

        if self.cleaning_configured:
            self._configure_cleaning_actuator()

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )
        self.create_subscription(Bool, "/cleaning/enable", self._on_cleaning_enable, 10)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
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
            f"cleaning_actuator={self.cleaning_actuator_joint or 'UNCONFIGURED'})"
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
        if not self.cleaning_configured:
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

    def goal_callback(self, goal_request):
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
        self.group_sync_read.txRxPacket()
        # 일부 ID가 버스에 없어도 응답받은 ID만 처리 (result 무시)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # controller fault 집계 — SyncRead 에 등록된(토크 ON 성공) ID 중 하나라도
        # Hardware Error Status != 0 이거나 이번 tick 응답이 없으면 fault=True.
        # 응답 없음도 fault 로 보는 이유: 활성 등록된 서보가 갑자기 무응답이면 버스/전원
        # 이상일 수 있어 "정상"으로 오인하면 안 됨(안전 측 기본값).
        fault = False

        # 팔 관절: position(rad) + effort(raw current, signed)
        for joint_name, config in JOINT_CONFIG.items():
            dxl_id = config["id"]
            if dxl_id not in self.active_ids:
                continue
            sample = self._read_sample(dxl_id)
            if sample is None:
                fault = True
                continue
            current_raw, tick, hw_error = sample
            if hw_error != 0:
                fault = True
            msg.name.append(joint_name)
            msg.position.append(self.tick_to_rad(joint_name, tick))
            msg.effort.append(float(current_raw))

        if (self.cleaning_configured
                and self.cleaning_actuator_id in self.active_ids):
            sample = self._read_sample(self.cleaning_actuator_id)
            if sample is None:
                fault = True
            else:
                current_raw, tick, hw_error = sample
                fault = fault or hw_error != 0
                msg.name.append(self.cleaning_actuator_joint)
                msg.position.append(float(to_signed(tick, LEN_PRESENT_POSITION)))
                msg.effort.append(float(current_raw))

        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=fault))

    def _read_sample(self, dxl_id):
        """SyncRead 블록에서 (signed current, position tick, hardware error) 추출.

        미수신 시 None.
        """
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT):
            return None
        if not self.group_sync_read.isAvailable(
                dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            return None
        hw_error = self.group_sync_read.getData(
            dxl_id, ADDR_HARDWARE_ERROR_STATUS, LEN_HARDWARE_ERROR_STATUS)
        current_raw = to_signed(
            self.group_sync_read.getData(
                dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT),
            LEN_PRESENT_CURRENT,
        )
        tick = self.group_sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        return current_raw, tick, hw_error

    def destroy_node(self):
        if self.cleaning_configured:
            self.packet_handler.write4ByteTxRx(
                self.port_handler, self.cleaning_actuator_id, ADDR_GOAL_VELOCITY, 0)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.cleaning_actuator_id,
                ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        for config in JOINT_CONFIG.values():
            self.packet_handler.write1ByteTxRx(
                self.port_handler, config["id"], ADDR_TORQUE_ENABLE, TORQUE_DISABLE
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
