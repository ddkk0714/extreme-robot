#!/usr/bin/env python3
"""ROS 그래프 검증을 위한 하드웨어 없는 MoveIt 컨트롤러 엔드포인트.

이 모듈은 의도적으로 Dynamixel SDK를 import하지 않고 직렬 장치를 열지 않는다.
MoveIt이 사용하는 팔 FollowJointTrajectory 인터페이스만 모사하고, 가상 관절
피드백을 발행하며 수락한 모든 명령을 기록한다.
"""

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory


ARM_JOINTS = (
    'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5')
GRIPPER_JOINT = 'gripper_drive_joint'
ALL_JOINTS = ARM_JOINTS + (GRIPPER_JOINT,)
DEFAULT_POSITIONS = (1.405, -0.1158041728290909, -0.11694705016553897,
                     0.3129320807286708, -0.007669903939428206, 0.0)


class MockMoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__('mock_moveit_dynamixel_bridge')
        self.declare_parameter('gripper_change_mode', True)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('initial_positions', list(DEFAULT_POSITIONS))
        self.declare_parameter('gripper_effort', 1.0e9)
        self.gripper_change_mode = bool(
            self.get_parameter('gripper_change_mode').value)
        initial = list(self.get_parameter('initial_positions').value)
        if len(initial) != len(ALL_JOINTS):
            raise RuntimeError(
                f'initial_positions must contain {len(ALL_JOINTS)} values')

        self._lock = threading.Lock()
        self._positions = dict(zip(ALL_JOINTS, map(float, initial)))
        self._velocities = {name: 0.0 for name in ALL_JOINTS}
        self._efforts = {name: 0.0 for name in ALL_JOINTS}
        self._efforts[GRIPPER_JOINT] = float(
            self.get_parameter('gripper_effort').value)

        received_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self.fault_pub = self.create_publisher(
            Bool, '/dynamixel/controller_fault', 10)
        self.received_pub = self.create_publisher(
            JointTrajectory, '/mock_bridge/received_trajectory', received_qos)
        self.received_gripper_pub = self.create_publisher(
            JointTrajectory, '/mock_bridge/received_gripper_trajectory',
            received_qos)

        callback_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group)
        self.gripper_action_server = ActionServer(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory,
            goal_callback=self.gripper_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group)

        rate = float(self.get_parameter('publish_rate').value)
        if rate <= 0.0:
            raise RuntimeError('publish_rate must be positive')
        self.create_timer(1.0 / rate, self.publish_state)
        self.get_logger().info(
            'Hardware-free mock bridge started: serial disabled, SDK unused, '
            f'gripper_change_mode={self.gripper_change_mode}')

    def goal_callback(self, request):
        if not self.gripper_change_mode:
            self.get_logger().error(
                'Rejecting mock goal: gripper_change_mode must be true')
            return GoalResponse.REJECT
        if not request.trajectory.points:
            return GoalResponse.REJECT
        unknown = set(request.trajectory.joint_names) - set(ARM_JOINTS)
        if unknown:
            self.get_logger().error(
                f'Rejecting non-arm trajectory joints: {sorted(unknown)}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def gripper_goal_callback(self, request):
        if not request.trajectory.points:
            return GoalResponse.REJECT
        if tuple(request.trajectory.joint_names) != (GRIPPER_JOINT,):
            self.get_logger().error(
                'Rejecting mock gripper trajectory joints: '
                f'{list(request.trajectory.joint_names)}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.received_pub.publish(trajectory)
        final = trajectory.points[-1]
        with self._lock:
            for name, position in zip(trajectory.joint_names, final.positions):
                self._positions[name] = float(position)
                self._velocities[name] = 0.0

        duration = final.time_from_start
        values = ', '.join(
            f'{name}={position:.9f}'
            for name, position in zip(trajectory.joint_names, final.positions))
        self.get_logger().info(
            f'Received arm trajectory: points={len(trajectory.points)}, '
            f'final_time={duration.sec}.{duration.nanosec:09d}s, final=[{values}]')
        self.publish_state()
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Hardware-free trajectory accepted'
        return result

    def execute_gripper_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.received_gripper_pub.publish(trajectory)
        final = trajectory.points[-1]
        with self._lock:
            self._positions[GRIPPER_JOINT] = float(final.positions[0])
            self._velocities[GRIPPER_JOINT] = 0.0
        self.get_logger().info(
            'Received single-motor gripper trajectory: '
            f'{GRIPPER_JOINT}={float(final.positions[0]):.9f}')
        self.publish_state()
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Hardware-free gripper trajectory accepted'
        return result

    def publish_state(self):
        now = self.get_clock().now().to_msg()
        with self._lock:
            msg = JointState()
            msg.header.stamp = now
            msg.name = list(ALL_JOINTS)
            msg.position = [self._positions[name] for name in ALL_JOINTS]
            msg.velocity = [self._velocities[name] for name in ALL_JOINTS]
            msg.effort = [self._efforts[name] for name in ALL_JOINTS]
        self.joint_state_pub.publish(msg)
        self.fault_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = MockMoveItDynamixelBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
