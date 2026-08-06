#!/usr/bin/env python3
"""감속기 축(arm_joint_2/3)의 기어비를 손으로 돌려서 실측하는 도구 (2026-08-07).

## 왜 필요한가

`teleop_core_node.py` 의 rad 도메인은 **서보축 각도**다(기어비를 적용하지 않는다).
반면 MoveIt/`arm_fsm` 은 **URDF 관절 각도**로 동작한다. 둘을 잇는 기어비가
`arm_joint_2/3` 에서 "약 10:1"(사용자 추정, 확정 아님)로만 알려져 있고, URDF 의
해당 관절 리밋은 `0~π` 자동생성 placeholder 라 역산도 불가능하다.

기어비를 모르면 IK 가 낸 관절각을 서보 tick 으로 못 바꾼다 —
`moveit_dynamixel_bridge` 의 `gear_ratio` 기본값이 안전 측(1.0, 언더슈트)으로
박혀 있는 이유이자, 실측 전까지 실제 파지를 돌리면 안 되는 이유다.

## 쓰는 법

**반드시 `moveit_dynamixel_bridge` 를 `read_only:=true` 로 띄운 상태에서** 쓴다
(토크가 꺼져 있어야 손으로 돌릴 수 있고, 이 스크립트는 아무것도 명령하지 않는다).

    ros2 run dynamixel_control moveit_dynamixel_bridge --ros-args -p read_only:=true
    python3 src/dynamixel_control/scripts/measure_gear_ratio.py arm_joint_2

1. 관절을 시작 위치에 두고 Enter → 시작 서보각 기록
2. 관절을 **각도계로 잴 수 있는 만큼** 크게 돌린다(클수록 오차가 준다 — 90° 권장)
3. Enter → 끝 서보각 기록
4. 실제로 돌린 **관절 각도**를 degree 로 입력
5. 기어비 = 서보축 회전량 / 관절 회전량

측정된 값은 브릿지에 이렇게 넣는다(코드 수정 불필요):

    ros2 run dynamixel_control moveit_dynamixel_bridge --ros-args \
        -p gear_ratios:="['arm_joint_2:9.8','arm_joint_3:10.1']"

⚠️ 각도계 오차가 그대로 기어비 오차가 된다. 90° 를 ±2° 오차로 재면 기어비도
   약 ±2% 흔들린다 — 파지 정밀도가 부족하면 더 큰 각도로 다시 잴 것.
"""
import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GearRatioMeasurer(Node):
    def __init__(self, joint_name):
        super().__init__("measure_gear_ratio")
        self.joint_name = joint_name
        self.latest = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if name == self.joint_name and i < len(msg.position):
                self.latest = msg.position[i]
                return

    def wait_for_sample(self, timeout_s=10.0):
        """최신 서보각 1개를 받을 때까지 스핀. 없으면 None."""
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while rclpy.ok() and self.latest is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                return None
        return self.latest

    def read_now(self):
        """버퍼를 비우고 새 샘플을 하나 받아 반환(과거 값 재사용 방지)."""
        self.latest = None
        return self.wait_for_sample()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("joint_name", help="측정할 관절 (예: arm_joint_2)")
    args = parser.parse_args()

    rclpy.init()
    node = GearRatioMeasurer(args.joint_name)

    try:
        domain = os.environ.get("ROS_DOMAIN_ID", "0(미설정)")
        print(f"[{args.joint_name}] /joint_states 수신 대기 중... (ROS_DOMAIN_ID={domain})")
        if node.read_now() is None:
            print(
                "서보각을 못 받았습니다. 확인 순서:\n"
                f"  1) ROS_DOMAIN_ID 가 bridge 와 같은가? (지금 이 셸은 {domain})\n"
                "     — bridge 를 다른 도메인으로 띄웠다면 여기서도 같은 값을 export 해야 한다.\n"
                "  2) bridge 가 read_only:=true 로 떠 있는가? (ros2 topic hz /joint_states)\n"
                f"  3) 관절 이름 '{args.joint_name}' 이 맞는가? "
                "(bridge 기동 로그의 arm=[...] 목록 참고)",
                file=sys.stderr,
            )
            return 1

        input(f"\n1) 관절을 시작 위치에 두고 Enter (현재 서보각 {node.latest:+.4f} rad) ")
        start = node.read_now()
        if start is None:
            print("시작값 수신 실패", file=sys.stderr)
            return 1
        print(f"   시작 서보각 = {start:+.4f} rad")

        input("\n2) 관절을 크게(90° 권장) 돌린 뒤 Enter ")
        end = node.read_now()
        if end is None:
            print("끝값 수신 실패", file=sys.stderr)
            return 1
        print(f"   끝 서보각   = {end:+.4f} rad")

        servo_delta = end - start
        print(f"   서보축 회전량 = {servo_delta:+.4f} rad "
              f"({math.degrees(servo_delta):+.2f}°, {servo_delta / (2 * math.pi):+.3f} 회전)")

        if abs(servo_delta) < 0.05:
            print("\n서보가 거의 안 움직였습니다 — 관절을 실제로 돌렸는지 확인하세요.",
                  file=sys.stderr)
            return 1

        raw = input("\n3) 실제로 돌린 **관절** 각도를 degree 로 입력 (예: 90): ").strip()
        try:
            joint_deg = float(raw)
        except ValueError:
            print(f"숫자가 아닙니다: {raw!r}", file=sys.stderr)
            return 1
        if abs(joint_deg) < 1e-6:
            print("관절 각도가 0 이면 기어비를 계산할 수 없습니다.", file=sys.stderr)
            return 1

        joint_delta = math.radians(joint_deg)
        ratio = servo_delta / joint_delta

        print("\n" + "=" * 58)
        print(f"  {args.joint_name} 기어비 = {abs(ratio):.3f} : 1")
        print("=" * 58)
        if ratio < 0:
            print("  ⚠️ 부호가 음수다 — 서보와 관절이 반대로 돈다는 뜻이다.")
            print("     JOINT_CONFIG 의 direction 부호를 뒤집어야 할 수 있다")
            print("     (기어비 자체는 절대값을 쓴다).")
        print("\n  브릿지에 반영:")
        print(f"    -p gear_ratios:=\"['{args.joint_name}:{abs(ratio):.3f}']\"")
        print()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
