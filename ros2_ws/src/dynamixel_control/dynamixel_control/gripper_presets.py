#!/usr/bin/env python3
"""그리퍼 모듈별 설정 (moveit_dynamixel_bridge / arm_fsm_node 공용).

robot_arm_description의 urdf/grippers/<gripper>.xacro 모듈화(xacro:arg gripper)에
대응하는 코드 레벨 preset. 새 그리퍼 모듈을 추가할 때는 두 노드를 건드리지 않고
이 dict에 항목만 추가하면 됨.
"""

GRIPPER_PRESETS = {
    "gripper_a": {
        # 랙피니언 그리퍼: XL430 2개(ID 3,4)가 **같은 랙을 공유**해 같은 방향으로 구동한다.
        # 랙이 하나뿐이라 두 모터는 항상 동일 goal_tick 을 쓰면 되고(단일 tick 확정),
        # 모터별 오프셋/direction 부호는 불필요하다. URDF 측은 여전히 gripper_drive_joint
        # 하나만 실제 구동, 나머지 조 관절은 <mimic> 으로 종속(2026-07-15 Isaac Sim export).
        # 기존 단일 서보(ID 5)에서 2모터 랙피니언으로 전환.
        "gripper_joints": ["gripper_drive_joint"],
        "gripper_ids": [3, 4],
        # 아래 tick 은 HW-8 단일서보 실측(215도/280도) 유산값 — 랙이 같아 tick 체계는
        # 동일하나, 새 조립체에서 open/close 각도가 그대로인지는 재확인 권장.
        "gripper_open_tick": 2446,
        "gripper_close_tick": 3186,
        "gripper_open_m": 0.02,       # 선형보간 기준점 placeholder — 실측 캘리브 필요
        "gripper_close_m": 0.0,
        # effort 임계 — 브릿지가 두 모터 전류의 max-abs 를 gripper_drive_joint effort 로 보고한다.
        "grasp_effort_thresh": 80.0,  # placeholder — 2모터 max-abs 기준 재실측 필요
        "drop_effort_thresh": 20.0,   # placeholder — 2모터 max-abs 기준 재실측 필요
        "gripper_action_time": 1.0,
    },
}

DEFAULT_GRIPPER = "gripper_a"


def get_preset(gripper_type, logger=None):
    preset = GRIPPER_PRESETS.get(gripper_type)
    if preset is None:
        if logger is not None:
            logger.warn(
                f"Unknown gripper_type '{gripper_type}', falling back to '{DEFAULT_GRIPPER}'"
            )
        preset = GRIPPER_PRESETS[DEFAULT_GRIPPER]
    return preset
