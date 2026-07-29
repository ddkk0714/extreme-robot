#!/usr/bin/env python3
"""그리퍼 모듈별 설정 (moveit_dynamixel_bridge / arm_fsm_node 공용).

새 그리퍼 모듈을 추가할 때는 두 노드를 건드리지 않고 이 dict에 항목만 추가하면 됨.
"""

GRIPPER_PRESETS = {
    "gripper_a": {
        # 랙피니언 그리퍼: XL430 2개(ID 3,4)가 각자 피니언을 돌리고, 두 모터는 항상
        # **같은 방향·같은 goal_tick** 으로 구동한다 → 좌우 조가 대칭으로 같이 열리고 닫힌다.
        # 그래서 모터별 오프셋/direction 부호가 불필요하고, 구동 조인트도 하나로 충분하다.
        #
        # URDF(2026-07-16 랙피니언 export 이식) 기준 구동 조인트는 gripper_left_pinion_joint
        # (revolute, rad) 하나뿐이고 나머지 3개(우 피니언·좌우 랙)는 <mimic> 으로 종속된다.
        # 부호 규약: 0.0 = 완전 닫힘, 양수로 갈수록 열림, 상한 1.9444 rad.
        "gripper_joints": ["gripper_left_pinion_joint"],
        "gripper_ids": [3, 4],
        # ⚠️ 아래 tick 은 HW-8 단일서보(ID 5) 실측(215도/280도) 유산값이라 새 2모터 조립체
        #    기준으로는 **미검증**이다. 게다가 tick span 740 은 XL430 해상도(651.9 tick/rad)로
        #    환산하면 1.135 rad 뿐이라, URDF 가동범위 1.9444 rad(≈1267 tick)와 맞지 않는다.
        #    → 실기에서 열림/닫힘 끝단 tick 을 재실측해서 이 4개 값을 함께 갱신할 것.
        "gripper_open_tick": 2446,
        "gripper_close_tick": 3186,
        "gripper_open_rad": 1.9444444444444444,   # URDF 상한(완전 열림)
        "gripper_close_rad": 0.0,                 # URDF 하한(완전 닫힘)
        # effort 임계 — 브릿지가 두 모터 전류의 max-abs 를 구동 조인트 effort 로 보고한다.
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
