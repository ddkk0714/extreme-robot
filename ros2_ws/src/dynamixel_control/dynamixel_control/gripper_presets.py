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
        # ⚠️ 2026-08-02 실기로 **단일 모터(ID 3) 구동으로 전환**했다. 원래는 ID 3,4 를
        # 같은 tick 으로 동시 구동했는데, 두 독립 위치제어 루프가 같은 강체 레일을
        # 밀면서 미세하게 어긋나 서로 힘을 겨루고 → 전류가 계속 상승해 트립,
        # 트립 후엔 goal_position 을 계속 갱신해도 두 모터 다 velocity/current=0 으로
        # 응답을 멈추는 증상이 재현됐다(하드웨어 에러 플래그는 안 뜸, 손으로 돌리면
        # 걸리는 데 없음 → 기구 문제가 아니라 서보끼리 버티는 문제로 확인).
        # 레일에 피니언이 2개 물려 있어도 하나만 구동하면 나머지는 자유롭게 딸려
        # 돈다. ID 4 는 토크도 안 걸어 자유회전으로 둔다(레일에 저항을 주지 않음).
        # 다시 2모터를 쓰려면 마스터-팔로워(하나는 위치제어, 하나는 그 위치를 추종하는
        # 전류/토크 제어)처럼 서로 겨루지 않는 구조가 필요하다.
        # teleop_core_node.py 의 DEFAULT_MOTOR_IDS 도 같은 결론으로 ID 3 만 쓴다.
        "gripper_ids": [3],
        # 2026-08-07 실기 재실측(scripts/measure_gripper_endpoints.py, ID 3 단일 구동).
        # 이전 값(open=2446 / close=3186)은 HW-8 단일서보(ID 5) 유산값이라 현재 조립과
        # 안 맞았다 — 실제로 그리퍼가 tick 974 일 때 5.81 rad(URDF 상한의 3배)로 보고됐다.
        #
        # ⚠️ **개폐 방향 부호가 뒤집혔다.** 예전엔 open(2446) < close(3186) 이라 "열기 =
        #    tick 감소" 였는데, 실측은 open(1083) > close(-401) 로 "열기 = tick 증가" 다.
        #    옛 값으로 구동했다면 여닫이가 반대로 갔다.
        #
        # ⚠️ close_tick 이 **음수**다 → 다회전(Extended Position) 영역이라 아래 "extended"
        #    가 True 여야 한다. 단일회전으로 clamp 하면 완전 닫힘이 tick 0 에서 잘려
        #    401 tick(≈35°) 덜 닫힌다.
        "gripper_open_tick": 1083,
        "gripper_close_tick": -401,
        # 이 그리퍼는 Extended Position Control Mode 로 돌아간다(실기 ID 3 확인). 스트로크가
        # 서보 한 바퀴에 육박해 단일회전으로 두면 wrap 경계가 사용 범위 한가운데 걸려
        # 양 끝이 막힌다 — 2026-08-02 에 실제로 그 증상을 겪었다(teleop_core 의
        # EXTENDED_POSITION_NAMES 에 그리퍼가 들어있는 것과 같은 이유).
        "extended": True,
        "gripper_open_rad": 1.9444444444444444,   # URDF 상한(완전 열림)
        "gripper_close_rad": 0.0,                 # URDF 하한(완전 닫힘)
        # effort 임계 — `arm_fsm._gripper_effort()` 가 **abs()** 로 비교하므로 부호는
        # 무관하다(2026-07-28 측정이 음수로 기록된 것과도 크기로 비교 가능).
        #   파지 판정: effort >= grasp_effort_thresh
        #   낙하 판정: effort <  drop_effort_thresh
        #
        # 2026-08-07 실기 실측(ID 3 단일 구동, 물체 물린 채 0.10 rad 씩 단계적 조임):
        #   빈손 정지 유지      62 ~ 119
        #   빈손 이동 중 마찰   91 ~ 167  (스텝 이동 중 순간값)
        #   물체 접촉 시작     290       (position ≈ 0.29 rad, 위치 지연 시작)
        #   물체 꽉 조임       495       (position ≈ 0.23 rad, 여기서 측정 중단)
        # → 정지 상태에서 빈손(≤119)과 파지(≥290)가 깨끗하게 갈린다. FSM 은 이동이
        #   끝난 뒤(gripper_action_time 경과 후) 비교하므로 정지 기준으로 잡는다.
        #
        # ⚠️ 이전 placeholder(80/20)는 **실제로 틀렸다**: 빈손 완전닫힘 유지 effort 가
        #    119 라 80 을 넘어 **헛파지를 매번 "성공"으로 판정**했고, effort 가 20 밑으로
        #    내려가는 일이 없어 낙하 판정은 **영영 발화하지 않았다.**
        #
        # ⚠️ 아직 **1회 측정**이다. CLAUDE.md 가 요구하는 empty/grasp/drop 각 5회 반복은
        #    미완 — 물체 종류·파지 위치에 따라 접촉 effort 가 290 보다 낮을 수 있으니,
        #    파지에 성공했는데 FSM 이 실패로 보면 grasp 값을 먼저 낮춰볼 것.
        "grasp_effort_thresh": 250.0,  # 빈손 상한(119) 과 접촉(290) 사이
        "drop_effort_thresh": 200.0,   # 빈손 상한 위 — 물체가 빠지면 즉시 빈손 수준으로 떨어진다
        # FSM 이 개폐 명령을 낸 뒤 effort 를 읽기까지 기다리는 시간 [s].
        #
        # ⚠️ 2026-08-09 실기: 1.0 이면 **닫히는 도중에 판정**해서 grasp effort 가 0.0 으로
        #    읽히고 파지가 무조건 실패한다. 서보 프로파일로 계산한 완전 개폐 시간은
        #      스트로크 1484 tick (open 1083 → close -401)
        #      Profile Velocity 80 = 18.3 rev/min = 1251 tick/s
        #      Profile Acceleration 25 = 1.49 rev/s^2 → 가감속 각 0.20s(128 tick)
        #      → 2*0.20 + (1484-256)/1251 = **1.39 s**
        #    물체에 닿으면 감속해 더 걸리므로 여유를 둬 2.5 로 잡는다.
        #
        # 이 값은 gripper_open_tick/gripper_close_tick 이나 브릿지의 PROFILE_VELOCITY/
        # PROFILE_ACCELERATION 을 바꾸면 같이 다시 계산해야 한다.
        "gripper_action_time": 2.5,
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
