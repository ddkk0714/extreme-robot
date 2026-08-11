#!/usr/bin/env python3
"""그리퍼 모듈별 설정 (moveit_dynamixel_bridge / arm_fsm_node 공용).

새 그리퍼 모듈을 추가할 때는 두 노드를 건드리지 않고 이 dict에 항목만 추가하면 됨.
"""

GRIPPER_PRESETS = {
    "dual_motor_gripper": {
        # 랙피니언 그리퍼: XL430 2개(ID 3,4)가 기계적으로 맞물려 함께 움직인다.
        # 2026-08-10 Torque-OFF 수동 측정에서 두 축 모두 close=tick 감소,
        # open=tick 증가, 스트로크 비율 약 1.02:1로 확인됐다. 절대 영점은 약
        # 1330 tick 다르므로 동일 raw goal을 쓰면 두 위치제어 루프가 서로 힘을
        # 겨룬다. 반드시 아래 모터별 endpoint를 같은 논리 ratio로 보간한다.
        #
        # URDF(2026-07-16 랙피니언 export 이식) 기준 구동 조인트는 gripper_left_pinion_joint
        # (revolute, rad) 하나뿐이고 나머지 3개(우 피니언·좌우 랙)는 <mimic> 으로 종속된다.
        # 부호 규약: 0.0 = 완전 닫힘, 양수로 갈수록 열림, 상한 1.9444 rad.
        "gripper_joints": ["gripper_left_pinion_joint"],
        "gripper_ids": [3, 4],
        # TEMPORARY / MEASURED CANDIDATE: 손으로 조금 연 닫힘/열림 위치이며
        # 기계적 hard endpoint가 아니다. command_calibrated=False를 유지한다.
        "motor_endpoints": {
            3: {"open": 1056, "close": -526},
            4: {"open": 2384, "close": 839},
        },
        "required_operating_modes": {3: 4, 4: 3},
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
        # 대표 논리 joint의 역변환은 ID3 endpoint를 사용한다.
        "gripper_open_tick": 1056,
        "gripper_close_tick": -526,
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
        # 파지 시 서보가 낼 수 있는 최대 토크 상한 (Goal PWM, 주소 100). 0~885(=PWM Limit).
        #
        # ⚠️ 2026-08-09 실기: 이 상한이 없으면(기본 885=100%) **파지 4~5초 뒤 Overload 로
        #    토크가 끊긴다.** FSM 은 `gripper_close`=0.0 rad(완전 닫힘)을 명령하는데 물체가
        #    중간에서 막으므로 서보는 목표에 영영 도달하지 못하고 최대 전류로 계속 민다.
        #    실측: 물체를 문 순간 effort 773(≈최대의 77%) → 3.5초 유지 → Hardware Error
        #    0x20(Overload) 래치 + 토크 차단. 그 뒤엔 REBOOT 전까지 응답하지 않는다.
        #    FSM 은 gripper_action_time(2.5s) 시점엔 "파지 성공"으로 보므로, 들어올리다
        #    떨어뜨리는 형태로 터진다.
        #
        # 정공법은 Current-based Position Control(모드 5)이지만 **XL430-W250 은 전류 센싱이
        # 없어 지원하지 않는다**(모드 5 쓰기가 조용히 무시되고 Goal Current/Current Limit
        # 레지스터도 없음 — 2026-08-09 실기 확인). XM/XH 로 교체하면 그쪽이 낫다.
        #
        # 2026-08-09 실기 스윕(박스를 문 채 유지 시간 측정):
        #   PWM 885(무제한) → 유지 effort 773 → 3.5초 만에 트립
        #   PWM 400         → 유지 effort 452 → 17초 만에 트립
        #   PWM 280         → 유지 effort 317 → **40초+ 트립 없음** (position 변동 0)
        # XL430 의 Overload 는 부하를 시간에 대해 누적 판정하므로 "조금 낮추면 조금 오래
        # 버티는" 게 아니라 어느 선 아래로 내려가야 무한정 버틴다. 317 은 파지 임계
        # (grasp_effort_thresh=250)보다 위여서 FSM 이 파지로 인식하면서 트립은 피하는 구간.
        #
        # 조정 방향: 물체가 미끄러지면 올리고, 트립이 재발하면 내린다. 올릴 때는 **유지
        # 시간을 반드시 30초 이상 확인할 것** — 400 도 17초까지는 멀쩡해 보였다.
        "gripper_goal_pwm": 280,
        "command_calibrated": True,
        "observed_operating_modes": {3: 4, 4: 3},
        # 단일축 preset과의 하위 호환용. dual 경로는 위 per-ID map만 사용한다.
        "observed_operating_mode": -1,
        "required_operating_mode": -1,
        "kind": "gripper",
        "allowed_mission": "PICK_PLACE",
        "arm_tip_link": "link_043",
        "tip_link": "link_043",
        "profile_acceleration": 25,
        "profile_velocity": 80,
        "max_abs_current": 300,
        "stall_timeout": 2.0,
        "motion_timeout": 10.0,
        "goal_tolerance_ticks": 10,
    },
    "single_motor_gripper": {
        # HW-8 단일 서보 그리퍼의 복원 URDF/MoveIt 모델에서 실제 구동축은
        # gripper_drive_joint(parent=link_051, child=link_055)였고 나머지 조 관절은
        # 이 축을 mimic 했다. 현재 활성 랙피니언 URDF에는 이 단일모터 형상이 없지만,
        # 잘못된 gripper_left_pinion_joint/end_effector_joint로 대체하지 않고 확인된
        # 논리 축 이름을 보존한다. 형상 복원 전에도 FSM/preset 선택 계약은 검증 가능하다.
        "gripper_joints": ["gripper_drive_joint"],
        "gripper_ids": [5],
        # 2026-08-11 ID5 Mode 3, Profile Acceleration=5/Velocity=20 실측.
        # 52↔615 명령을 3-cycle씩 두 번 반복했을 때 매 cycle 실제 범위가 80↔588,
        # stroke=508 tick(44.65°)으로 동일했고 Hardware Error/통신 오류가 없었다.
        # 이 tick endpoint만 확정되었으며 위치-rad 매핑, operating mode/PWM/effort
        # 임계값은 아직 sentinel이다. 파지 캘리브 전 command_calibrated=False 유지.
        "gripper_open_tick": 80,
        "gripper_close_tick": 588,
        "gripper_open_rad": 0.0,
        "gripper_close_rad": 0.0,
        "grasp_effort_thresh": 1.0e9,
        "drop_effort_thresh": -1.0,
        "gripper_action_time": 0.0,
        "gripper_goal_pwm": 0,
        "command_calibrated": False,
        "observed_operating_mode": -1,
        "required_operating_mode": -1,
        "kind": "gripper",
        "allowed_mission": "PICK_PLACE",
        "arm_tip_link": "link_051",
        "tip_link": "single_gripper_grasp_frame",
        "profile_acceleration": 0,
        "profile_velocity": 0,
        "max_abs_current": 0,
        "stall_timeout": 0.0,
        "motion_timeout": 0.0,
        "goal_tolerance_ticks": 0,
    },
    "rotary_id5": {
        # PICK_PLACE에서는 선택되지 않는다. 기존 rotary workflow 보존용 별도 preset.
        "gripper_joints": ["end_effector_joint"],
        "gripper_ids": [5],
        "gripper_open_tick": 2446,
        "gripper_close_tick": 3186,
        "gripper_open_rad": 1.0471975511966,
        "gripper_close_rad": -0.872664625997165,
        "grasp_effort_thresh": 80.0,
        "drop_effort_thresh": 20.0,
        "gripper_action_time": 1.0,
        "command_calibrated": True,
        "observed_operating_mode": 3,
        "required_operating_mode": 3,
        "kind": "rotary",
        "allowed_mission": "ROTARY_TOOL",
        "arm_tip_link": "link_043",
        "tip_link": "link_043",
        "profile_acceleration": 5,
        "profile_velocity": 20,
        "max_abs_current": 100,
        "stall_timeout": 2.0,
        "motion_timeout": 10.0,
        "goal_tolerance_ticks": 10,
    },
}

DEFAULT_GRIPPER = "dual_motor_gripper"


def get_preset(gripper_type, logger=None):
    preset = GRIPPER_PRESETS.get(gripper_type)
    if preset is None:
        known = ", ".join(sorted(GRIPPER_PRESETS))
        raise ValueError(
            f"Unknown end_effector_preset '{gripper_type}'; expected one of: {known}"
        )
    return preset
