#!/usr/bin/env python3
"""팔 관절의 안전 가동범위 — 단일 출처 (2026-08-07 신설).

## 왜 이 모듈이 필요한가

`moveit_dynamixel_bridge` 의 `rad_to_tick` 은 그동안 **서보 tick 범위로만** clamp 했다
(0~4095, 다회전축은 ±256회전). 그건 "서보가 표현할 수 있는 값" 이지 "관절이 부딪히지
않는 범위" 가 아니다 — IK 가 엉뚱한 각도를 내면 그대로 서보로 나가 구조물을 때린다.

URDF 를 쓰면 될 것 같지만 못 쓴다: `arm_joint_2/3` 의 URDF 리밋은 `0~π` 자동생성
placeholder 이고(CAD 미반영), `arm_joint_1/5` 는 `continuous` 라 리밋 자체가 없다.
실제로 믿을 수 있는 건 CAD 실측이 반영된 `arm_joint_4` 뿐이다.

그래서 **실측 가동범위**를 이 모듈에 모아 단일 출처로 둔다.

## 도메인 주의 (제일 헷갈리는 지점)

여기 값은 전부 **관절 각도(rad)** 다 — `moveit_dynamixel_bridge.JOINT_CONFIG` 의
`center`/`gear_ratio` 로 환산된 도메인이고, MoveIt/`arm_fsm` 이 쓰는 도메인과 같다.

`teleop_core_node.py` 의 `DEFAULT_MIN_RADS`/`DEFAULT_MAX_RADS` 는 **서보축 각도**라
숫자가 전혀 다르다. **두 파일 값을 서로 복사하지 말 것.**

⚠️ `center` 나 `gear_ratio` 가 바뀌면(=팔 재조립, 서보 뿔 재장착, 기어비 재측정)
   여기 값은 **전부 무효**다. 영점 재측정 후 이 범위도 다시 재야 한다.

## confidence 필드

- `measured`   — 이 캘리브 도메인에서 양쪽 하드스톱까지 직접 재서 넣은 값. 신뢰 가능.
- `derived`    — 2026-08-02 서보 도메인 실측을 2026-08-07 캘리브로 **환산**한 값.
                 원 측정이 다른 조립 상태였을 수 있어 `measured` 보다 약하다.
- `provisional`— 실측이 없거나 모순이라 **보수적으로 좁게 박아둔** 값. 실기에서
                 "왜 조금밖에 안 움직이지?" 싶으면 이 표시부터 확인할 것.

새로 재려면: `scripts/measure_joint_limits.py` (run_calib.sh limits)
"""

#: 실측이 없는 축에 쓰는 보수적 기본 폭 [rad] (≈±8.6°).
#: 안 움직이는 것보다 조금 움직이는 게 낫지만, 하드스톱 위치를 모르는 상태에서
#: 넓게 열어두면 그대로 들이받는다 — 좁게 두고 실측으로 넓히는 방향이 맞다.
PROVISIONAL_HALF_RANGE = 0.15

# ⚠️ **하한 0.0 의 의미** (arm_joint_2/3/5)
#
# 2026-08-07 실측은 사용자가 **의도적으로 하드스톱 안쪽에서 멈춘** 보수적 범위다
# (첫 구동이라 여유를 크게 뒀다 — 하드스톱 실측이 아니다). 그 결과 세 축에서 측정
# 하한이 home(0.0) 위로 올라와 home 이 범위 밖이 됐다.
#
# 그대로 두면 **파워트레인 계약이 깨진다**: `arm_fsm` 의 stow 자세가 all-zero
# (`stow_joint_positions=[0,0,0,0]`)인데, 리밋이 0 을 배제하면 stow 명령이 clamp 되고
# → `_near_stow_posture()` 의 허용오차(`stow_pos_tol_rad`=0.1rad)를 못 맞추고
# → `STOWED_LOCKED` 를 영영 발행 못 하고
# → 파워트레인이 주행 허가를 못 받아 **차가 출발하지 못한다.**
#
# home 은 영점 캘리브 때 팔이 실제로 그 자세에 서 있었으므로 **도달 가능이 확인된
# 지점**이다. 그래서 상한(사용자의 보수적 값)은 그대로 두고 하한만 0.0 까지 넓혔다.
# 넓힌 방향은 "이미 가 봤던 곳"이라 새로운 위험을 만들지 않는다.
JOINT_LIMITS = {
    "arm_joint_2": {
        "lower": 0.0,
        "upper": +2.2559,
        "confidence": "measured",
        "source": "2026-08-07 실측(보수적, 하드스톱 아님). 하한은 측정값 +0.1366 → "
                  "home 포함 위해 0.0 으로 확장",
    },
    "arm_joint_3": {
        "lower": 0.0,
        "upper": +2.0341,
        "confidence": "measured",
        "source": "2026-08-07 실측(보수적, 하드스톱 아님). 하한은 측정값 +0.1610 → "
                  "home 포함 위해 0.0 으로 확장",
    },
    # 이 축만 측정 범위가 home 을 이미 포함해서 손대지 않았다.
    "arm_joint_4": {
        "lower": -0.6400,
        "upper": +0.7352,
        "confidence": "measured",
        "source": "2026-08-07 실측(보수적, 하드스톱 아님, span 82.8°)",
    },
    "arm_joint_5": {
        "lower": 0.0,
        "upper": +3.1588,
        "confidence": "measured",
        "source": "2026-08-07 실측(보수적, 하드스톱 아님). 하한은 측정값 +0.0349 → "
                  "home 포함 위해 0.0 으로 확장",
    },
}


def get_limits(joint_name):
    """(lower, upper) 반환. 등록되지 않은 관절이면 None."""
    entry = JOINT_LIMITS.get(joint_name)
    if entry is None:
        return None
    return entry["lower"], entry["upper"]


def clamp(joint_name, rad):
    """관절각을 안전 범위로 제한. (clamped_rad, was_clamped) 반환.

    등록되지 않은 관절은 통과시킨다 — 이 모듈이 모르는 축까지 막아버리면
    새 축을 배선할 때 원인 모를 정지가 난다(호출부가 경고를 찍는다).
    """
    limits = get_limits(joint_name)
    if limits is None:
        return rad, False
    lower, upper = limits
    clamped = max(lower, min(upper, rad))
    return clamped, clamped != rad


def provisional_joints():
    """실측이 안 된(보수적으로 좁혀둔) 관절 목록 — 기동 시 경고용."""
    return [n for n, e in JOINT_LIMITS.items() if e["confidence"] == "provisional"]
