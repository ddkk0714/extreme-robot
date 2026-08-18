"""`/arm/teleop_cmd` 화이트리스트.

`teleop_core.on_cmd` 는 모르는 명령을 경고만 찍고 넘긴다 — 오타가 조용히 삼켜지는
게 제어 경로에서 가장 나쁜 실패라, GUI 쪽에서 먼저 거절하는 것을 여기서 고정한다.
"""

from robot_arm_gui.teleop_vocab import validate


def test_no_arg_commands_pass():
    for cmd in ('stop', 'resume', 'home', 'poses', 'freedrive',
                'freedrive_cancel', 'calib_start', 'calib_mark', 'calib_cancel'):
        assert validate(cmd) == (cmd, None)


def test_action_is_lowercased_but_name_is_not():
    """teleop_core 와 같은 정규화 규칙 — 자세 이름의 대소문자는 보존한다."""
    assert validate('STOP') == ('stop', None)
    assert validate('goto Pick_Ready') == ('goto Pick_Ready', None)


def test_no_arg_command_rejects_an_argument():
    cmd, reason = validate('stop now')
    assert cmd is None and '인자를' in reason


def test_name_commands_require_a_name():
    for action in ('save', 'goto', 'reboot'):
        cmd, reason = validate(action)
        assert cmd is None and '이름이 필요' in reason


def test_name_with_comma_is_rejected():
    """쉼표는 /arm/teleop_poses 의 구분자다 — 이름에 들어가면 목록이 깨진다."""
    cmd, reason = validate('save a,b')
    assert cmd is None and '쓸 수 없는 문자' in reason


def test_name_with_space_is_rejected():
    cmd, reason = validate('save my pose')
    assert cmd is None


def test_reboot_all_is_allowed():
    assert validate('reboot all') == ('reboot all', None)


def test_threshold_commands_accept_on_off_and_integers():
    assert validate('spike on') == ('spike on', None)
    assert validate('trip off') == ('trip off', None)
    assert validate('spike 350') == ('spike 350', None)


def test_threshold_rejects_garbage():
    cmd, reason = validate('spike maybe')
    assert cmd is None and 'on|off' in reason


def test_limit_accepts_only_on_off():
    assert validate('limit off') == ('limit off', None)
    cmd, _ = validate('limit 5')
    assert cmd is None


def test_unknown_command_is_rejected():
    cmd, reason = validate('selfdestruct')
    assert cmd is None and '알 수 없는 명령' in reason


def test_empty_and_non_string_are_rejected():
    assert validate('')[0] is None
    assert validate('   ')[0] is None
    assert validate(None)[0] is None
    assert validate(42)[0] is None


def test_absurdly_long_command_is_rejected():
    cmd, reason = validate('goto ' + 'x' * 500)
    assert cmd is None
