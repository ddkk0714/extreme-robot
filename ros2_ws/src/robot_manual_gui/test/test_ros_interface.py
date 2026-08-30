"""Static contract tests for the GUI ROS frontend."""

from pathlib import Path
import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


ROOT = Path(__file__).parents[1] / 'robot_manual_gui'


def test_gui_never_imports_dynamixel_sdk():
    source = ''.join(path.read_text(encoding='utf-8')
                     for path in ROOT.glob('*.py'))
    assert 'dynamixel_sdk' not in source
    assert 'write1Byte' not in source
    assert 'write2Byte' not in source
    assert 'write4Byte' not in source


def test_gui_uses_existing_control_interfaces():
    source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    for interface in (
            '/arm_controller/joint_trajectory',
            '/gripper_controller/follow_joint_trajectory',
            '/cleaning/enable', '/tool/emergency_stop', '/tool/detached'):
        assert interface in source


def test_mode_status_does_not_overwrite_pending_operator_request():
    source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert 'self.mode_combo.setCurrentText(mode)' not in source


def test_end_effector_scope_blocks_arm_publish_path():
    ros_source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window_source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "self.control_scope == 'END_EFFECTOR_ONLY'" in ros_source
    assert 'manual and not end_effector_only' in window_source
    assert 'CONTROL / TEST SCOPE:' in window_source


def _window(scope):
    from PyQt5.QtWidgets import QApplication
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals

    app = QApplication.instance() or QApplication([])
    goals = []
    node = SimpleNamespace(
        control_scope=scope, selected_tool='dual_motor_gripper',
        positions={}, efforts={}, gripper_busy=False,
        request_mode=lambda _mode: None, jog_arm=lambda *_args: None,
        command_arm=lambda *_args: None,
        command_gripper=lambda position: (goals.append(position) or True),
        stop_gripper=lambda: None, command_cleaner=lambda *_args: None,
        emergency_stop=lambda: None, tool_detached=lambda: None)
    profile = {
        'calibrated': True, 'actuator_ids': [3, 4],
        'open_position': 1.0, 'close_position': 0.0,
        'safe_min_tick': -526, 'safe_max_tick': 2384,
        'motor_endpoints': {
            3: {'open': 1056, 'close': -526},
            4: {'open': 2384, 'close': 839}}}
    return app, ManualMainWindow(node, GuiSignals(), profile, False), goals


def _ready_status(scope):
    return {
        'control_scope': scope, 'tool_type': 'dual_motor_gripper',
        'profile_valid': True, 'calibrated': True,
        'actuators_discovered': True, 'motion_allowed': True,
        'read_only': False, 'emergency_stop': False, 'tool_detached': False,
        'bridge_connected': True,
        'actuators': [
            {'id': 3, 'online': True, 'position': 265, 'effort': 10},
            {'id': 4, 'online': True, 'position': 1612, 'effort': 10}]}


def test_end_effector_scope_enables_only_tool_controls():
    _app, window, _goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    assert window.open_button.isEnabled()
    assert window.close_button.isEnabled()
    assert window.tool_stop.isEnabled()
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.close()


def test_full_robot_preserves_arm_feedback_gate():
    _app, window, _goals = _window('FULL_ROBOT')
    window._update_tool_status(_ready_status('FULL_ROBOT'))
    window._update_mode('MANUAL')
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.seen_arm_joints.add('arm_joint_1')
    window._refresh_buttons()
    assert all(widget.isEnabled() for widget in window.arm_widgets['arm_joint_1'])
    assert not any(widget.isEnabled()
                   for widget in window.arm_widgets['arm_joint_2'])
    window.close()


def test_jog_interpolates_both_motors_and_busy_blocks_queue():
    _app, window, goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    window._jog_gripper(1)
    assert len(goals) == 1
    assert window.gripper_target_ticks == {3: 270, 4: 1617}
    window.gripper_busy = True
    window._jog_gripper(1)
    assert len(goals) == 1
    window.close()
