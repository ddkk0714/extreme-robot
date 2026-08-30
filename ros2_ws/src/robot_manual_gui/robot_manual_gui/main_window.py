"""PyQt5 widgets for manual robot validation."""

import time

from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtWidgets import (
    QAbstractSpinBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QLineEdit, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from robot_manual_gui.ros_interface import ARM_JOINTS
from dynamixel_control.tool_manager import ToolManager


TRUE_STYLE = 'color: #0b7a25; font-weight: bold;'
FALSE_STYLE = 'color: #b00020; font-weight: bold;'
ESTOP_STYLE = 'background: #b00020; color: white; font-size: 20px; font-weight: bold;'


class ManualMainWindow(QMainWindow):
    """Hardware-test dashboard backed exclusively by ROS interfaces."""

    def __init__(self, ros_node, signals, profile, mock_mode=False):
        super().__init__()
        self.node = ros_node
        self.signals = signals
        self.profile = profile
        self.mock_mode = mock_mode
        self.tool_status = {}
        self.fsm_state = 'UNKNOWN'
        self.control_mode = 'FSM'
        self.last_status_time = 0.0
        self.processes = []
        self.joint_rows = {}
        self.seen_arm_joints = set()
        self.arm_widgets = {}
        self.gripper_busy = False
        self.gripper_target_ticks = {}
        self.spur_torque_enabled = False
        self.spur_torque_state = 'UNKNOWN'
        self.spur_endpoints = {}
        self.spur_zero_tick = None
        self.dual_calibration_buttons = []
        self.dual_calibration_step = None
        self.dual_calibration_state = None
        self.dual_start_calibration = None
        self.dual_capture_open = None
        self.dual_capture_close = None
        self.dual_validate_calibration = None
        self.dual_save_calibration = None
        self.dual_capture_label = None
        # External spur gears reverse rotation.  This is deliberately shown in
        # the GUI instead of being hidden in a raw-tick jog control.
        self.spur_output_direction = -1
        self.spur_gear_ratio = 1.0
        self.temporary_jog_safe_min = getattr(
            self.node, 'temporary_jog_safe_min', 2867)
        self.temporary_jog_safe_max = getattr(
            self.node, 'temporary_jog_safe_max', 3807)
        get_param = getattr(self.node, 'get_parameter', None)
        self.temporary_jog_mechanical_open = (
            get_param('temporary_jog_mechanical_open_tick').value
            if get_param else 2817)
        self.temporary_jog_mechanical_close = (
            get_param('temporary_jog_mechanical_close_tick').value
            if get_param else 3857)
        self.setWindowTitle('Extreme Robot Manual Hardware Validation')
        self.resize(1180, 850)
        self._build_ui()
        self._connect_signals()
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh_connection)
        self.watchdog.start(500)

    def _build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)

        self.scope_banner = QLabel(f'CONTROL / TEST SCOPE: {self.node.control_scope}')
        self.scope_banner.setAlignment(Qt.AlignCenter)
        self.scope_banner.setStyleSheet(
            'font-size: 22px; font-weight: bold; padding: 8px; '
            'background: #ffe08a; color: #202020;')
        outer.addWidget(self.scope_banner)

        safety = QHBoxLayout()
        self.estop = QPushButton('EMERGENCY STOP')
        self.estop.setMinimumHeight(62)
        self.estop.setStyleSheet(ESTOP_STYLE)
        self.estop.clicked.connect(self._estop)
        self.detach = QPushButton('TOOL DETACHED')
        self.detach.clicked.connect(self._detach)
        self.reset = QPushButton('RESET E-STOP (restart required)')
        self.reset.setEnabled(False)
        self.estop_state = QLabel('E-STOP: FALSE')
        self.estop_state.setStyleSheet(TRUE_STYLE)
        safety.addWidget(self.estop, 3)
        safety.addWidget(self.detach)
        safety.addWidget(self.reset)
        safety.addWidget(self.estop_state)
        outer.addLayout(safety)

        columns = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()
        left.addWidget(self._status_group())
        left.addWidget(self._arm_group())
        right.addWidget(self._tool_selection_group())
        right.addWidget(self._tool_control_group())
        columns.addLayout(left, 3)
        columns.addLayout(right, 2)
        outer.addLayout(columns)

        self.diag = QTableWidget(0, 5)
        self.diag.setHorizontalHeaderLabels(
            ['ID', 'Joint', 'Position', 'Current/Load', 'Online'])
        outer.addWidget(self.diag)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(120)
        outer.addWidget(self.log)
        self.setCentralWidget(root)

    def _status_group(self):
        box = QGroupBox('Connection / Status')
        form = QFormLayout(box)
        self.status_labels = {}
        for key, title in (
                ('connection', 'Bridge connection'),
                ('u2d2', 'U2D2 / serial'), ('tool_type', 'Tool type'),
                ('profile_valid', 'Profile valid'),
                ('actuators_discovered', 'Actuators discovered'),
                ('motion_allowed', 'Motion allowed'), ('fsm', 'FSM state'),
                ('arm_status', 'Arm contract state'), ('mode', 'Control mode'),
                ('contact', 'Contact sensor')):
            label = QLabel('UNKNOWN')
            self.status_labels[key] = label
            form.addRow(title, label)
        return box

    def _arm_group(self):
        box = QGroupBox('Arm Manual Control')
        layout = QGridLayout(box)
        layout.addWidget(QLabel('Joint'), 0, 0)
        layout.addWidget(QLabel('Current rad'), 0, 1)
        layout.addWidget(QLabel('Jog'), 0, 2, 1, 2)
        layout.addWidget(QLabel('Target rad'), 0, 4)
        self.arm_buttons = []
        self.arm_position_labels = {}
        self.arm_targets = {}
        for row, joint in enumerate(ARM_JOINTS, 1):
            label = QLabel('0.0000')
            minus = QPushButton('−')
            plus = QPushButton('+')
            target = QDoubleSpinBox()
            target.setRange(-6.283, 6.283)
            target.setDecimals(4)
            send = QPushButton('GO')
            minus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, -1))
            plus.clicked.connect(
                lambda _checked=False, name=joint: self._jog(name, 1))
            send.clicked.connect(
                lambda _checked=False, name=joint: self._arm_target(name))
            layout.addWidget(QLabel(joint), row, 0)
            layout.addWidget(label, row, 1)
            layout.addWidget(minus, row, 2)
            layout.addWidget(plus, row, 3)
            layout.addWidget(target, row, 4)
            layout.addWidget(send, row, 5)
            self.arm_position_labels[joint] = label
            self.arm_targets[joint] = target
            self.arm_buttons.extend([minus, plus, target, send])
            self.arm_widgets[joint] = [minus, plus, target, send]
        self.jog_step = QComboBox()
        self.jog_step.addItems(['0.5', '1.0', '5.0'])
        layout.addWidget(QLabel('Jog step (deg)'), 6, 0)
        layout.addWidget(self.jog_step, 6, 1)
        return box

    def _tool_selection_group(self):
        box = QGroupBox('Tool Selection / Ownership')
        form = QFormLayout(box)
        self.tool_combo = QComboBox()
        self.tool_combo.addItems([
            'dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'])
        self.tool_combo.setCurrentText(self.node.selected_tool)
        request = QPushButton('REQUEST TOOL CHANGE')
        request.clicked.connect(self._request_tool_change)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['FSM', 'MANUAL'])
        mode_request = QPushButton('REQUEST MODE')
        mode_request.clicked.connect(self._request_mode)
        form.addRow('Selected tool', self.tool_combo)
        form.addRow('', request)
        form.addRow('Ownership', self.mode_combo)
        form.addRow('', mode_request)
        return box

    def _tool_control_group(self):
        box = QGroupBox('End Effector')
        layout = QVBoxLayout(box)
        self.profile_text = QLabel(self._profile_summary())
        self.profile_text.setWordWrap(True)
        layout.addWidget(self.profile_text)
        row = QHBoxLayout()
        self.open_button = QPushButton('OPEN')
        self.close_button = QPushButton('CLOSE')
        self.tool_stop = QPushButton('STOP')
        self.open_button.clicked.connect(lambda: self._command_tool('OPEN'))
        self.close_button.clicked.connect(lambda: self._command_tool('CLOSE'))
        self.tool_stop.clicked.connect(self._stop_tool)
        row.addWidget(self.open_button)
        row.addWidget(self.close_button)
        row.addWidget(self.tool_stop)
        layout.addLayout(row)
        self.spur_enable = QPushButton('ENABLE ID5')
        self.spur_disable = QPushButton('DISABLE ID5')
        self.spur_enable.clicked.connect(self._enable_spur_motor)
        self.spur_disable.clicked.connect(self._disable_spur_motor)
        enable_row = QHBoxLayout()
        enable_row.addWidget(self.spur_enable)
        enable_row.addWidget(self.spur_disable)
        self.dual_enable = QPushButton('ENABLE ID3/ID4')
        self.dual_disable = QPushButton('DISABLE ID3/ID4')
        self.dual_enable.clicked.connect(self._enable_dual_motors)
        self.dual_disable.clicked.connect(self._disable_dual_motors)
        enable_row.addWidget(self.dual_enable)
        enable_row.addWidget(self.dual_disable)
        layout.addLayout(enable_row)
        self.dual_recovery_buttons = []
        if self.node.selected_tool == 'dual_motor_gripper':
            recovery = QGroupBox('MANUAL DUAL MOTOR RECOVERY (one click only)')
            recovery_layout = QGridLayout(recovery)
            for row, dxl_id in enumerate((3, 4)):
                recovery_layout.addWidget(QLabel(f'ID{dxl_id}'), row, 0)
                for column, delta in enumerate((-0.5, 0.5), 1):
                    button = QPushButton(f'ID{dxl_id} {delta:+.1f}°')
                    button.setAutoRepeat(False)
                    button.clicked.connect(
                        lambda _checked=False, motor=dxl_id, step=delta:
                        self._manual_dual_recovery_jog(motor, step))
                    recovery_layout.addWidget(button, row, column)
                    self.dual_recovery_buttons.append((dxl_id, button))
            layout.addWidget(recovery)
            calibration = QGroupBox('DUAL ENDPOINT CALIBRATION')
            calibration_layout = QGridLayout(calibration)
            self.dual_start_calibration = QPushButton('START DUAL CALIBRATION')
            self.dual_start_calibration.clicked.connect(self._start_dual_calibration)
            self.dual_calibration_state = QLabel('RECALIBRATION_REQUIRED')
            calibration_layout.addWidget(self.dual_start_calibration, 0, 0, 1, 2)
            calibration_layout.addWidget(self.dual_calibration_state, 0, 2, 1, 2)
            calibration_layout.addWidget(QLabel('Calibration step (motor degree)'), 1, 0, 1, 2)
            self.dual_calibration_step = QComboBox()
            self.dual_calibration_step.addItems(['0.5', '1', '2', '5'])
            calibration_layout.addWidget(self.dual_calibration_step, 1, 2, 1, 2)
            for row, dxl_id in enumerate((3, 4), 2):
                calibration_layout.addWidget(QLabel(f'ID{dxl_id}'), row, 0)
                for column, direction in enumerate((-1.0, 1.0), 1):
                    button = QPushButton(f'ID{dxl_id} {"−" if direction < 0 else "+"} step')
                    button.setAutoRepeat(False)
                    button.clicked.connect(
                        lambda _checked=False, motor=dxl_id, sign=direction:
                        self._jog_dual_calibration_motor(motor, sign))
                    calibration_layout.addWidget(button, row, column)
                    self.dual_calibration_buttons.append((dxl_id, button))
            self.dual_capture_open = QPushButton('CAPTURE OPEN')
            self.dual_capture_close = QPushButton('CAPTURE CLOSE')
            self.dual_capture_open.clicked.connect(
                lambda: self._command_dual_calibration('capture_open'))
            self.dual_capture_close.clicked.connect(
                lambda: self._command_dual_calibration('capture_close'))
            calibration_layout.addWidget(self.dual_capture_open, 4, 0, 1, 2)
            calibration_layout.addWidget(self.dual_capture_close, 4, 2, 1, 2)
            self.dual_capture_label = QLabel('Captured OPEN: — | CLOSE: —')
            calibration_layout.addWidget(self.dual_capture_label, 5, 0, 1, 4)
            self.dual_validate_calibration = QPushButton('VALIDATE DUAL CALIBRATION')
            self.dual_save_calibration = QPushButton('SAVE DUAL CALIBRATION')
            self.dual_validate_calibration.clicked.connect(
                lambda: self._command_dual_calibration('validate'))
            self.dual_save_calibration.clicked.connect(
                lambda: self._command_dual_calibration('save'))
            calibration_layout.addWidget(self.dual_validate_calibration, 6, 0, 1, 2)
            calibration_layout.addWidget(self.dual_save_calibration, 6, 2, 1, 2)
            calibration_layout.addWidget(QLabel(
                'Captured endpoint pairs become the only OPEN/CLOSE targets. '
                'Capture itself performs reads only.'), 7, 0, 1, 4)
            bypass = QLabel(
                'CALIBRATION JOG: spread protection bypassed\n'
                'Normal OPEN/CLOSE and legacy gripper JOG remain blocked until READY.')
            bypass.setStyleSheet(FALSE_STYLE)
            bypass.setWordWrap(True)
            calibration_layout.addWidget(bypass, 8, 0, 1, 4)
            layout.addWidget(calibration)
        jog = QGroupBox('GRIPPER JOG')
        jog_layout = QGridLayout(jog)
        self.spur_minus_5 = self.spur_zero = self.spur_plus_5 = None
        if self.node.selected_tool == 'spur_1motor_gripper':
            left_label, right_label = 'LEFT / −  (OPEN)', 'RIGHT / +  (CLOSE)'
        else:
            left_label, right_label = 'LEFT / −  (CLOSE)', 'RIGHT / +  (OPEN)'
        self.jog_close = QPushButton(left_label)
        self.jog_open = QPushButton(right_label)
        self.gripper_jog_step = QComboBox()
        self.gripper_jog_step.addItems(['5', '10', '25', '50'])
        self.gripper_busy_label = QLabel('READY')
        self.gripper_position_label = QLabel('Gripper position: UNKNOWN')
        self.gripper_feedback_label = QLabel('ID3: UNKNOWN\nID4: UNKNOWN')
        self.gripper_feedback_label.setWordWrap(True)
        shortcut = QLabel(
            'Shortcuts: Left=CLOSE jog, Right=OPEN jog, Space=STOP\n'
            '(disabled while editing a field; key auto-repeat ignored)')
        shortcut.setWordWrap(True)
        self.jog_close.clicked.connect(lambda: self._jog_gripper(-1))
        self.jog_open.clicked.connect(lambda: self._jog_gripper(1))
        jog_layout.addWidget(self.jog_close, 0, 0)
        jog_layout.addWidget(self.jog_open, 0, 1)
        jog_layout.addWidget(QLabel('Step (tick equivalent)'), 1, 0)
        jog_layout.addWidget(self.gripper_jog_step, 1, 1)
        jog_layout.addWidget(self.gripper_busy_label, 2, 0, 1, 2)
        jog_layout.addWidget(self.gripper_position_label, 3, 0, 1, 2)
        jog_layout.addWidget(self.gripper_feedback_label, 4, 0, 1, 2)
        shortcut_row = 5
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.spur_actual_state = QLabel('ID5: position=UNKNOWN torque=UNKNOWN load=UNKNOWN')
            jog_layout.addWidget(self.spur_actual_state, 5, 0, 1, 2)
            self.capture_open = QPushButton('SET CURRENT AS OPEN')
            self.capture_close = QPushButton('SET CURRENT AS CLOSE')
            self.capture_open.clicked.connect(lambda: self._capture_spur_endpoint('open'))
            self.capture_close.clicked.connect(lambda: self._capture_spur_endpoint('close'))
            jog_layout.addWidget(self.capture_open, 6, 0)
            jog_layout.addWidget(self.capture_close, 6, 1)
            self.captured_endpoints_label = QLabel('Captured OPEN: — | CLOSE: —')
            jog_layout.addWidget(self.captured_endpoints_label, 7, 0, 1, 2)
            self.validate_calibration = QPushButton('VALIDATE CALIBRATION')
            self.save_calibration = QPushButton('SAVE CALIBRATION')
            self.validate_calibration.clicked.connect(self._validate_spur_calibration)
            self.save_calibration.clicked.connect(self._save_spur_calibration)
            jog_layout.addWidget(self.validate_calibration, 8, 0)
            jog_layout.addWidget(self.save_calibration, 8, 1)
            self.motor_minus_half = QPushButton('MOTOR −0.5°')
            self.motor_plus_half = QPushButton('MOTOR +0.5°')
            self.motor_minus_one = QPushButton('MOTOR −1°')
            self.motor_plus_one = QPushButton('MOTOR +1°')
            for button, degrees in ((self.motor_minus_half, -0.5),
                                    (self.motor_plus_half, 0.5),
                                    (self.motor_minus_one, -1.0),
                                    (self.motor_plus_one, 1.0)):
                button.clicked.connect(
                    lambda _checked=False, delta=degrees: self._jog_spur_motor(delta))
            jog_layout.addWidget(self.motor_minus_half, 9, 0)
            jog_layout.addWidget(self.motor_plus_half, 9, 1)
            jog_layout.addWidget(self.motor_minus_one, 10, 0)
            jog_layout.addWidget(self.motor_plus_one, 10, 1)
            jog_layout.addWidget(QLabel(
                'Safety policy: captured OPEN/CLOSE are the command limits; '
                'no hidden endpoint or temporary range is used.'), 11, 0, 1, 2)
            self.spur_mapping = QLabel('Output mapping: waiting for ID5 feedback')
            self.spur_mapping.setWordWrap(True)
            jog_layout.addWidget(self.spur_mapping, 12, 0, 1, 2)
            self.spur_minus_5 = QPushButton('OUTPUT −5°')
            self.spur_zero = QPushButton('OUTPUT 0°')
            self.spur_plus_5 = QPushButton('OUTPUT +5°')
            self.spur_minus_5.clicked.connect(lambda: self._command_spur_output_deg(-5.0))
            self.spur_zero.clicked.connect(lambda: self._command_spur_output_deg(0.0))
            self.spur_plus_5.clicked.connect(lambda: self._command_spur_output_deg(5.0))
            jog_layout.addWidget(self.spur_minus_5, 13, 0)
            jog_layout.addWidget(self.spur_zero, 13, 1)
            jog_layout.addWidget(self.spur_plus_5, 14, 0, 1, 2)
            shortcut_row = 15
        jog_layout.addWidget(shortcut, shortcut_row, 0, 1, 2)
        layout.addWidget(jog)
        cleaner = QHBoxLayout()
        self.clean_start = QPushButton('CLEANER START')
        self.clean_stop = QPushButton('CLEANER STOP')
        self.clean_start.clicked.connect(lambda: self.node.command_cleaner(True))
        self.clean_stop.clicked.connect(lambda: self.node.command_cleaner(False))
        cleaner.addWidget(self.clean_start)
        cleaner.addWidget(self.clean_stop)
        layout.addLayout(cleaner)
        calibration = QHBoxLayout()
        self.read_diag = QPushButton('READ ONLY DIAGNOSTIC')
        self.start_cal = QPushButton('START CALIBRATION')
        self.read_diag.clicked.connect(self._read_only_diagnostic)
        self.start_cal.clicked.connect(self._start_calibration)
        calibration.addWidget(self.read_diag)
        calibration.addWidget(self.start_cal)
        layout.addLayout(calibration)
        return box

    def _profile_summary(self):
        keys = ('calibrated', 'actuator_ids', 'safe_min_tick', 'safe_max_tick',
                'open_tick', 'close_tick', 'profile_velocity',
                'profile_acceleration')
        return '\n'.join(f'{key}: {self.profile.get(key)}' for key in keys)

    def _connect_signals(self):
        self.signals.joint_states.connect(self._update_joints)
        self.signals.tool_status.connect(self._update_tool_status)
        self.signals.fsm_state.connect(self._update_fsm)
        self.signals.control_mode.connect(self._update_mode)
        self.signals.arm_status.connect(
            lambda value: self.status_labels['arm_status'].setText(value))
        self.signals.contact_status.connect(
            lambda value: self._set_bool(self.status_labels['contact'], value))
        self.signals.log.connect(self._append_log)
        self.signals.gripper_state.connect(self._update_gripper_state)

    def _set_bool(self, label, value):
        label.setText('TRUE' if value else 'FALSE')
        label.setStyleSheet(TRUE_STYLE if value else FALSE_STYLE)

    def _refresh_connection(self):
        connected = time.monotonic() - self.last_status_time < 1.5
        self._set_bool(self.status_labels['connection'], connected)
        if not connected:
            self._set_bool(self.status_labels['motion_allowed'], False)
        self._refresh_buttons()

    def _update_tool_status(self, status):
        self.tool_status = status
        self.last_status_time = time.monotonic()
        self.status_labels['tool_type'].setText(status.get('tool_type', 'UNKNOWN'))
        self._update_fsm(status.get('fsm_state', 'UNKNOWN'))
        self._set_bool(
            self.status_labels['u2d2'], bool(status.get('u2d2_connected')))
        for key in ('profile_valid', 'actuators_discovered', 'motion_allowed'):
            self._set_bool(self.status_labels[key], bool(status.get(key)))
        estop = bool(status.get('emergency_stop'))
        self.estop_state.setText(f'E-STOP: {str(estop).upper()}')
        self.estop_state.setStyleSheet(FALSE_STYLE if estop else TRUE_STYLE)
        self._refresh_buttons()
        self._rebuild_diagnostics(status.get('actuators', []))
        self._update_gripper_feedback()

    def _update_joints(self, values):
        for joint, sample in values.items():
            if joint in self.arm_position_labels and sample['position'] is not None:
                self.seen_arm_joints.add(joint)
                self.arm_position_labels[joint].setText(f'{sample["position"]:.4f}')
                self.arm_targets[joint].setValue(float(sample['position']))
        self._refresh_buttons()
        self._rebuild_diagnostics(self.tool_status.get('actuators', []), values)

    def _update_fsm(self, state):
        self.fsm_state = state
        self.status_labels['fsm'].setText(state)

    def _update_mode(self, mode):
        self.control_mode = mode
        self.status_labels['mode'].setText(mode)
        self._refresh_buttons()

    def _refresh_buttons(self):
        manual = self.control_mode == 'MANUAL'
        end_effector_only = self.node.control_scope == 'END_EFFECTOR_ONLY'
        for widget in self.arm_buttons:
            widget.setEnabled(manual and not end_effector_only)
        if not self.mock_mode:
            for joint, widgets in self.arm_widgets.items():
                for widget in widgets:
                    widget.setEnabled(
                        manual and not end_effector_only
                        and joint in self.seen_arm_joints)
        profile_ok = bool(self.tool_status.get('profile_valid'))
        motion = self._tool_motion_ready()
        gripper = self.node.selected_tool.endswith('gripper')
        spur = self.node.selected_tool == 'spur_1motor_gripper'
        dual = self.node.selected_tool == 'dual_motor_gripper'
        calibrated = bool(self.tool_status.get('calibrated')) or self.mock_mode
        captured = (set(self.spur_endpoints) == {'open', 'close'}
                    and self.spur_endpoints['open'] != self.spur_endpoints['close'])
        dual_calibration = self.tool_status.get('dual_calibration') or {}
        dual_ready = dual_calibration.get('state') == 'READY'
        preset_ready = (manual and gripper and profile_ok and motion
                        and calibrated and not self.gripper_busy
                        and (not spur or self.fsm_state == 'READY')
                        and (not dual or dual_ready))
        # Captures are only a candidate.  They never silently turn an
        # uncalibrated live profile into a normal-motion profile.
        self.open_button.setEnabled(preset_ready)
        self.close_button.setEnabled(preset_ready)
        self.spur_enable.setVisible(spur)
        self.spur_disable.setVisible(spur)
        self.dual_enable.setVisible(dual)
        self.dual_disable.setVisible(dual)
        calibration = self.tool_status.get('calibration') or {}
        self.spur_enable.setEnabled(
            spur and manual and calibration.get('active', False)
            and self._tool_enable_ready() and not calibration.get('enabled', False))
        self.spur_disable.setEnabled(
            spur and calibration.get('active', False)
            and self.spur_torque_state == 'ON')
        dual_samples = self._gripper_samples()
        dual_online = all(dual_samples.get(dxl_id, {}).get('online')
                          for dxl_id in (3, 4))
        dual_healthy = all(dual_samples.get(dxl_id, {}).get('hardware_error') == 0
                           for dxl_id in (3, 4))
        dual_torque_on = all(dual_samples.get(dxl_id, {}).get('torque_state') == 'ON'
                             for dxl_id in (3, 4))
        self.tool_stop.setEnabled(
            (spur and not bool(self.tool_status.get('read_only'))
             and bool(self.tool_status.get('online')))
            or (dual and dual_online
                and not bool(self.tool_status.get('read_only'))))
        self.dual_enable.setEnabled(
            dual and manual and dual_online and dual_healthy
            and not dual_torque_on and not bool(self.tool_status.get('read_only')))
        self.dual_disable.setEnabled(
            dual and dual_online and not bool(self.tool_status.get('read_only')))
        recovery_base = (
            dual and manual and self.node.control_scope == 'END_EFFECTOR_ONLY'
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached')))
        for dxl_id, button in self.dual_recovery_buttons:
            sample = dual_samples.get(dxl_id, {})
            button.setEnabled(
                recovery_base and sample.get('online')
                and sample.get('hardware_error') == 0
                and sample.get('torque_state') == 'ON')
        dual_calibration_active = bool(dual_calibration.get('active'))
        dual_calibration_capture_ready = (
            dual and manual and end_effector_only
            and not bool(self.tool_status.get('read_only'))
            and not bool(self.tool_status.get('emergency_stop'))
            and not bool(self.tool_status.get('tool_detached'))
            and dual_online and dual_healthy)
        dual_calibration_jog_ready = (
            dual_calibration_capture_ready and dual_torque_on)
        if self.dual_calibration_state is not None:
            self.dual_calibration_state.setText(
                dual_calibration.get('state', 'RECALIBRATION_REQUIRED'))
        if self.dual_capture_label is not None:
            captures = dual_calibration.get('captures') or {}
            self.dual_capture_label.setText(
                f'Captured OPEN: {captures.get("open", "—")} | '
                f'CLOSE: {captures.get("close", "—")}')
        if self.dual_start_calibration is not None:
            self.dual_start_calibration.setEnabled(
                dual and manual and end_effector_only
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached'))
                and not dual_calibration_active)
        for _dxl_id, button in self.dual_calibration_buttons:
            button.setEnabled(dual_calibration_active and dual_calibration_jog_ready)
        if self.dual_calibration_step is not None:
            self.dual_calibration_step.setEnabled(
                dual_calibration_active and dual_calibration_jog_ready)
        if self.dual_capture_open is not None:
            self.dual_capture_open.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready)
            self.dual_capture_close.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready)
            captured_pairs = dual_calibration.get('captures') or {}
            both_pairs = set(captured_pairs) == {'open', 'close'}
            self.dual_validate_calibration.setEnabled(
                dual_calibration_active and dual_calibration_capture_ready and both_pairs)
            self.dual_save_calibration.setEnabled(
                dual_calibration_active and bool(dual_calibration.get('validated')))
        jog_ready = (manual and not self.gripper_busy
                     and self.node.control_scope == 'END_EFFECTOR_ONLY'
                     and self.node.selected_tool in (
                         'dual_motor_gripper', 'spur_1motor_gripper')
                     and self._tool_motion_ready()
                     and self._gripper_positions_synchronized()
                     and (not dual or dual_ready))
        # When the measured position is outside the temporary range, expose
        # only the inward recovery direction.  This prevents a disabled
        # direction from being retried by either a click or a key shortcut.
        spur_open_allowed = True   # LEFT / '-' decreases ticks (opens)
        spur_close_allowed = True  # RIGHT / '+' increases ticks (closes)
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            current = sample.get('position')
            if current is not None:
                if current > self.temporary_jog_safe_max:
                    spur_close_allowed = False
                elif current < self.temporary_jog_safe_min:
                    spur_open_allowed = False
        self.jog_close.setEnabled(not spur and jog_ready and spur_open_allowed)
        self.jog_open.setEnabled(not spur and jog_ready and spur_close_allowed)
        self.gripper_jog_step.setEnabled(not self.gripper_busy)
        cleaner = self.node.selected_tool == 'cleaner'
        configured = bool(self.tool_status.get('actuators_discovered'))
        self.clean_start.setEnabled(manual and cleaner and profile_ok
                                    and motion and configured)
        self.clean_stop.setEnabled(manual and cleaner and profile_ok and motion)
        for widget in (self.spur_minus_5, self.spur_zero, self.spur_plus_5):
            if widget is not None:
                widget.setEnabled(False)
        if spur:
            calibration_ready = (manual and calibration.get('active', False)
                                 and self.spur_torque_state == 'ON'
                                 and calibration.get('enabled', False)
                                 and bool(self.tool_status.get('calibration_jog_enabled'))
                                 and self._gripper_positions_synchronized())
            for widget in (self.motor_minus_half, self.motor_plus_half,
                           self.motor_minus_one, self.motor_plus_one,
                           self.capture_open, self.capture_close):
                widget.setEnabled(calibration_ready)
            captures = calibration.get('captures', {})
            both_captured = (set(captures) == {'open', 'close'}
                             and captures['open'] != captures['close'])
            self.validate_calibration.setEnabled(
                manual and calibration.get('active', False) and both_captured)
            self.save_calibration.setEnabled(
                manual and calibration.get('active', False)
                and calibration.get('validated', False))
        self.read_diag.setEnabled(spur and not self.mock_mode)
        self.start_cal.setEnabled(
            spur and manual and bool(self.tool_status.get('calibration_jog_enabled'))
            and not calibration.get('active', False))

    def _tool_motion_ready(self):
        fresh = time.monotonic() - self.last_status_time < 1.5
        scope_ok = self.tool_status.get('control_scope') == self.node.control_scope
        expected_ids = set(self.profile.get('actuator_ids', []))
        samples = self.tool_status.get('actuators', [])
        online_ids = {sample.get('id') for sample in samples
                      if sample.get('online')}
        actuators_ok = bool(expected_ids) and online_ids == expected_ids
        profile_ready = bool(self.tool_status.get('profile_valid')) \
            and bool(self.tool_status.get('calibrated'))
        temporary_ready = bool(self.tool_status.get('temporary_jog_ready')) \
            and self.node.temporary_jog_mode
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('motion_allowed')) and scope_ok
                and actuators_ok and (profile_ready or temporary_ready)
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _tool_enable_ready(self):
        """Readiness before torque is enabled; used only by ENABLE ID5."""
        fresh = time.monotonic() - self.last_status_time < 1.5
        return (fresh and bool(self.tool_status.get('bridge_connected'))
                and bool(self.tool_status.get('online'))
                and self.tool_status.get('position') is not None
                and self.tool_status.get('hardware_error') == 0
                and not bool(self.tool_status.get('read_only'))
                and not bool(self.tool_status.get('emergency_stop'))
                and not bool(self.tool_status.get('tool_detached')))

    def _update_gripper_state(self, busy, state):
        self.gripper_busy = bool(busy)
        self.gripper_busy_label.setText(
            f'BUSY: {state}' if busy else f'READY: {state}')
        self.gripper_busy_label.setStyleSheet(
            FALSE_STYLE if busy else TRUE_STYLE)
        self._refresh_buttons()

    def _motor_endpoints(self):
        endpoints = self.profile.get('motor_endpoints', {})
        return {
            dxl_id: endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
            for dxl_id in self.profile.get('actuator_ids', [])}

    def _gripper_samples(self):
        return {sample.get('id'): sample
                for sample in self.tool_status.get('actuators', [])}

    def _normalized_positions(self):
        samples = self._gripper_samples()
        fractions = {}
        for dxl_id, endpoint in self._motor_endpoints().items():
            sample = samples.get(dxl_id)
            if not endpoint or not sample or sample.get('position') is None:
                return {}
            span = endpoint['open'] - endpoint['close']
            if span == 0:
                return {}
            fractions[dxl_id] = (
                (float(sample['position']) - endpoint['close']) / span)
        return fractions

    def _gripper_positions_synchronized(self):
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            return sample.get('position') is not None and bool(sample.get('online'))
        fractions = self._normalized_positions()
        return (len(fractions) == len(self.profile.get('actuator_ids', []))
                and max(fractions.values()) - min(fractions.values()) <= 0.05)

    def _update_gripper_feedback(self):
        samples = self._gripper_samples()
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = samples.get(5, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(5)
            error = None if current is None or target is None else target - current
            self.gripper_position_label.setText(
                f'Spur Gripper | Current: {current} | Target: {target} '
                f'| Error: {error}')
            self.gripper_feedback_label.setText(
                f'ID5: current={current}, target={target}, error={error}, '
                f'current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}\n'
                f'Safe range: {self.temporary_jog_safe_min} ~ '
                f'{self.temporary_jog_safe_max}\n'
                f'Mechanical range: {self.temporary_jog_mechanical_open} ~ '
                f'{self.temporary_jog_mechanical_close}')
            self.spur_torque_state = self.tool_status.get(
                'tool_torque_state', 'UNKNOWN')
            self.spur_torque_enabled = self.spur_torque_state == 'ON'
            self.spur_endpoints = dict((self.tool_status.get('calibration') or {}).get(
                'captures', self.spur_endpoints))
            self.captured_endpoints_label.setText(
                f'Captured OPEN: {self.spur_endpoints.get("open", "—")} | '
                f'CLOSE: {self.spur_endpoints.get("close", "—")}')
            self.spur_actual_state.setText(
                f'ID5: position={current} torque={self.spur_torque_state} '
                f'load={sample.get("effort")} mode={sample.get("operating_mode")} '
                f'velocity={sample.get("profile_velocity")} '
                f'acceleration={sample.get("profile_acceleration")} '
                f'hardware_error={self.tool_status.get("hardware_error")} '
                f'model={self.tool_status.get("model")} '
                f'fsm={self.fsm_state} calibrated={self.tool_status.get("calibrated")}')
            zero = 'UNSET' if self.spur_zero_tick is None else str(self.spur_zero_tick)
            self.spur_mapping.setText(
                'Output mapping: zero offset/reference tick=' + zero + '\n'
                'GUI output angle → motor tick: motor_deg = '
                f'output_deg × {self.spur_gear_ratio:.3f} / '
                f'({self.spur_output_direction:+d}); '
                'tick = zero + motor_deg × 4096 / 360.\n'
                'External spur pair: output direction is the inverse of motor direction.')
            return
        fractions = self._normalized_positions()
        if fractions:
            normalized = sum(fractions.values()) / len(fractions)
            spread = max(fractions.values()) - min(fractions.values())
            self.gripper_position_label.setText(
                f'Gripper position: {normalized:.4f} '
                f'(0.0=closed, 1.0=open, motor spread={spread:.4f})')
            if not self.gripper_busy and spread > 0.05:
                self.gripper_busy_label.setText(
                    f'BLOCKED: motor normalized spread {spread:.4f} > 0.0500')
                self.gripper_busy_label.setStyleSheet(FALSE_STYLE)
        else:
            self.gripper_position_label.setText('Gripper position: UNKNOWN')
        lines = []
        for dxl_id in self.profile.get('actuator_ids', []):
            sample = samples.get(dxl_id, {})
            current = sample.get('position')
            target = self.gripper_target_ticks.get(dxl_id)
            error = None if current is None or target is None else target - current
            lines.append(
                f'ID{dxl_id}: current={current}, target={target}, '
                f'error={error}, current/load={sample.get("effort")}, '
                f'online={sample.get("online", False)}, '
                f'actual torque={sample.get("torque_state", "UNKNOWN")}, '
                f'hardware error={sample.get("hardware_error")}')
        self.gripper_feedback_label.setText('\n'.join(lines) or 'No actuator data')

    def _jog_gripper(self, direction):
        reason = self._gripper_jog_block_reason()
        if reason:
            self._append_log(f'Gripper jog blocked: {reason}')
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            self._jog_spur(direction)
            return
        endpoints = self._motor_endpoints()
        fractions = self._normalized_positions()
        current = sum(fractions.values()) / len(fractions)
        spread = max(fractions.values()) - min(fractions.values())
        if spread > 0.05:
            self._append_log(
                f'Gripper jog blocked: motor normalized positions disagree '
                f'({fractions}, spread={spread:.4f})')
            return
        max_span = max(abs(ep['open'] - ep['close'])
                       for ep in endpoints.values())
        step = int(self.gripper_jog_step.currentText())
        target_fraction = min(1.0, max(
            0.0, current + direction * step / max_span))
        if abs(target_fraction - current) < 1e-9:
            self._append_log('Gripper jog blocked: already at profile boundary')
            return
        low = int(self.profile['safe_min_tick'])
        high = int(self.profile['safe_max_tick'])
        targets = {
            dxl_id: int(round(ep['close'] + target_fraction
                              * (ep['open'] - ep['close'])))
            for dxl_id, ep in endpoints.items()}
        outside = {dxl_id: target for dxl_id, target in targets.items()
                   if not low <= target <= high}
        if outside:
            self._append_log(
                f'Gripper jog blocked: targets outside [{low}, {high}]: '
                f'{outside}')
            return
        close_position = float(self.profile.get('close_position', 0.0))
        open_position = float(self.profile.get('open_position', 1.0))
        logical = close_position + target_fraction * (
            open_position - close_position)
        self._append_log(
            f'Gripper jog request: normalized={target_fraction:.6f}, '
            f'targets={targets}, step={step}')
        if self.node.command_gripper(logical):
            self.gripper_target_ticks = targets
            self._update_gripper_feedback()

    def _gripper_jog_block_reason(self):
        if (self.node.selected_tool == 'dual_motor_gripper'
                and (self.tool_status.get('dual_calibration') or {}).get('state')
                != 'READY'):
            return 'dual endpoint recalibration is required'
        if self.node.control_scope != 'END_EFFECTOR_ONLY':
            return 'control scope is not END_EFFECTOR_ONLY'
        if self.node.selected_tool not in (
                'dual_motor_gripper', 'spur_1motor_gripper'):
            return 'selected tool is not a supported gripper'
        if self.control_mode != 'MANUAL':
            return 'ownership is not MANUAL'
        if self.gripper_busy or self.node.gripper_busy:
            return 'BUSY'
        if not self._tool_motion_ready():
            return 'bridge/tool safety status is not ready or fresh'
        if self.node.selected_tool == 'spur_1motor_gripper':
            sample = self._gripper_samples().get(5, {})
            if sample.get('position') is None or not sample.get('online'):
                return 'ID5 position/online feedback unavailable'
            return ''
        if not self._normalized_positions():
            return 'current actuator positions are unavailable'
        if not self._gripper_positions_synchronized():
            return 'motor normalized positions are not synchronized'
        return ''

    def _jog_spur(self, direction):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        step = int(self.gripper_jog_step.currentText())
        # Spur mapping: decreasing ticks opens, increasing ticks closes.
        target = int(current) + direction * step
        in_safe = self.temporary_jog_safe_min <= target <= self.temporary_jog_safe_max
        recovery = current < self.temporary_jog_safe_min or current > self.temporary_jog_safe_max
        inward = ((current > self.temporary_jog_safe_max and direction < 0)
                  or (current < self.temporary_jog_safe_min and direction > 0))
        if (not in_safe and not (recovery and inward)):
            self._append_log(
                f'Spur jog blocked: target={target} outside safe range '
                f'[{self.temporary_jog_safe_min}, {self.temporary_jog_safe_max}]')
            return
        if self.node.command_gripper(target):
            self.gripper_target_ticks = {5: target}
            self._update_gripper_feedback()

    def _jog_spur_motor(self, degrees):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None or self.spur_torque_state != 'ON':
            self._append_log('Motor jog blocked: ID5 position/actual torque unavailable')
            return
        if self.node.command_calibration('jog_motor_degrees', delta_deg=float(degrees)):
            self._append_log(f'ID5 CalibrationSession jog {degrees:+.1f}° requested')

    def _capture_spur_endpoint(self, label):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None:
            self._append_log(f'Capture {label} blocked: ID5 position unavailable')
            return
        if self.node.command_calibration(f'capture_{label}'):
            self._append_log(f'CalibrationSession capture {label.upper()} requested (read only)')
            self._refresh_buttons()

    def _validate_spur_calibration(self):
        if self.node.command_calibration('validate'):
            self._append_log('CalibrationSession validation requested (no motor write)')

    def _save_spur_calibration(self):
        if self.node.command_calibration('save'):
            self._append_log(
                'Calibration save requested; bridge will atomically reload and require READY')

    def _command_tool(self, command):
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.node.command_spur_fsm(command)
            return
        position = float(self.profile.get(
            'open_position' if command == 'OPEN' else 'close_position',
            1.0 if command == 'OPEN' else 0.0))
        self.node.command_gripper(position)

    def _stop_tool(self):
        if self.node.selected_tool == 'spur_1motor_gripper':
            self.node.command_spur_fsm('STOP')
            return
        if self.node.selected_tool == 'dual_motor_gripper':
            self.node.set_dual_motor_enabled(False, self.profile.get('actuator_ids', []))
            return
        self.node.stop_gripper()

    def _enable_spur_motor(self):
        sample = self._gripper_samples().get(5, {})
        current = sample.get('position')
        if current is None:
            self._append_log('ID5 Enable blocked: current tick feedback unavailable')
            return
        self.node.command_calibration('enable')
        self._append_log('CalibrationSession ENABLE ID5 requested')

    def _disable_spur_motor(self):
        self.node.command_calibration('disable')
        self._append_log('CalibrationSession DISABLE ID5 requested')

    def _enable_dual_motors(self):
        if self.node.set_dual_motor_enabled(True, self.profile.get('actuator_ids', [])):
            self._append_log('Operator requested dual torque enable for IDs [3, 4]')

    def _disable_dual_motors(self):
        if self.node.set_dual_motor_enabled(False, self.profile.get('actuator_ids', [])):
            self._append_log('Operator requested dual torque disable for IDs [3, 4]')

    def _manual_dual_recovery_jog(self, actuator_id, delta_deg):
        if self.node.manual_dual_recovery_jog(actuator_id, delta_deg):
            self._append_log(
                f'Operator requested one-click recovery jog: ID{actuator_id} '
                f'{delta_deg:+.1f}° (bridge re-reads actual position)')

    def _start_dual_calibration(self):
        if self.node.command_dual_calibration('start'):
            self._append_log('Dual endpoint calibration started (no motor write)')

    def _jog_dual_calibration_motor(self, actuator_id, direction):
        if self.dual_calibration_step is None:
            return
        degrees = float(self.dual_calibration_step.currentText()) * float(direction)
        if self.node.command_dual_calibration(
                'jog_motor_degrees', actuator_id=int(actuator_id),
                delta_deg=degrees):
            self._append_log(
                f'Dual calibration one-click jog requested: ID{actuator_id} '
                f'{degrees:+.1f}°')

    def _command_dual_calibration(self, command):
        if self.node.command_dual_calibration(command):
            self._append_log(f'Dual calibration command requested: {command}')

    def _command_spur_output_deg(self, output_deg):
        self._append_log('Output-angle command is unavailable during ID5 calibration')

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            event.ignore()
            return
        focus = self.focusWidget()
        editing = isinstance(
            focus, (QAbstractSpinBox, QLineEdit, QTextEdit, QComboBox))
        enabled = (self.node.control_scope == 'END_EFFECTOR_ONLY'
                   and self.control_mode == 'MANUAL')
        if enabled and event.key() == Qt.Key_Space:
            self._stop_tool()
            event.accept()
            return
        if self.node.selected_tool == 'spur_1motor_gripper':
            event.ignore()
            return
        if enabled and not editing and event.key() == Qt.Key_Left:
            self._jog_gripper(-1)
            event.accept()
            return
        if enabled and not editing and event.key() == Qt.Key_Right:
            self._jog_gripper(1)
            event.accept()
            return
        super().keyPressEvent(event)

    def _jog(self, joint, sign):
        self.node.jog_arm(joint, sign * float(self.jog_step.currentText()))

    def _arm_target(self, joint):
        self.node.command_arm(joint, self.arm_targets[joint].value())

    def _request_mode(self):
        requested = self.mode_combo.currentText()
        self._append_log(
            f'Mode request clicked: requested={requested}, '
            f'approved={self.control_mode}')
        if (not self.mock_mode and requested == 'MANUAL'
                and self.fsm_state not in ToolManager.SAFE_CHANGE_STATES
                and not (self.node.selected_tool == 'spur_1motor_gripper'
                         and self.fsm_state == 'CALIBRATION_REQUIRED')
                and not (self.node.selected_tool == 'dual_motor_gripper'
                         and self.node.control_scope == 'END_EFFECTOR_ONLY')):
            QMessageBox.warning(
                self, 'Ownership denied',
                f'MANUAL is allowed only in IDLE/STOWED; current={self.fsm_state}')
            return
        self.node.request_mode(requested)

    def _request_tool_change(self):
        requested = self.tool_combo.currentText()
        current = self.tool_status.get('tool_type', self.node.selected_tool)
        if requested == current:
            self._append_log(f'{requested} is already selected')
            return
        if self.fsm_state not in ToolManager.SAFE_CHANGE_STATES:
            QMessageBox.warning(
                self, 'Tool change denied',
                f'ToolManager policy denies changes in {self.fsm_state}')
            self.tool_combo.setCurrentText(current)
            return
        QMessageBox.information(
            self, 'Restart required',
            'Runtime hardware reprovisioning is not implemented. Stop the launch, '
            f'detach safely, then restart with tool_type:={requested}.')
        self.tool_combo.setCurrentText(current)

    def _estop(self):
        self.node.emergency_stop()
        self.estop_state.setText('E-STOP: REQUESTED')
        self.estop_state.setStyleSheet(FALSE_STYLE)

    def _detach(self):
        answer = QMessageBox.question(
            self, 'Confirm detach', 'Mark the current tool as DETACHED and stop it?')
        if answer == QMessageBox.Yes:
            self.node.tool_detached()

    def _run_process(self, program, args):
        process = QProcess(self)
        process.setProgram(program)
        process.setArguments(args)
        process.readyReadStandardOutput.connect(
            lambda: self._append_log(bytes(
                process.readAllStandardOutput()).decode(errors='replace')))
        process.readyReadStandardError.connect(
            lambda: self._append_log(bytes(
                process.readAllStandardError()).decode(errors='replace')))
        process.finished.connect(lambda: self._append_log('Diagnostic process finished'))
        self.processes.append(process)
        process.start()

    def _read_only_diagnostic(self):
        if time.monotonic() - self.last_status_time < 1.5:
            self._append_log(
                'Bridge already owns the serial bus; using /tool/status read-only '
                f'diagnostics: {self.tool_status}')
            return
        ids = self.profile.get('actuator_ids', [5])
        self._run_process('ros2', [
            'run', 'dynamixel_control', 'spur_gripper_calibration',
            '--actuator-id', str(ids[0]), '--read-only'])

    def _start_calibration(self):
        if self.node.command_calibration('start'):
            self._append_log('CalibrationSession started (no register write)')

    def _rebuild_diagnostics(self, actuators, joint_values=None):
        joint_values = joint_values or {}
        rows = []
        for index, joint in enumerate(ARM_JOINTS):
            sample = joint_values.get(joint, {})
            position = sample.get('position', self.node.positions.get(joint))
            effort = sample.get('effort', self.node.efforts.get(joint))
            rows.append((index, joint, position, effort, position is not None))
        for sample in actuators:
            rows.append((sample.get('id'), sample.get('joint'),
                         sample.get('position'), sample.get('effort'),
                         sample.get('online', False)))
        self.diag.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                text = '—' if value is None else str(value)
                item = QTableWidgetItem(text)
                if column == 4:
                    item.setForeground(Qt.darkGreen if value else Qt.red)
                self.diag.setItem(row, column, item)

    def _append_log(self, text):
        self.log.append(str(text).strip())
