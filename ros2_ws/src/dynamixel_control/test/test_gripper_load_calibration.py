"""Unit tests for guarded two-motor gripper calibration."""

from types import SimpleNamespace

import pytest

from dynamixel_control.gripper_load_calibration import (
    CalibrationError,
    CalibrationSession,
    goals_for_ratio,
    ratio_for_position,
    unwrap_position,
    update_asymmetry_count,
    validate_target_ratio,
)


def test_endpoint_mapping_and_distinct_goals():
    assert goals_for_ratio(0.0) == {3: 1180, 4: 2510}
    assert goals_for_ratio(0.5) == {3: 508, 4: 1861}
    assert goals_for_ratio(1.0) == {3: -164, 4: 1212}
    assert all(goals_for_ratio(r)[3] != goals_for_ratio(r)[4]
               for r in (0.0, 0.5, 0.7, 1.0))


def test_wrap_and_unwrap_are_continuous():
    assert unwrap_position(5, 4092) == -4
    assert unwrap_position(-4, 4080) == -16
    assert ratio_for_position(3, -164) == pytest.approx(1.0)


def test_ratio_range_guard():
    validate_target_ratio(0.0)
    validate_target_ratio(0.70)
    validate_target_ratio(1.10)
    with pytest.raises(CalibrationError):
        validate_target_ratio(1.101)
    with pytest.raises(ValueError):
        goals_for_ratio(-0.1)


def test_asymmetry_ignores_quantization_and_requires_consecutive_samples():
    assert update_asymmetry_count([0, 2], 3, 0, 2) == 0
    assert update_asymmetry_count([0, 3], 3, 0, 2) == 1
    with pytest.raises(CalibrationError, match="sustained asymmetric"):
        update_asymmetry_count([0, 4], 3, 1, 2)


def test_asymmetry_counter_resets_when_both_stall_or_move():
    assert update_asymmetry_count([0, 0], 3, 1, 2) == 0
    assert update_asymmetry_count([3, 5], 3, 1, 2) == 0


class FakeBus:
    def __init__(self):
        self.device = "/dev/fake"
        self.disabled = False
        self.closed = False
        self.opened = True

    def set_torque(self, enabled):
        assert enabled is False
        self.disabled = True

    def read_states(self):
        return {
            i: {"torque": 0, "hardware_error": 0}
            for i in (3, 4)
        }

    def close(self):
        self.closed = True
        self.opened = False


def test_emergency_stop_and_cleanup_disable_both(tmp_path):
    bus = FakeBus()
    args = SimpleNamespace(armed=False, output_dir=tmp_path,
                           load_stop_threshold=300, max_ratio=0.7)
    session = CalibrationSession(bus, args)
    session.emergency_stop()
    assert bus.disabled
    session.cleanup()
    assert bus.disabled and bus.closed
