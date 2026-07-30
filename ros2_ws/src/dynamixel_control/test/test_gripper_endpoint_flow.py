"""Integration tests for endpoint capture and load-calibration handoff."""

from types import SimpleNamespace

import pytest

from dynamixel_control import gripper_calibration as endpoint_tool
from dynamixel_control import gripper_load_calibration as load_tool


class FakeEndpointBus:
    """Record lifecycle calls without accessing serial hardware."""

    instances = []

    def __init__(self):
        self.opened = False
        self.closed = False
        self.operating_modes = {3: 4, 4: 4}
        self.__class__.instances.append(self)

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True


def test_endpoint_capture_file_is_consumed_without_schema_translation(
        tmp_path, monkeypatch):
    """Exercise the producer file and load-calibration parser together."""
    output = tmp_path / "gripper_endpoints.json"
    captures = iter(({3: 739, 4: 2106}, {3: 1100, 4: 1700}))
    monkeypatch.setattr(endpoint_tool, "Bus", FakeEndpointBus)
    monkeypatch.setattr(
        endpoint_tool, "require_safe_read_state", lambda *args, **kwargs: {})
    monkeypatch.setattr(
        endpoint_tool, "capture_endpoint", lambda *args: next(captures))

    endpoint_tool.stage_endpoints(SimpleNamespace(
        samples=7, min_span_ticks=50, output=output))

    data = endpoint_tool.load_endpoints(output)
    assert data["id3_open_tick"] == 739
    assert data["id3_close_tick"] == 1100
    assert data["id4_open_tick"] == 2106
    assert data["id4_close_tick"] == 1700
    assert data["operating_modes"] == {"3": 4, "4": 4}
    endpoints = load_tool.load_endpoints(output)
    assert endpoints == {
        3: {"open": 739, "close": 1100},
        4: {"open": 2106, "close": 1700},
    }
    assert endpoints.operating_modes == {3: 4, 4: 4}
    assert load_tool.goals_for_ratio(0.0, endpoints) == {3: 739, 4: 2106}
    assert load_tool.goals_for_ratio(1.0, endpoints) == {3: 1100, 4: 1700}
    assert load_tool.ratio_for_position(3, 1100, endpoints) == 1.0
    assert load_tool.ratio_for_position(4, 1700, endpoints) == 1.0
    assert FakeEndpointBus.instances[-1].closed


@pytest.mark.parametrize("samples,min_span", [(0, 50), (7, 0)])
def test_invalid_capture_arguments_fail_before_opening_bus(
        tmp_path, samples, min_span):
    """Reject unusable captures before acquiring the serial port."""
    FakeEndpointBus.instances.clear()
    with pytest.raises(endpoint_tool.CalibrationError):
        endpoint_tool.stage_endpoints(SimpleNamespace(
            samples=samples, min_span_ticks=min_span,
            output=tmp_path / "endpoints.json"))
    assert not FakeEndpointBus.instances
