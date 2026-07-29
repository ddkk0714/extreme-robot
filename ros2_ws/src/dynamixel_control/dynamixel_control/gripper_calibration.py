#!/usr/bin/env python3
"""Safe two-motor XL430 gripper endpoint and gradual-grasp calibration."""

import argparse
import json
import statistics
import time
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, GroupSyncWrite, PacketHandler, PortHandler

DEVICE = "/dev/ttyUSB0"
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 2.0
DXL_IDS = (3, 4)
EXPECTED_MODEL = 1060

ADDR_MODEL_NUMBER = 0
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

TORQUE_OFF = 0
TORQUE_ON = 1
POSITION_MODE = 3
ENDPOINT_FILE = Path("/tmp/gripper_endpoints.json")
RESULT_FILE = Path("/tmp/gripper_grasp_result.json")


class CalibrationError(RuntimeError):
    pass


def signed(value, bits):
    sign_bit = 1 << (bits - 1)
    return value - (1 << bits) if value & sign_bit else value


def le32(value):
    value &= 0xFFFFFFFF
    return [(value >> shift) & 0xFF for shift in (0, 8, 16, 24)]


class Bus:
    def __init__(self):
        self.port = PortHandler(DEVICE)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_goal = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, 4)

    def open(self):
        if not self.port.openPort():
            raise CalibrationError(f"cannot open {DEVICE}")
        if not self.port.setBaudRate(BAUD_RATE):
            self.port.closePort()
            raise CalibrationError(f"cannot set baud rate {BAUD_RATE}")

    def close(self):
        self.port.closePort()

    def _read(self, dxl_id, address, size, label):
        reader = {
            1: self.packet.read1ByteTxRx,
            2: self.packet.read2ByteTxRx,
            4: self.packet.read4ByteTxRx,
        }[size]
        value, result, error = reader(self.port, dxl_id, address)
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)}")
        if error:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getRxPacketError(error)}")
        return value

    def read1(self, dxl_id, address, label):
        return self._read(dxl_id, address, 1, label)

    def read2(self, dxl_id, address, label):
        return self._read(dxl_id, address, 2, label)

    def read4(self, dxl_id, address, label):
        return self._read(dxl_id, address, 4, label)

    def _write(self, dxl_id, address, size, value, label):
        writer = {
            1: self.packet.write1ByteTxRx,
            4: self.packet.write4ByteTxRx,
        }[size]
        result, error = writer(self.port, dxl_id, address, value)
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)}")
        if error:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getRxPacketError(error)}")

    def snapshot(self, dxl_id):
        return {
            "model": self.read2(dxl_id, ADDR_MODEL_NUMBER, "model"),
            "operating_mode": self.read1(
                dxl_id, ADDR_OPERATING_MODE, "operating mode"),
            "torque": self.read1(dxl_id, ADDR_TORQUE_ENABLE, "torque"),
            "hardware_error": self.read1(
                dxl_id, ADDR_HARDWARE_ERROR_STATUS, "hardware error"),
            "position": signed(self.read4(
                dxl_id, ADDR_PRESENT_POSITION, "position"), 32),
            "load": signed(self.read2(
                dxl_id, ADDR_PRESENT_LOAD, "present load"), 16),
        }

    def set_profile(self, dxl_id, acceleration, velocity):
        self._write(dxl_id, ADDR_PROFILE_ACCELERATION, 4,
                    acceleration, "profile acceleration")
        self._write(dxl_id, ADDR_PROFILE_VELOCITY, 4,
                    velocity, "profile velocity")

    def set_torque(self, dxl_id, enabled):
        self._write(dxl_id, ADDR_TORQUE_ENABLE, 1,
                    TORQUE_ON if enabled else TORQUE_OFF, "torque")

    def write_goals(self, goals):
        self.sync_goal.clearParam()
        for dxl_id in DXL_IDS:
            if not self.sync_goal.addParam(dxl_id, le32(goals[dxl_id])):
                raise CalibrationError(f"cannot queue goal for ID {dxl_id}")
        result = self.sync_goal.txPacket()
        self.sync_goal.clearParam()
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"sync goal write: {self.packet.getTxRxResult(result)}")


def require_safe_read_state(bus, require_position_mode=False):
    snapshots = {dxl_id: bus.snapshot(dxl_id) for dxl_id in DXL_IDS}
    for dxl_id, state in snapshots.items():
        if state["model"] != EXPECTED_MODEL:
            raise CalibrationError(
                f"ID {dxl_id}: model {state['model']} != {EXPECTED_MODEL}")
        if state["hardware_error"] != 0:
            raise CalibrationError(
                f"ID {dxl_id}: hardware error 0x{state['hardware_error']:02X}")
        if state["torque"] != TORQUE_OFF:
            raise CalibrationError(
                f"ID {dxl_id}: torque is enabled; disable it before calibration")
        if require_position_mode and state["operating_mode"] != POSITION_MODE:
            raise CalibrationError(
                f"ID {dxl_id}: operating mode {state['operating_mode']} != 3")
    return snapshots


def capture_endpoint(bus, label, sample_count):
    input(f"Move the torque-free gripper fully {label}, then press Enter: ")
    positions = {dxl_id: [] for dxl_id in DXL_IDS}
    for _ in range(sample_count):
        states = require_safe_read_state(bus)
        for dxl_id in DXL_IDS:
            positions[dxl_id].append(states[dxl_id]["position"])
        time.sleep(0.1)
    result = {
        dxl_id: int(statistics.median(values))
        for dxl_id, values in positions.items()
    }
    print(f"{label}: ID3={result[3]}, ID4={result[4]}")
    return result


def stage_endpoints(args):
    bus = Bus()
    bus.open()
    try:
        require_safe_read_state(bus)
        opened = capture_endpoint(bus, "open", args.samples)
        closed = capture_endpoint(bus, "closed", args.samples)
        for dxl_id in DXL_IDS:
            if abs(closed[dxl_id] - opened[dxl_id]) < args.min_span_ticks:
                raise CalibrationError(
                    f"ID {dxl_id}: endpoint span is too small")
        data = {
            "device": DEVICE,
            "baud_rate": BAUD_RATE,
            "protocol": PROTOCOL_VERSION,
            "model": EXPECTED_MODEL,
            "id3_open_tick": opened[3],
            "id3_close_tick": closed[3],
            "id4_open_tick": opened[4],
            "id4_close_tick": closed[4],
            "approved": False,
        }
        args.output.write_text(json.dumps(data, indent=2) + "\n")
        print(json.dumps(data, indent=2))
        print(f"Saved unapproved endpoints to {args.output}")
    finally:
        bus.close()


def load_endpoints(path):
    data = json.loads(path.read_text())
    required = (
        "id3_open_tick", "id3_close_tick",
        "id4_open_tick", "id4_close_tick",
    )
    if any(key not in data for key in required):
        raise CalibrationError("endpoint file is incomplete")
    return data


def interpolate(data, ratio):
    return {
        3: round(data["id3_open_tick"] + ratio * (
            data["id3_close_tick"] - data["id3_open_tick"])),
        4: round(data["id4_open_tick"] + ratio * (
            data["id4_close_tick"] - data["id4_open_tick"])),
    }


def observed_ratio(position, opened, closed):
    return (position - opened) / (closed - opened)


def check_live_safety(states, endpoints, args, previous_positions=None):
    for dxl_id, state in states.items():
        if state["hardware_error"]:
            raise CalibrationError(
                f"ID {dxl_id}: hardware error 0x{state['hardware_error']:02X}")
        if abs(state["load"]) >= args.max_abs_load:
            raise CalibrationError(
                f"ID {dxl_id}: abs(load) {abs(state['load'])} reached safety limit")
    ratios = {
        3: observed_ratio(states[3]["position"], endpoints["id3_open_tick"],
                          endpoints["id3_close_tick"]),
        4: observed_ratio(states[4]["position"], endpoints["id4_open_tick"],
                          endpoints["id4_close_tick"]),
    }
    if abs(ratios[3] - ratios[4]) > args.max_ratio_deviation:
        raise CalibrationError(
            f"coupling mismatch: ID3 ratio={ratios[3]:.3f}, ID4 ratio={ratios[4]:.3f}")
    movement = None
    if previous_positions is not None:
        movement = max(abs(states[i]["position"] - previous_positions[i])
                       for i in DXL_IDS)
    return ratios, movement


def stage_grasp(args):
    if not args.approve_endpoints:
        raise CalibrationError("Stage 2 requires --approve-endpoints")
    if not 0.0 < args.ratio_step <= 0.01:
        raise CalibrationError("ratio step must be in (0, 0.01]")
    if not 0.0 < args.max_close_ratio < 1.0:
        raise CalibrationError("max close ratio must be in (0, 1)")
    if not 0.0 <= args.extra_close_ratio <= 0.02:
        raise CalibrationError("extra close ratio must be in [0, 0.02]")
    if not 0 < args.contact_load < args.max_abs_load:
        raise CalibrationError("contact load must be below the absolute load limit")
    if not 1 <= args.profile_acceleration <= 25:
        raise CalibrationError("profile acceleration must be in [1, 25]")
    if not 1 <= args.profile_velocity <= 80:
        raise CalibrationError("profile velocity must be in [1, 80]")
    endpoints = load_endpoints(args.endpoints)
    print(json.dumps(endpoints, indent=2))
    confirmation = input(
        "Type APPROVE POWERED GRASP to confirm these endpoints: ")
    if confirmation != "APPROVE POWERED GRASP":
        raise CalibrationError("powered calibration not approved")

    bus = Bus()
    profiles = {}
    torque_attempted = False
    max_load = {3: 0, 4: 0}
    result = {"contact": False}
    bus.open()
    try:
        initial = require_safe_read_state(bus, require_position_mode=True)
        for dxl_id in DXL_IDS:
            open_tick = endpoints[f"id{dxl_id}_open_tick"]
            if abs(initial[dxl_id]["position"] - open_tick) > args.open_tolerance:
                raise CalibrationError(
                    f"ID {dxl_id}: manually place at open endpoint before Stage 2")
            profiles[dxl_id] = {
                "acceleration": bus.read4(
                    dxl_id, ADDR_PROFILE_ACCELERATION, "profile acceleration"),
                "velocity": bus.read4(
                    dxl_id, ADDR_PROFILE_VELOCITY, "profile velocity"),
            }
            bus.set_profile(dxl_id, args.profile_acceleration,
                            args.profile_velocity)

        for dxl_id in DXL_IDS:
            bus.set_torque(dxl_id, True)
            torque_attempted = True
        bus.write_goals(interpolate(endpoints, 0.0))
        time.sleep(args.initial_settle_seconds)

        contact_count = 0
        previous_positions = None
        ratio = 0.0
        while ratio < args.max_close_ratio:
            ratio = min(args.max_close_ratio, ratio + args.ratio_step)
            goals = interpolate(endpoints, ratio)
            bus.write_goals(goals)
            deadline = time.monotonic() + args.step_timeout
            while time.monotonic() < deadline:
                states = {i: bus.snapshot(i) for i in DXL_IDS}
                _, movement = check_live_safety(
                    states, endpoints, args, previous_positions)
                previous_positions = {
                    i: states[i]["position"] for i in DXL_IDS}
                for i in DXL_IDS:
                    max_load[i] = max(max_load[i], abs(states[i]["load"]))
                load_contact = max(abs(states[i]["load"]) for i in DXL_IDS) \
                    >= args.contact_load
                still = movement is not None and movement <= args.max_contact_movement
                contact_count = contact_count + 1 if load_contact and still else 0
                print(
                    f"ratio={ratio:.3f} goals={goals} "
                    f"pos={{3:{states[3]['position']},4:{states[4]['position']}}} "
                    f"load={{3:{states[3]['load']},4:{states[4]['load']}}}")
                if contact_count >= args.contact_samples:
                    contact_ratio = ratio
                    contact_states = states
                    extra_ratio = min(
                        args.max_close_ratio, ratio + args.extra_close_ratio)
                    bus.write_goals(interpolate(endpoints, extra_ratio))
                    time.sleep(args.extra_settle_seconds)
                    final = {i: bus.snapshot(i) for i in DXL_IDS}
                    check_live_safety(final, endpoints, args)
                    result = {
                        "contact": True,
                        "contact_ratio": contact_ratio,
                        "final_ratio": extra_ratio,
                        "contact_before_fully_closed": contact_ratio < 1.0,
                        "id3_contact_tick": contact_states[3]["position"],
                        "id4_contact_tick": contact_states[4]["position"],
                        "id3_present_load": contact_states[3]["load"],
                        "id4_present_load": contact_states[4]["load"],
                        "id3_final_tick": final[3]["position"],
                        "id4_final_tick": final[4]["position"],
                        "maximum_observed_load": max(max_load.values()),
                        "maximum_observed_load_by_id": max_load,
                    }
                    args.output.write_text(json.dumps(result, indent=2) + "\n")
                    print(json.dumps(result, indent=2))
                    return
                if all(abs(states[i]["position"] - goals[i]) <= args.goal_tolerance
                       for i in DXL_IDS):
                    break
                time.sleep(1.0 / args.sample_hz)
        result.update({
            "maximum_observed_load": max(max_load.values()),
            "maximum_observed_load_by_id": max_load,
        })
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(
            "No contact detected before the configured maximum close ratio; "
            "the calibrated mechanical endpoint was not commanded.")
    finally:
        if torque_attempted:
            for dxl_id in DXL_IDS:
                try:
                    bus.set_torque(dxl_id, False)
                except Exception as exc:
                    print(f"WARNING: failed to disable torque on ID {dxl_id}: {exc}")
        for dxl_id, profile in profiles.items():
            try:
                bus.set_profile(dxl_id, profile["acceleration"], profile["velocity"])
            except Exception as exc:
                print(f"WARNING: failed to restore profile on ID {dxl_id}: {exc}")
        bus.close()
        print("Torque-disable attempted for both motors; serial port closed.")


def parser():
    root = argparse.ArgumentParser()
    sub = root.add_subparsers(dest="stage", required=True)
    endpoints = sub.add_parser("endpoints", help="read-only endpoint capture")
    endpoints.add_argument("--samples", type=int, default=7)
    endpoints.add_argument("--min-span-ticks", type=int, default=50)
    endpoints.add_argument("--output", type=Path, default=ENDPOINT_FILE)
    endpoints.set_defaults(func=stage_endpoints)

    grasp = sub.add_parser("grasp", help="approved powered gradual grasp")
    grasp.add_argument("--endpoints", type=Path, default=ENDPOINT_FILE)
    grasp.add_argument("--output", type=Path, default=RESULT_FILE)
    grasp.add_argument("--approve-endpoints", action="store_true")
    grasp.add_argument("--ratio-step", type=float, default=0.005)
    grasp.add_argument("--profile-velocity", type=int, default=20)
    grasp.add_argument("--profile-acceleration", type=int, default=5)
    grasp.add_argument("--max-abs-load", type=int, default=300)
    grasp.add_argument("--contact-load", type=int, default=80)
    grasp.add_argument("--contact-samples", type=int, default=3)
    grasp.add_argument("--max-contact-movement", type=int, default=3)
    grasp.add_argument("--max-ratio-deviation", type=float, default=0.05)
    grasp.add_argument("--goal-tolerance", type=int, default=30)
    grasp.add_argument("--open-tolerance", type=int, default=30)
    grasp.add_argument("--extra-close-ratio", type=float, default=0.01)
    grasp.add_argument("--max-close-ratio", type=float, default=0.95)
    grasp.add_argument("--sample-hz", type=float, default=10.0)
    grasp.add_argument("--step-timeout", type=float, default=1.5)
    grasp.add_argument("--initial-settle-seconds", type=float, default=1.0)
    grasp.add_argument("--extra-settle-seconds", type=float, default=0.5)
    grasp.set_defaults(func=stage_grasp)
    return root


def main():
    args = parser().parse_args()
    try:
        args.func(args)
    except (CalibrationError, KeyboardInterrupt) as exc:
        print(f"STOPPED: {exc}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
