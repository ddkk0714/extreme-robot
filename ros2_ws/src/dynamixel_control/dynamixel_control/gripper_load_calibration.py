#!/usr/bin/env python3
"""Interactive, guarded load calibration for coupled XL430 IDs 3 and 4."""

import argparse
import csv
import json
import math
import signal
import statistics
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from dynamixel_sdk import (
    COMM_SUCCESS,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

DEVICE = "/dev/ttyUSB0"
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 2.0
DXL_IDS = (3, 4)
MODEL_NUMBER = 1060

ADDR_MODEL_NUMBER = 0
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_MOVING = 122
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

SYNC_READ_START = ADDR_TORQUE_ENABLE
SYNC_READ_LENGTH = ADDR_PRESENT_POSITION + 4 - SYNC_READ_START
PROFILE_ACCELERATION = 25
PROFILE_VELOCITY = 80
MAX_RATIO_STEP = 0.01
MIN_STEP_INTERVAL = 0.05
INITIAL_RATIO_LIMIT = 0.70
OPEN = {3: 1180, 4: 2510}
CLOSE_UNWRAPPED = {3: -164, 4: 1212}
SPAN = {3: 1344, 4: 1298}
OUTPUT_DIR = Path("/tmp/gripper_load_calibration")


class CalibrationError(RuntimeError):
    """A condition requiring an immediate safe stop."""


def signed(value, bits):
    """Interpret an unsigned value as two's-complement."""
    sign = 1 << (bits - 1)
    return value - (1 << bits) if value & sign else value


def le32(value):
    """Encode a 32-bit register value for GroupSyncWrite."""
    value &= 0xFFFFFFFF
    return [(value >> shift) & 0xFF for shift in (0, 8, 16, 24)]


def unwrap_position(previous_unwrapped, current_raw):
    """Continue a 0..4095 raw position across the wrap boundary."""
    previous_raw = previous_unwrapped % 4096
    delta = current_raw - previous_raw
    if delta > 2048:
        delta -= 4096
    elif delta < -2048:
        delta += 4096
    return previous_unwrapped + delta


def nearest_continuous(raw, references):
    """Choose the 4096-period image nearest to either endpoint reference."""
    candidates = [raw + 4096 * k for k in range(-2, 3)]
    return min(candidates, key=lambda value: min(
        abs(value - reference) for reference in references))


def goals_for_ratio(ratio):
    """Return distinct raw position goals for the logical close ratio."""
    if not 0.0 <= ratio <= 1.0:
        raise ValueError("ratio must be in [0, 1]")
    unwrapped3 = OPEN[3] - SPAN[3] * ratio
    goal4 = OPEN[4] - SPAN[4] * ratio
    return {3: round(unwrapped3) % 4096, 4: round(goal4) % 4096}


def ratio_for_position(dxl_id, position):
    """Convert a continuous measured position to logical close ratio."""
    return (OPEN[dxl_id] - position) / SPAN[dxl_id]


def validate_target_ratio(ratio, limit=INITIAL_RATIO_LIMIT):
    """Enforce the currently approved powered range."""
    if not 0.0 <= ratio <= limit:
        raise CalibrationError(
            f"ratio {ratio:.3f} outside approved range 0.00..{limit:.2f}")


@dataclass
class Sample:
    timestamp: float
    phase: str
    trial: int
    target_ratio: float
    id3_goal_raw: int
    id4_goal_raw: int
    id3_position_raw: int
    id3_position_unwrapped: int
    id4_position: int
    id3_ratio: float
    id4_ratio: float
    id3_load_raw: int
    id4_load_raw: int
    max_abs_load: int
    moving3: int
    moving4: int
    hardware_error3: int
    hardware_error4: int


class DynamixelBus:
    """Protocol 2.0 transport restricted to XL430 IDs 3 and 4."""

    def __init__(self, device=DEVICE):
        self.device = device
        self.port = PortHandler(device)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.reader = GroupSyncRead(
            self.port, self.packet, SYNC_READ_START, SYNC_READ_LENGTH)
        self.writer = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, 4)
        self.opened = False
        self.last_unwrapped = {}

    def open(self):
        if not self.port.openPort():
            raise CalibrationError(
                f"cannot open {self.device}; stop ROS/other serial owners "
                "first")
        self.opened = True
        if not self.port.setBaudRate(BAUD_RATE):
            self.close()
            raise CalibrationError(f"cannot set baud rate {BAUD_RATE}")
        for dxl_id in DXL_IDS:
            if not self.reader.addParam(dxl_id):
                self.close()
                raise CalibrationError(f"cannot register SyncRead ID {dxl_id}")

    def close(self):
        if self.opened:
            self.port.closePort()
            self.opened = False

    def _read2(self, dxl_id, address, label):
        value, result, error = self.packet.read2ByteTxRx(
            self.port, dxl_id, address)
        self._check_result(dxl_id, label, result, error)
        return value

    def _check_result(self, dxl_id, label, result, error):
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)}")
        if error:
            raise CalibrationError(
                f"ID {dxl_id} {label}: {self.packet.getRxPacketError(error)}")

    def verify_models(self):
        for dxl_id in DXL_IDS:
            model = self._read2(dxl_id, ADDR_MODEL_NUMBER, "model")
            if model != MODEL_NUMBER:
                raise CalibrationError(
                    f"ID {dxl_id}: model {model}, expected {MODEL_NUMBER}")

    def _data(self, dxl_id, address, length, label):
        if not self.reader.isAvailable(dxl_id, address, length):
            raise CalibrationError(f"ID {dxl_id}: missing {label} in SyncRead")
        return self.reader.getData(dxl_id, address, length)

    def read_states(self):
        result = self.reader.txRxPacket()
        if result != COMM_SUCCESS:
            raise CalibrationError(
                f"SyncRead: {self.packet.getTxRxResult(result)}")
        states = {}
        for dxl_id in DXL_IDS:
            raw32 = self._data(dxl_id, ADDR_PRESENT_POSITION, 4, "position")
            raw = raw32 % 4096
            if dxl_id not in self.last_unwrapped:
                references = (OPEN[dxl_id], CLOSE_UNWRAPPED[dxl_id])
                continuous = nearest_continuous(raw, references)
            else:
                continuous = unwrap_position(self.last_unwrapped[dxl_id], raw)
            self.last_unwrapped[dxl_id] = continuous
            states[dxl_id] = {
                "torque": self._data(
                    dxl_id, ADDR_TORQUE_ENABLE, 1, "torque enable"),
                "hardware_error": self._data(
                    dxl_id, ADDR_HARDWARE_ERROR, 1, "hardware error"),
                "profile_acceleration": self._data(
                    dxl_id, ADDR_PROFILE_ACCELERATION, 4,
                    "profile acceleration"),
                "profile_velocity": self._data(
                    dxl_id, ADDR_PROFILE_VELOCITY, 4, "profile velocity"),
                "moving": self._data(dxl_id, ADDR_MOVING, 1, "moving"),
                "load": signed(self._data(
                    dxl_id, ADDR_PRESENT_LOAD, 2, "present load"), 16),
                "position_raw": raw,
                "position_unwrapped": continuous,
                "ratio": ratio_for_position(dxl_id, continuous),
            }
        return states

    def write_goals(self, goals):
        self.writer.clearParam()
        try:
            for dxl_id in DXL_IDS:
                if not self.writer.addParam(dxl_id, le32(goals[dxl_id])):
                    raise CalibrationError(
                        f"cannot queue goal for ID {dxl_id}")
            result = self.writer.txPacket()
            if result != COMM_SUCCESS:
                raise CalibrationError(
                    f"SyncWrite goal: {self.packet.getTxRxResult(result)}")
        finally:
            self.writer.clearParam()

    def set_torque(self, enabled):
        value = 1 if enabled else 0
        failures = []
        for dxl_id in DXL_IDS:
            result, error = self.packet.write1ByteTxRx(
                self.port, dxl_id, ADDR_TORQUE_ENABLE, value)
            try:
                self._check_result(dxl_id, "torque", result, error)
            except CalibrationError as exc:
                failures.append(str(exc))
        if failures:
            raise CalibrationError("; ".join(failures))


class CalibrationSession:
    """Safety state machine and interactive calibration session."""

    def __init__(self, bus, args):
        self.bus = bus
        self.args = args
        self.armed = args.armed
        self.torque_on = False
        self.target_ratio = math.nan
        self.last_goals = {3: -1, 4: -1}
        self.samples = []
        self.trials = []
        self.stop_requested = False
        self.hold_validated = False
        self.output_dir = args.output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def validate_state(self, states, target_ratio=None, previous=None):
        for dxl_id, state in states.items():
            if state["hardware_error"] != 0:
                raise CalibrationError(
                    f"ID {dxl_id}: hardware error "
                    f"0x{state['hardware_error']:02X}")
            if state["profile_acceleration"] != PROFILE_ACCELERATION:
                raise CalibrationError(
                    f"ID {dxl_id}: Profile Acceleration is "
                    f"{state['profile_acceleration']}, required 25")
            if state["profile_velocity"] != PROFILE_VELOCITY:
                raise CalibrationError(
                    f"ID {dxl_id}: Profile Velocity is "
                    f"{state['profile_velocity']}, required 80")
            if abs(state["load"]) > self.args.load_stop_threshold:
                raise CalibrationError(
                    f"ID {dxl_id}: abs(Present Load) {abs(state['load'])} "
                    "exceeds protection limit "
                    f"{self.args.load_stop_threshold}")
        mismatch = abs(states[3]["ratio"] - states[4]["ratio"])
        if mismatch > self.args.max_ratio_difference:
            raise CalibrationError(
                f"coupled ratio mismatch {mismatch:.3f} exceeds "
                f"{self.args.max_ratio_difference:.3f}")
        if target_ratio is not None:
            for dxl_id in DXL_IDS:
                error = abs(states[dxl_id]["ratio"] - target_ratio)
                if error > self.args.path_ratio_tolerance:
                    raise CalibrationError(
                        f"ID {dxl_id}: path ratio error {error:.3f} exceeds "
                        f"{self.args.path_ratio_tolerance:.3f}")
        if previous is not None and target_ratio is not None:
            changes = {
                dxl_id: abs(states[dxl_id]["position_unwrapped"] -
                            previous[dxl_id]["position_unwrapped"])
                for dxl_id in DXL_IDS
            }
            moving = {dxl_id: changes[dxl_id] >= self.args.min_motion_ticks
                      for dxl_id in DXL_IDS}
            if moving[3] != moving[4]:
                raise CalibrationError(
                    f"asymmetric motion: ID3 delta={changes[3]}, "
                    f"ID4 delta={changes[4]}")

    def make_sample(self, states, phase="status", trial=0):
        return Sample(
            time.time(), phase, trial,
            self.target_ratio if math.isfinite(self.target_ratio) else -1.0,
            self.last_goals[3], self.last_goals[4],
            states[3]["position_raw"], states[3]["position_unwrapped"],
            states[4]["position_unwrapped"], states[3]["ratio"],
            states[4]["ratio"], states[3]["load"], states[4]["load"],
            max(abs(states[3]["load"]), abs(states[4]["load"])),
            states[3]["moving"], states[4]["moving"],
            states[3]["hardware_error"], states[4]["hardware_error"])

    def read_checked(
            self, phase="status", trial=0, target=None, previous=None):
        states = self.bus.read_states()
        self.validate_state(states, target, previous)
        sample = self.make_sample(states, phase, trial)
        self.samples.append(sample)
        self.print_states(states)
        return states

    @staticmethod
    def print_states(states):
        print(
            "ID  raw  unwrapped  ratio   load  moving  torque  hwerr  "
            "accel/vel")
        for dxl_id in DXL_IDS:
            state = states[dxl_id]
            print(
                f"{dxl_id:2d} {state['position_raw']:4d} "
                f"{state['position_unwrapped']:9d} {state['ratio']:6.3f} "
                f"{state['load']:6d} {state['moving']:7d} "
                f"{state['torque']:7d} 0x{state['hardware_error']:02X} "
                f"{state['profile_acceleration']}/{state['profile_velocity']}")

    def confirm_motion(self, ratio):
        goals = goals_for_ratio(ratio)
        print(
            f"target ratio={ratio:.3f}: ID3 raw={goals[3]}, "
            f"ID4 raw={goals[4]}")
        answer = input("Type MOVE to send this gradual motion: ")
        if answer != "MOVE":
            raise CalibrationError("motion not confirmed")
        return goals

    def torque_enable_hold(self):
        if not self.armed:
            raise CalibrationError(
                "restart with --armed before any goal write")
        states = self.read_checked("torque-hold-precheck")
        if any(state["torque"] for state in states.values()):
            raise CalibrationError("torque already enabled unexpectedly")
        print("Hold goals use current positions:")
        hold = {i: states[i]["position_raw"] for i in DXL_IDS}
        print(f"ID3={hold[3]}, ID4={hold[4]}")
        answer = input(
            "Type HOLD to write current goals and enable torque: ")
        if answer != "HOLD":
            raise CalibrationError("torque hold not confirmed")
        self.bus.write_goals(hold)
        self.last_goals = hold
        self.bus.set_torque(True)
        self.torque_on = True
        start = {i: dict(states[i]) for i in DXL_IDS}
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            current = self.read_checked("torque-hold")
            for dxl_id in DXL_IDS:
                jump = abs(current[dxl_id]["position_unwrapped"] -
                           start[dxl_id]["position_unwrapped"])
                if jump > self.args.hold_jump_ticks:
                    raise CalibrationError(
                        f"ID {dxl_id}: moved {jump} ticks during hold")
            time.sleep(0.1)
        if not self.hold_validated:
            self.bus.set_torque(False)
            self.torque_on = False
            self.hold_validated = True
            print("Phase B hold passed; torque automatically disabled.")
            print("Do not continue to Phase C without separate user approval.")
        else:
            print(
                "Validated hold enabled for an explicitly approved Phase C "
                "move.")

    def move_gradually(self, target):
        if not self.armed or not self.torque_on:
            raise CalibrationError("--armed and torque-on are required")
        validate_target_ratio(target, self.args.max_ratio)
        self.confirm_motion(target)
        current = self.bus.read_states()
        self.validate_state(current)
        start_ratio = (current[3]["ratio"] + current[4]["ratio"]) / 2.0
        direction = 1.0 if target > start_ratio else -1.0
        ratio = start_ratio
        stalled_since = None
        while direction * (target - ratio) > 1e-9:
            ratio += direction * min(MAX_RATIO_STEP, abs(target - ratio))
            goals = goals_for_ratio(ratio)
            self.bus.write_goals(goals)
            self.last_goals = goals
            self.target_ratio = ratio
            time.sleep(max(MIN_STEP_INTERVAL, self.args.step_interval))
            states = self.bus.read_states()
            self.validate_state(states, ratio, current)
            changes = [abs(states[i]["position_unwrapped"] -
                           current[i]["position_unwrapped"]) for i in DXL_IDS]
            if max(changes) < self.args.min_motion_ticks:
                stalled_since = stalled_since or time.monotonic()
                if time.monotonic() - stalled_since > self.args.stall_seconds:
                    raise CalibrationError(
                        "goal changes but both positions are stalled")
            else:
                stalled_since = None
            self.samples.append(self.make_sample(states, "move"))
            self.print_states(states)
            current = states

    def collect_trial(self, phase, trial):
        if phase not in {"empty", "grasp", "drop"}:
            raise CalibrationError("phase must be empty, grasp, or drop")
        print("Waiting 0.5 s; motion spike is excluded.")
        time.sleep(0.5)
        collected = []
        count = max(10, round(self.args.sample_hz))
        for _ in range(count):
            self.read_checked(phase, trial)
            collected.append(self.samples[-1])
            time.sleep(1.0 / self.args.sample_hz)
        stable = collected[len(collected) // 4:]
        representative = statistics.median(s.max_abs_load for s in stable)
        summary = {
            "phase": phase,
            "trial": trial,
            "median_max_abs_load": representative,
            "min": min(s.max_abs_load for s in stable),
            "max": max(s.max_abs_load for s in stable),
            "id3_median_load": statistics.median(
                s.id3_load_raw for s in stable),
            "id4_median_load": statistics.median(
                s.id4_load_raw for s in stable),
        }
        self.trials.append(summary)
        print(json.dumps(summary, indent=2))
        self.save()

    def thresholds(self):
        groups = {
            phase: [row["median_max_abs_load"] for row in self.trials
                    if row["phase"] == phase]
            for phase in ("empty", "grasp", "drop")
        }
        if any(len(values) < 5 for values in groups.values()):
            print("At least five trials per group are required.")
            return
        empty_max = max(groups["empty"])
        grasp_min = min(groups["grasp"])
        drop_max = max(groups["drop"])
        low_max = max(empty_max, drop_max)
        margin = grasp_min - low_max
        result = dict(empty_max=empty_max, grasp_min=grasp_min,
                      drop_max=drop_max, low_max=low_max, margin=margin)
        if margin > 0:
            drop = round(low_max + 0.30 * margin)
            grasp = round(low_max + 0.60 * margin)
            if low_max < drop < grasp < grasp_min:
                result.update(drop_load_thresh=drop, grasp_load_thresh=grasp)
            else:
                result["threshold_status"] = (
                    "rounding left insufficient separation")
        else:
            result["threshold_status"] = "groups overlap; no proposal"
        print(json.dumps(result, indent=2))

    def save(self):
        fields = list(asdict(self.samples[0]).keys()) if self.samples else []
        if fields:
            raw_path = self.output_dir / "raw_samples.csv"
            with raw_path.open("w", newline="") as out:
                writer = csv.DictWriter(out, fieldnames=fields)
                writer.writeheader()
                writer.writerows(asdict(sample) for sample in self.samples)
        if self.trials:
            summary_path = self.output_dir / "trial_summary.csv"
            with summary_path.open("w", newline="") as out:
                writer = csv.DictWriter(out, fieldnames=list(self.trials[0]))
                writer.writeheader()
                writer.writerows(self.trials)
        session = {
            "device": self.bus.device,
            "baud_rate": BAUD_RATE,
            "protocol": PROTOCOL_VERSION,
            "ids": list(DXL_IDS),
            "profile_acceleration": PROFILE_ACCELERATION,
            "profile_velocity": PROFILE_VELOCITY,
            "load_unit": (
                "XL430 Present Load raw; 0.1% inferred load per count"),
            "load_stop_threshold": self.args.load_stop_threshold,
            "approved_ratio_limit": self.args.max_ratio,
            "trials": self.trials,
        }
        (self.output_dir / "session.json").write_text(
            json.dumps(session, indent=2) + "\n")

    def emergency_stop(self, reason="emergency stop"):
        self.stop_requested = True
        try:
            self.bus.set_torque(False)
        finally:
            self.torque_on = False
        print(f"{reason}: torque-disable sent to IDs 3 and 4")

    def cleanup(self):
        try:
            self.bus.set_torque(False)
        except Exception as exc:
            print(f"WARNING: final torque-disable failed: {exc}")
        self.torque_on = False
        try:
            states = self.bus.read_states()
            for dxl_id in DXL_IDS:
                print(
                    f"final ID {dxl_id}: torque={states[dxl_id]['torque']} "
                    f"hwerr=0x{states[dxl_id]['hardware_error']:02X}")
        except Exception as exc:
            print(f"WARNING: final state unreadable: {exc}")
        self.save()
        self.bus.close()
        print("Goal transmission stopped; port closed.")

    def command_loop(self):
        print("Commands: status, torque-on, torque-off, goto R, open, sample, "
              "measure PHASE TRIAL, thresholds, emergency-stop, quit")
        while not self.stop_requested:
            try:
                parts = input("gripper-cal> ").strip().split()
            except EOFError:
                parts = ["quit"]
            if not parts:
                continue
            command = parts[0]
            if command == "status":
                self.read_checked()
            elif command == "torque-on":
                self.torque_enable_hold()
            elif command == "torque-off":
                self.emergency_stop("torque-off")
                self.stop_requested = False
            elif command == "goto" and len(parts) == 2:
                self.move_gradually(float(parts[1]))
            elif command == "open":
                self.move_gradually(0.0)
            elif command == "sample":
                self.read_checked("sample")
            elif command == "measure" and len(parts) == 3:
                self.collect_trial(parts[1], int(parts[2]))
            elif command == "thresholds":
                self.thresholds()
            elif command == "emergency-stop":
                self.emergency_stop()
            elif command == "quit":
                break
            else:
                print("Unknown or incomplete command; no motion sent.")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Guarded ID3/ID4 XL430 Present Load calibration")
    parser.add_argument("--device", default=DEVICE)
    parser.add_argument("--armed", action="store_true",
                        help="permit confirmed goal writes and torque-on")
    parser.add_argument("--read-only", action="store_true",
                        help=(
                            "Phase A status check; sends no goal and no "
                            "torque-on"))
    parser.add_argument("--max-ratio", type=float, default=INITIAL_RATIO_LIMIT)
    parser.add_argument("--load-stop-threshold", type=int, default=300,
                        help=(
                            "temporary mechanical protection limit, not "
                            "grasp threshold"))
    parser.add_argument("--max-ratio-difference", type=float, default=0.05)
    parser.add_argument("--path-ratio-tolerance", type=float, default=0.08)
    parser.add_argument(
        "--step-interval", type=float, default=MIN_STEP_INTERVAL)
    parser.add_argument("--stall-seconds", type=float, default=0.5)
    parser.add_argument("--min-motion-ticks", type=int, default=1)
    parser.add_argument("--hold-jump-ticks", type=int, default=10)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    return parser


def validate_arguments(args):
    if args.max_ratio > INITIAL_RATIO_LIMIT:
        raise CalibrationError(
            "this build refuses --max-ratio above 0.70; range extension "
            "needs review")
    validate_target_ratio(0.0, args.max_ratio)
    if args.step_interval < MIN_STEP_INTERVAL:
        raise CalibrationError("step interval must be at least 0.05 seconds")
    if args.load_stop_threshold <= 0:
        raise CalibrationError("load stop threshold must be positive")
    if args.read_only and args.armed:
        raise CalibrationError(
            "--read-only and --armed are mutually exclusive")


def main():
    args = build_parser().parse_args()
    bus = DynamixelBus(args.device)
    session = CalibrationSession(bus, args)
    old_handlers = {}

    def request_stop(signum, _frame):
        session.stop_requested = True
        raise KeyboardInterrupt(f"signal {signum}")

    try:
        validate_arguments(args)
        print(
            "WARNING: stop ROS Dynamixel nodes; this tool requires exclusive "
            "port access.")
        print(
            "EEPROM, operating mode, ID, and baud registers are never "
            "written.")
        print("Present Load protection limit is not a grasp threshold.")
        bus.open()
        bus.verify_models()
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)
        states = session.read_checked("phase-a")
        if any(state["torque"] for state in states.values()):
            raise CalibrationError(
                "startup requires torque disabled on both motors")
        print("Phase A read-only check passed; no Goal Position was sent.")
        if not args.read_only:
            session.command_loop()
    except (CalibrationError, KeyboardInterrupt, ValueError) as exc:
        print(f"STOPPED: {exc}")
    finally:
        if bus.opened:
            session.cleanup()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()
