#!/usr/bin/env python3
"""
Centered 270 degree rocking motion for a YZ-AIM servo.

This version uses Modbus speed mode instead of position moves. Position-move
retargeting caused endpoint dwell on this drive. Speed mode lets the script
continuously command velocity based on measured position:

    slow near endpoints, fastest near center, immediate direction flip at limit

Direction support is drive-dependent. By default this script writes signed RPM
values to the target-speed register. If the drive ignores signed speed, try
--direction-method dir-polarity.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from aim_servo import AIMServo


@dataclass(frozen=True)
class RockProfile:
    swing_deg: float = 270.0
    period_s: float = 6.0
    pulses_per_turn: int = 32768

    @property
    def amplitude_deg(self) -> float:
        return self.swing_deg / 2.0

    @property
    def amplitude_counts(self) -> int:
        return pulses_from_degrees(self.amplitude_deg, self.pulses_per_turn)

    @property
    def sine_peak_rpm(self) -> float:
        # For x = A*sin(wt), max speed is A*w. Convert deg/s to RPM.
        return (2.0 * math.pi * self.amplitude_deg / self.period_s) / 6.0


def pulses_from_degrees(degrees: float, pulses_per_turn: int) -> int:
    return int(round((degrees / 360.0) * pulses_per_turn))


def validate_profile(profile: RockProfile) -> None:
    if profile.swing_deg <= 0:
        raise ValueError("--swing-deg must be greater than zero")
    if profile.swing_deg > 360:
        raise ValueError("--swing-deg must be 360 degrees or less")
    if profile.period_s <= 0:
        raise ValueError("--period must be greater than zero")
    if profile.pulses_per_turn <= 0:
        raise ValueError("--pulses-per-turn must be greater than zero")


def derived_peak_rpm(profile: RockProfile, max_rpm: int) -> int:
    return max(1, min(max_rpm, int(round(profile.sine_peak_rpm))))


def write_signed_speed_rpm(servo: "AIMServo", rpm: int) -> None:
    servo.write_register(servo.REG_TARGET_SPEED, int(rpm) & 0xFFFF)


def write_directional_speed_rpm(
    servo: "AIMServo",
    rpm: int,
    direction_method: str,
    positive_dir_polarity: int,
) -> None:
    if direction_method == "signed":
        write_signed_speed_rpm(servo, rpm)
        return

    polarity = positive_dir_polarity if rpm >= 0 else 1 - positive_dir_polarity
    servo.set_dir_polarity(polarity)
    time.sleep(0.005)
    servo.set_target_speed(abs(rpm))


def speed_for_position(
    pos: int,
    center: int,
    amplitude_counts: int,
    direction: int,
    peak_rpm: int,
    min_rpm: int,
) -> int:
    if amplitude_counts <= 0:
        return 0

    normalized = abs((pos - center) / amplitude_counts)
    normalized = min(1.0, max(0.0, normalized))

    # Sine-position equivalent velocity: zero at endpoints, max at center.
    shape = math.sqrt(max(0.0, 1.0 - normalized * normalized))
    rpm = int(round(min_rpm + (peak_rpm - min_rpm) * shape))
    return direction * max(min_rpm, min(peak_rpm, rpm))


def run_rocking(
    servo: "AIMServo",
    profile: RockProfile,
    poll_hz: float,
    current_limit_a: float,
    peak_rpm: int | None,
    min_rpm: int,
    direction_method: str,
    positive_dir_polarity: int,
    duration_s: float | None,
    hold: bool,
) -> None:
    validate_profile(profile)

    if poll_hz <= 0:
        raise ValueError("--poll-hz must be greater than zero")
    if current_limit_a <= 0:
        raise ValueError("--current-limit must be greater than zero")
    if min_rpm < 0:
        raise ValueError("--min-rpm must not be negative")
    if direction_method not in ("signed", "dir-polarity"):
        raise ValueError("--direction-method must be signed or dir-polarity")
    if positive_dir_polarity not in (0, 1):
        raise ValueError("--positive-dir-polarity must be 0 or 1")
    if duration_s is not None and duration_s <= 0:
        raise ValueError("--duration must be greater than zero")

    servo.ensure_position_mode()
    center = servo.read_absolute_position()
    endpoint_a = center - profile.amplitude_counts
    endpoint_b = center + profile.amplitude_counts

    commanded_peak = peak_rpm
    if commanded_peak is None:
        commanded_peak = derived_peak_rpm(profile, servo.max_rpm)
    if not (1 <= commanded_peak <= servo.max_rpm):
        raise ValueError(f"--peak-rpm must be 1..{servo.max_rpm} RPM for model {servo.model}")
    if min_rpm >= commanded_peak:
        raise ValueError("--min-rpm must be less than --peak-rpm")

    servo.set_speed_mode()
    time.sleep(0.02)
    servo.enable_output(True)
    time.sleep(0.02)

    print(
        "Speed-mode rocking between "
        f"{endpoint_a} and {endpoint_b} counts "
        f"({profile.swing_deg:.1f} deg total, center={center})"
    )
    print(
        f"period={profile.period_s:.3f}s, peak_rpm={commanded_peak}, "
        f"min_rpm={min_rpm}, direction_method={direction_method}, "
        f"current_limit={current_limit_a:.3f} A"
    )
    print(
        "If the motor will not reverse, retry with "
        "--direction-method dir-polarity --positive-dir-polarity 0 or 1.\n"
    )

    direction = 1
    last_commanded_rpm: int | None = None
    interval_s = 1.0 / poll_hz
    start = time.monotonic()
    next_tick = start
    last_telemetry_time = 0.0
    peak_current = 0.0
    stopped_for_current = False

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start

            if duration_s is not None and elapsed >= duration_s:
                break

            current_a = servo.read_current_a()
            peak_current = max(peak_current, current_a)
            if current_a >= current_limit_a:
                stopped_for_current = True
                write_directional_speed_rpm(servo, 0, direction_method, positive_dir_polarity)
                servo.enable_output(False)
                print(
                    "\nCurrent limit exceeded: "
                    f"limit={current_limit_a:.3f} A, measured={current_a:.3f} A"
                )
                break

            pos = servo.read_absolute_position()
            if direction > 0 and pos >= endpoint_b:
                direction = -1
            elif direction < 0 and pos <= endpoint_a:
                direction = 1

            rpm = speed_for_position(
                pos=pos,
                center=center,
                amplitude_counts=profile.amplitude_counts,
                direction=direction,
                peak_rpm=commanded_peak,
                min_rpm=min_rpm,
            )

            if rpm != last_commanded_rpm:
                write_directional_speed_rpm(
                    servo,
                    rpm,
                    direction_method=direction_method,
                    positive_dir_polarity=positive_dir_polarity,
                )
                last_commanded_rpm = rpm

            if now - last_telemetry_time >= 0.25:
                actual_speed = servo.read_speed_rpm()
                alarm = servo.read_alarm_code()
                last_telemetry_time = now
                sys.stdout.write(
                    "\r"
                    f"t={elapsed:7.2f}s | "
                    f"cmd={rpm:5d} rpm | "
                    f"pos={pos:8d} | "
                    f"speed={actual_speed:7.1f} rpm | "
                    f"I={current_a:5.3f} A | "
                    f"peak={peak_current:5.3f} A | "
                    f"alarm=0x{alarm:04X}   "
                )
                sys.stdout.flush()

            next_tick += interval_s
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        try:
            write_directional_speed_rpm(servo, 0, direction_method, positive_dir_polarity)
            time.sleep(0.05)
            servo.set_position_mode()
            time.sleep(0.02)
        except Exception as exc:
            print(f"\nFailed to stop speed mode cleanly: {exc}", file=sys.stderr)

        if not hold and not stopped_for_current:
            try:
                servo.enable_output(False)
                print("\nOutput disabled.")
            except Exception as exc:
                print(f"\nFailed to disable output: {exc}", file=sys.stderr)
        elif hold:
            print("\nHolding output enabled.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Centered speed-mode rocking motion for a YZ-AIM servo",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-110")
    parser.add_argument("--addr", type=int, default=1, help="Drive address")
    parser.add_argument("--baud", type=int, default=19200, help="Baud rate")
    parser.add_argument("--model", default="42AIM15", help="Motor model for RPM limiting")
    parser.add_argument("--pulses-per-turn", type=int, default=32768)
    parser.add_argument("--swing-deg", type=float, default=270.0, help="Total rocking range")
    parser.add_argument("--period", type=float, default=6.0, help="Target seconds per full cycle")
    parser.add_argument("--poll-hz", type=float, default=20.0, help="Position/current poll rate")
    parser.add_argument("--current-limit", type=float, default=0.800, help="Auto-release current")
    parser.add_argument(
        "--peak-rpm",
        type=int,
        default=None,
        help="Peak commanded speed. Defaults from --period.",
    )
    parser.add_argument("--min-rpm", type=int, default=6, help="Minimum moving speed near endpoints")
    parser.add_argument(
        "--direction-method",
        choices=["signed", "dir-polarity"],
        default="signed",
        help="How to command reverse motion in speed mode",
    )
    parser.add_argument(
        "--positive-dir-polarity",
        type=int,
        choices=[0, 1],
        default=0,
        help="DIR polarity used for positive motion with --direction-method dir-polarity",
    )
    parser.add_argument("--duration", type=float, default=None, help="Optional run duration in seconds")
    parser.add_argument(
        "--hold",
        action="store_true",
        help="Leave output enabled on normal stop instead of releasing torque",
    )
    parser.add_argument("--verbose", action="store_true", help="Show raw serial frames")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    profile = RockProfile(
        swing_deg=args.swing_deg,
        period_s=args.period,
        pulses_per_turn=args.pulses_per_turn,
    )
    validate_profile(profile)

    try:
        from aim_servo import AIMServo
    except ModuleNotFoundError as exc:
        if exc.name == "serial":
            print("ERROR: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
            return 1
        raise

    servo = AIMServo(
        port=args.port,
        address=args.addr,
        baudrate=args.baud,
        verbose=args.verbose,
        pulses_per_turn=args.pulses_per_turn,
        model=args.model,
    )

    try:
        run_rocking(
            servo=servo,
            profile=profile,
            poll_hz=args.poll_hz,
            current_limit_a=args.current_limit,
            peak_rpm=args.peak_rpm,
            min_rpm=args.min_rpm,
            direction_method=args.direction_method,
            positive_dir_polarity=args.positive_dir_polarity,
            duration_s=args.duration,
            hold=args.hold,
        )
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        servo.close()


if __name__ == "__main__":
    raise SystemExit(main())
