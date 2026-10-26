#!/usr/bin/env python3
"""
Configure a YZ-AIM servo drive for external pulse + direction control.

This applies the known-good settings used by the Teensy pulse/direction
rocking firmware:
  - Modbus command channel temporarily enabled for setup
  - speed limit 1500 RPM
  - acceleration/feedforward 60098
  - electronic gear 16:1
  - special function 0x19 = 0, pulse + direction mode
  - Modbus command mode disabled again so external pulses are accepted
"""

from __future__ import annotations

import argparse
import time

from aim_servo import AIMServo


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Configure YZ-AIM drive for pulse + direction mode",
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-1130")
    parser.add_argument("--addr", type=int, default=1, help="Drive address")
    parser.add_argument("--baud", type=int, default=19200, help="Baud rate")
    parser.add_argument("--speed", type=int, default=1500, help="Drive speed limit register value")
    parser.add_argument("--accel", type=int, default=60098, help="Drive acceleration/feedforward value")
    parser.add_argument("--gear-num", type=int, default=16, help="Electronic gear numerator")
    parser.add_argument("--gear-den", type=int, default=1, help="Electronic gear denominator")
    parser.add_argument(
        "--enable-output",
        action="store_true",
        help="Also write output-enable register 0x01 = 1 after configuration",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save parameters to drive nonvolatile memory",
    )
    parser.add_argument("--verbose", action="store_true", help="Show raw serial frames")
    return parser


def print_registers(servo: AIMServo) -> None:
    regs = servo.read_registers(0x00, 26)
    for reg in (0x00, 0x01, 0x02, 0x03, 0x0A, 0x0B, 0x19):
        value = regs[reg]
        print(f"0x{reg:02X}={value} (0x{value:04X})")


def main() -> int:
    args = build_parser().parse_args()

    servo = AIMServo(
        port=args.port,
        address=args.addr,
        baudrate=args.baud,
        verbose=args.verbose,
    )

    try:
        print("Enabling Modbus setup access...")
        servo.enable_modbus(True)
        time.sleep(0.05)

        print(f"Setting speed limit: {args.speed}")
        servo.set_target_speed(args.speed)
        time.sleep(0.02)

        print(f"Setting acceleration/feedforward: {args.accel}")
        servo.set_acceleration(args.accel)
        time.sleep(0.02)

        print(f"Setting electronic gear: {args.gear_num}:{args.gear_den}")
        servo.set_electronic_gear(args.gear_num, args.gear_den)
        time.sleep(0.02)

        print("Setting special function 0x19 = 0, pulse + direction mode")
        servo.set_special_function(0)
        time.sleep(0.02)

        if args.enable_output:
            print("Writing output-enable register 0x01 = 1")
            servo.enable_output(True)
            time.sleep(0.02)

        if args.save:
            print("Saving parameters to nonvolatile memory")
            servo.save_parameters()

        print("Disabling Modbus motion control so external pulses are accepted...")
        servo.enable_modbus(False)
        time.sleep(0.05)

        print("\nVerification registers:")
        print_registers(servo)

        print("\nExpected pulse/dir values:")
        print("0x00=0, 0x02=1500, 0x03=60098, 0x0A=16, 0x0B=1, 0x19=0")
        return 0
    finally:
        servo.close()


if __name__ == "__main__":
    raise SystemExit(main())
