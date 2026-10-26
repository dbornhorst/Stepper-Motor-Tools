#!/usr/bin/env python3
"""
aim_servo.py

YZ-AIM integrated servo controller over USB-RS485.

Features:
- Position mode init
- Status / monitor
- Relative moves
- Software absolute moves
- Zero current position
- Continuous spin mode with live speed adjustment
- Current monitoring
- Command help/examples
- Model-based RPM limiting

Notes:
- For this tested setup, move_abs is implemented as software absolute motion.
- For model 42AIM15, speed is clamped to 1500 RPM based on datasheet limits.
- Spin direction switching in Modbus speed mode is not yet verified.

Install:
    pip install pyserial

Examples:
    python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 init
    python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 status
    python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_rel 1000 --wait
    python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_abs 0 --wait
    python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 spin --rpm 300 --dir cw
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass

import serial


HELP_TEXT = """
Common commands
---------------

Startup after power cycle:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 init

Read status:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 status

Zero current position:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 zero_here

Move relative:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_rel 1000 --wait
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_rel -1000 --wait

Move absolute:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_abs 0 --wait
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_abs 2000 --wait

Move by degrees / turns:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_deg 90 --wait
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_turns 0.5 --wait

Continuous spin:
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 spin --rpm 300
  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 spin --rpm 300 --dir cw

  Note: --dir is currently retained as an experimental argument.
  On this tested setup, Modbus speed-mode direction switching was not verified.

Spin mode keys
--------------
  ] or +   increase speed by step
  [ or -   decrease speed by step
  0        stop spin (set RPM = 0)
  q        quit spin mode
  e        disable output (release torque)
  E        enable output again

Notes
-----
- move_abs is software absolute motion:
  current position is read, delta is computed, then move_rel(delta) is used.
- move_abs_hw is retained only as a hardware test path for the documented 0x7B command.
- zero_here rewrites the absolute position counter to 0.
- spin direction is set before entering speed mode using DIR polarity register behavior.
"""


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    return frame + struct.pack("<H", crc16_modbus(frame))


def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    body = frame[:-2]
    rx_crc = struct.unpack("<H", frame[-2:])[0]
    return rx_crc == crc16_modbus(body)


def int32_to_be_bytes(value: int) -> bytes:
    return struct.pack(">i", int(value))


def be_bytes_to_int32(data: bytes) -> int:
    return struct.unpack(">i", data)[0]


def u16_to_i16(x: int) -> int:
    return x - 0x10000 if x & 0x8000 else x


def u32_words_to_i32(low_word: int, high_word: int) -> int:
    value = ((high_word & 0xFFFF) << 16) | (low_word & 0xFFFF)
    return value - 0x100000000 if value & 0x80000000 else value


@dataclass
class ServoStatus:
    alarm_code: int
    output_enabled: int
    modbus_enabled: int
    speed_rpm: float
    current_a: float
    absolute_position: int
    voltage_v: float
    temperature_c: int

    def __str__(self) -> str:
        return (
            f"alarm=0x{self.alarm_code:04X}, "
            f"output_enabled={self.output_enabled}, "
            f"modbus_enabled={self.modbus_enabled}, "
            f"speed_rpm={self.speed_rpm:.1f}, "
            f"current={self.current_a:.3f}A, "
            f"abs_position={self.absolute_position}, "
            f"voltage={self.voltage_v:.2f}V, "
            f"temp={self.temperature_c}C"
        )


class AIMServo:
    # Registers
    REG_MODBUS_ENABLE = 0x00
    REG_OUTPUT_ENABLE = 0x01
    REG_TARGET_SPEED = 0x02
    REG_ACCEL = 0x03
    REG_DIR_POLARITY = 0x09
    REG_ELEC_GEAR_NUM = 0x0A
    REG_ELEC_GEAR_DEN = 0x0B
    REG_ALARM = 0x0E
    REG_SYSTEM_CURRENT = 0x0F
    REG_CURRENT_SPEED = 0x10
    REG_SYSTEM_VOLTAGE = 0x11
    REG_SYSTEM_TEMP = 0x12
    REG_SAVE_FLAG = 0x14
    REG_DEVICE_ADDRESS = 0x15
    REG_ABS_POS_LOW = 0x16
    REG_ABS_POS_HIGH = 0x17
    REG_STILL_MAX_OUTPUT = 0x18
    REG_SPECIAL_FUNCTION = 0x19

    # Function codes
    FC_READ = 0x03
    FC_WRITE_SINGLE = 0x06
    FC_WRITE_MULTI = 0x10
    FC_MOVE_REL = 0x78
    FC_MOVE_ABS = 0x7B  # kept for testing only

    # Modes / special function values
    SPECIAL_POSITION_MODE = 0
    SPECIAL_SPEED_MODE = 3

    ALARM_TEXT = {
        0x10: "battery off alarm",
        0x12: "overflow alarm",
        0x14: "blocked rotation alarm",
        0x15: "overvoltage / overpressure alarm",
    }

    def __init__(
        self,
        port: str,
        address: int = 1,
        baudrate: int = 19200,
        timeout: float = 0.25,  
        verbose: bool = False,
        pulses_per_turn: int = 32768,
        model: str = "42AIM15",
    ) -> None:
        self.address = address & 0xFF
        self.verbose = verbose
        self.pulses_per_turn = pulses_per_turn
        self.model = model

        # Based on datasheet model table
        self.max_rpm_by_model = {
            "42AIM15": 1500,
            "42AIM10": 1500,
            "57AIM15": 1500,
            "57AIM15H": 3000,
            "57AIM30": 1500,
            "57AIM30H": 3000,
            "60AIM25": 1500,
            "60AIM25H": 3000,
        }
        self.max_rpm = self.max_rpm_by_model.get(model, 1500)

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def _exchange(self, request_wo_crc: bytes, min_reply_len: int = 5) -> bytes:
        frame = append_crc(request_wo_crc)

        if self.verbose:
            print(f"TX: {frame.hex(' ')}")

        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

        time.sleep(0.02)
        reply = self.ser.read(256)

        if self.verbose and reply:
            print(f"RX: {reply.hex(' ')}")

        if not reply:
            raise TimeoutError("No reply from drive")

        if not check_crc(reply):
            raise ValueError(f"Bad CRC reply: {reply.hex(' ')}")

        if reply[0] != self.address:
            raise ValueError(
                f"Unexpected slave address {reply[0]}, expected {self.address}. "
                f"Reply: {reply.hex(' ')}"
            )

        if len(reply) < min_reply_len:
            raise ValueError(
                f"Reply shorter than expected for this command: {reply.hex(' ')}"
            )

        return reply

    def read_registers(self, start_reg: int, count: int) -> list[int]:
        req = struct.pack(">BBHH", self.address, self.FC_READ, start_reg, count)
        reply = self._exchange(req, min_reply_len=5 + 2 * count)

        if reply[1] != self.FC_READ:
            raise ValueError(f"Unexpected read reply: {reply.hex(' ')}")

        byte_count = reply[2]
        expected = count * 2
        if byte_count != expected:
            raise ValueError(
                f"Unexpected byte count {byte_count}, expected {expected}. "
                f"Reply: {reply.hex(' ')}"
            )

        regs = []
        for i in range(count):
            off = 3 + 2 * i
            regs.append(struct.unpack(">H", reply[off:off + 2])[0])
        return regs

    def write_register(self, reg: int, value: int) -> None:
        req = struct.pack(">BBHH", self.address, self.FC_WRITE_SINGLE, reg, value & 0xFFFF)
        reply = self._exchange(req, min_reply_len=8)

        if reply[1] != self.FC_WRITE_SINGLE:
            raise ValueError(f"Unexpected write reply: {reply.hex(' ')}")

        r_reg = struct.unpack(">H", reply[2:4])[0]
        r_val = struct.unpack(">H", reply[4:6])[0]
        if r_reg != reg or r_val != (value & 0xFFFF):
            raise ValueError(
                f"Write echo mismatch. Sent reg=0x{reg:04X} val=0x{value & 0xFFFF:04X}, "
                f"got reply: {reply.hex(' ')}"
            )

    def write_registers(self, start_reg: int, values: list[int]) -> None:
        if not values:
            raise ValueError("values must not be empty")

        count = len(values)
        byte_count = count * 2
        payload = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)

        req = struct.pack(
            ">BBHHB",
            self.address,
            self.FC_WRITE_MULTI,
            start_reg,
            count,
            byte_count,
        ) + payload

        reply = self._exchange(req, min_reply_len=8)

        if reply[1] != self.FC_WRITE_MULTI:
            raise ValueError(f"Unexpected write_registers reply: {reply.hex(' ')}")

        r_start = struct.unpack(">H", reply[2:4])[0]
        r_count = struct.unpack(">H", reply[4:6])[0]
        if r_start != start_reg or r_count != count:
            raise ValueError(
                f"Write-multiple echo mismatch. Sent start=0x{start_reg:04X}, count={count}, "
                f"got reply: {reply.hex(' ')}"
            )

    # -------- basic control --------

    def enable_modbus(self, enabled: bool = True) -> None:
        self.write_register(self.REG_MODBUS_ENABLE, 1 if enabled else 0)

    def enable_output(self, enabled: bool = True) -> None:
        self.write_register(self.REG_OUTPUT_ENABLE, 1 if enabled else 0)

    def set_target_speed(self, rpm: int) -> None:
        if not (0 <= rpm <= self.max_rpm):
            raise ValueError(f"Speed must be 0..{self.max_rpm} RPM for model {self.model}")
        self.write_register(self.REG_TARGET_SPEED, rpm)

    def set_acceleration(self, accel: int) -> None:
        if not (0 <= accel <= 60098):
            raise ValueError("Acceleration must be 0..60098")
        self.write_register(self.REG_ACCEL, accel)

    def set_dir_polarity(self, value: int) -> None:
        if value not in (0, 1):
            raise ValueError("DIR polarity must be 0 or 1")
        self.write_register(self.REG_DIR_POLARITY, value)

    def set_special_function(self, value: int) -> None:
        self.write_register(self.REG_SPECIAL_FUNCTION, value)

    def set_electronic_gear(self, numerator: int, denominator: int) -> None:
        if not (0 <= numerator <= 65535):
            raise ValueError("Numerator must be 0..65535")
        if not (1 <= denominator <= 65535):
            raise ValueError("Denominator must be 1..65535")
        self.write_register(self.REG_ELEC_GEAR_NUM, numerator)
        time.sleep(0.02)
        self.write_register(self.REG_ELEC_GEAR_DEN, denominator)

    def save_parameters(self) -> None:
        self.write_register(self.REG_SAVE_FLAG, 1)
        time.sleep(0.1)

    def set_comm_mode_persistent(self) -> None:
        # Manual notes that electronic gear numerator = 0 enables power-up comm control behavior
        self.write_register(self.REG_ELEC_GEAR_NUM, 0)

    def set_position_mode(self) -> None:
        self.set_special_function(self.SPECIAL_POSITION_MODE)

    def set_speed_mode(self) -> None:
        self.set_special_function(self.SPECIAL_SPEED_MODE)

    # -------- reads --------

    def read_alarm_code(self) -> int:
        return self.read_registers(self.REG_ALARM, 1)[0]

    def read_modbus_enable(self) -> int:
        return self.read_registers(self.REG_MODBUS_ENABLE, 1)[0]

    def read_output_enable(self) -> int:
        return self.read_registers(self.REG_OUTPUT_ENABLE, 1)[0]

    def read_current_a(self) -> float:
        raw = self.read_registers(self.REG_SYSTEM_CURRENT, 1)[0]
        return raw / 2000.0

    def read_speed_rpm(self) -> float:
        raw = self.read_registers(self.REG_CURRENT_SPEED, 1)[0]
        return u16_to_i16(raw) / 10.0

    def read_voltage_v(self) -> float:
        raw = self.read_registers(self.REG_SYSTEM_VOLTAGE, 1)[0]
        return raw / 327.0

    def read_temperature_c(self) -> int:
        return self.read_registers(self.REG_SYSTEM_TEMP, 1)[0]

    def read_absolute_position(self) -> int:
        low, high = self.read_registers(self.REG_ABS_POS_LOW, 2)
        return u32_words_to_i32(low, high)

    def read_status(self) -> ServoStatus:
        return ServoStatus(
            alarm_code=self.read_alarm_code(),
            output_enabled=self.read_output_enable(),
            modbus_enabled=self.read_modbus_enable(),
            speed_rpm=self.read_speed_rpm(),
            current_a=self.read_current_a(),
            absolute_position=self.read_absolute_position(),
            voltage_v=self.read_voltage_v(),
            temperature_c=self.read_temperature_c(),
        )

    def alarm_text(self, code: int) -> str:
        return self.ALARM_TEXT.get(code, "unknown / no listed alarm")

    # -------- init / zero / motion --------

    def initialize_basic_position_mode(
        self,
        target_speed_rpm: int = 1500,
        acceleration: int = 5000,
        persistent_comm_mode: bool = True,
        enable_output: bool = True,
    ) -> None:
        self.enable_modbus(True)
        time.sleep(0.05)

        self.set_position_mode()
        time.sleep(0.02)

        self.set_acceleration(acceleration)
        time.sleep(0.02)

        self.set_target_speed(target_speed_rpm)
        time.sleep(0.02)

        if persistent_comm_mode:
            self.set_comm_mode_persistent()
            time.sleep(0.02)

        if enable_output:
            self.enable_output(True)
            time.sleep(0.02)

        self.save_parameters()

    def move_relative(self, delta_pulses: int) -> int:
        self.ensure_position_mode()

        req = bytes([self.address, self.FC_MOVE_REL]) + int32_to_be_bytes(delta_pulses)
        reply = self._exchange(req, min_reply_len=8)

        if reply[1] != self.FC_MOVE_REL:
            raise ValueError(f"Unexpected move_relative reply: {reply.hex(' ')}")

        ack = be_bytes_to_int32(bytes([reply[4], reply[5], reply[2], reply[3]]))
        return ack

    def move_absolute(self, target_pulses: int) -> int:
        """
        Software absolute move.

        Bench-verified behavior on this setup:
        - relative motion works
        - absolute position readback works
        - direct hardware absolute motion command (0x7B) is acknowledged but did
        not produce reliable physical motion

        So move_absolute() is implemented by:
        1. reading the current absolute position
        2. computing delta = target - current
        3. issuing move_relative(delta)
        """
        self.ensure_position_mode()

        current = self.read_absolute_position()
        delta = int(target_pulses) - int(current)

        if self.verbose:
            print(
                f"move_absolute software mode: current={current}, "
                f"target={target_pulses}, delta={delta}"
            )

        return self.move_relative(delta)

    def move_absolute_hw(self, target_pulses: int) -> int:
        """
        Direct hardware absolute command via 0x7B.
        Kept for testing.
        """
        req = bytes([self.address, self.FC_MOVE_ABS]) + int32_to_be_bytes(target_pulses)
        reply = self._exchange(req, min_reply_len=8)

        if reply[1] != self.FC_MOVE_ABS:
            raise ValueError(f"Unexpected move_absolute_hw reply: {reply.hex(' ')}")

        ack = be_bytes_to_int32(bytes([reply[4], reply[5], reply[2], reply[3]]))
        return ack

    def set_absolute_counter(self, position_counts: int) -> None:
        value_u32 = position_counts & 0xFFFFFFFF
        low_word = value_u32 & 0xFFFF
        high_word = (value_u32 >> 16) & 0xFFFF

        self.wait_until_stopped(timeout_s=3.0)
        self.write_register(self.REG_ELEC_GEAR_NUM, 0)
        time.sleep(0.02)
        self.write_registers(self.REG_ABS_POS_LOW, [low_word, high_word])
        time.sleep(0.02)

    def zero_here(self) -> None:
        self.ensure_position_mode()
        self.set_absolute_counter(0)

    def pulses_from_degrees(self, degrees: float) -> int:
        return int(round((degrees / 360.0) * self.pulses_per_turn))

    def pulses_from_turns(self, turns: float) -> int:
        return int(round(turns * self.pulses_per_turn))

    def move_degrees_relative(self, degrees: float) -> int:
        return self.move_relative(self.pulses_from_degrees(degrees))

    def move_turns_relative(self, turns: float) -> int:
        return self.move_relative(self.pulses_from_turns(turns))

    # -------- wait / monitor --------

    def wait_until_stopped(
        self,
        speed_threshold_rpm: float = 1.0,
        hold_time_s: float = 0.20,
        timeout_s: float = 10.0,
        poll_interval_s: float = 0.05,
    ) -> bool:
        start = time.time()
        below_since = None

        while True:
            speed = abs(self.read_speed_rpm())
            now = time.time()

            if speed <= speed_threshold_rpm:
                if below_since is None:
                    below_since = now
                elif now - below_since >= hold_time_s:
                    return True
            else:
                below_since = None

            if now - start > timeout_s:
                return False

            time.sleep(poll_interval_s)

    def wait_until_position(
        self,
        target_pulses: int,
        tolerance_pulses: int = 10,
        hold_time_s: float = 0.10,
        timeout_s: float = 10.0,
        poll_interval_s: float = 0.05,
    ) -> bool:
        start = time.time()
        in_tol_since = None

        while True:
            pos = self.read_absolute_position()
            err = abs(pos - target_pulses)
            now = time.time()

            if err <= tolerance_pulses:
                if in_tol_since is None:
                    in_tol_since = now
                elif now - in_tol_since >= hold_time_s:
                    return True
            else:
                in_tol_since = None

            if now - start > timeout_s:
                return False

            time.sleep(poll_interval_s)

    def monitor(self, interval_s: float = 0.25) -> None:
        try:
            while True:
                st = self.read_status()
                extra = f" ({self.alarm_text(st.alarm_code)})" if st.alarm_code else ""
                print(f"{st}{extra}")
                time.sleep(interval_s)
        except KeyboardInterrupt:
            print("\nStopped.")

    # -------- spin mode --------

    def configure_spin_direction(self, direction: str) -> None:
        """
        Spin direction via Modbus speed mode is not yet bench-verified on this drive.

        The manual's DIR polarity register appears tied to external pulse/direction
        behavior, and changing it did not produce reliable cw/ccw changes in tested
        Modbus speed mode.

        For now this function is a no-op except for validation.
        """
        direction = direction.lower()
        if direction not in ("cw", "ccw"):
            raise ValueError("direction must be 'cw' or 'ccw'")

    def start_spin_mode(self, rpm: int, direction: str = "cw") -> None:
        self.enable_modbus(True)
        time.sleep(0.02)

        self.configure_spin_direction(direction)
        if direction != "cw":
            print("Warning: Modbus speed-mode direction is not yet verified on this drive; requested direction may be ignored.")

        time.sleep(0.02)
        self.set_speed_mode()
        time.sleep(0.02)
        self.enable_output(True)
        time.sleep(0.02)
        self.set_target_speed(rpm)

    def stop_spin(self) -> None:
        self.set_target_speed(0)

    def spin_interactive(
        self,
        rpm: int,
        direction: str = "cw",
        step: int = 50,
        interval_s: float = 0.15,
    ) -> None:
        """
        Interactive spin mode:
          ] or + : increase speed
          [ or - : decrease speed
          0      : stop spin
          e      : disable output
          E      : enable output
          q      : quit
        """
        if not (0 <= rpm <= self.max_rpm):
            raise ValueError(f"RPM must be 0..{self.max_rpm} for model {self.model}")

        self.start_spin_mode(rpm=rpm, direction=direction)
        cmd_rpm = rpm

        print(f"Spin mode started: dir={direction}, rpm={cmd_rpm}")
        print("Keys: ]/+ faster, [/- slower, 0 stop, e disable output, E enable output, q quit")

        old_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

        try:
            while True:
                # nonblocking key read
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)

                    if ch in ("]", "+"):
                        cmd_rpm = min(self.max_rpm, cmd_rpm + step)
                        self.set_target_speed(cmd_rpm)

                    elif ch in ("[", "-"):
                        cmd_rpm = max(0, cmd_rpm - step)
                        self.set_target_speed(cmd_rpm)

                    elif ch == "0":
                        cmd_rpm = 0
                        self.set_target_speed(cmd_rpm)

                    elif ch == "e":
                        self.enable_output(False)

                    elif ch == "E":
                        self.enable_output(True)
                        time.sleep(0.02)
                        self.set_target_speed(cmd_rpm)
                    elif ch.lower() == "q":
                        self.set_target_speed(0)
                        time.sleep(0.05)
                        self.set_position_mode()
                        time.sleep(0.02)
                        break

                st = self.read_status()
                alarm_extra = f" ({self.alarm_text(st.alarm_code)})" if st.alarm_code else ""

                line = (
                    f"\rdir={direction:>3} | "
                    f"cmd={cmd_rpm:4d} rpm | "
                    f"actual={st.speed_rpm:7.1f} rpm | "
                    f"current={st.current_a:6.3f} A | "
                    f"voltage={st.voltage_v:5.2f} V | "
                    f"alarm=0x{st.alarm_code:04X}{alarm_extra}    "
                )
                sys.stdout.write(line)
                sys.stdout.flush()
                time.sleep(interval_s)
            
            self.set_position_mode()
            time.sleep(0.02)
            print("\nSpin mode stopped.")

        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)

    # -------- helpers --------

    def print_help_text(self) -> None:
        print(HELP_TEXT)

    def ensure_position_mode(self) -> None:
        self.enable_modbus(True)
        time.sleep(0.02)
        self.set_position_mode()
        time.sleep(0.02)
        self.enable_output(True)
        time.sleep(0.02)


def build_parser() -> argparse.ArgumentParser:
    epilog = (
        "Examples:\n"
        "  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 init\n"
        "  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 status\n"
        "  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_rel 1000 --wait\n"
        "  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 move_abs 0 --wait\n"
        "  python aim_servo.py --port /dev/tty.usbserial-110 --addr 1 spin --rpm 300 --dir cw\n"
    )

    p = argparse.ArgumentParser(
        description="YZ-AIM servo controller over RS485",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=epilog + "\n\n" + HELP_TEXT,
    )

    p.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-110")
    p.add_argument("--addr", type=int, default=1, help="Drive address")
    p.add_argument("--baud", type=int, default=19200, help="Baud rate")
    p.add_argument("--verbose", action="store_true", help="Show raw serial frames")
    p.add_argument(
        "--pulses-per-turn",
        type=int,
        default=32768,
        help="Pulse scale for degree/turn helpers",
    )
    p.add_argument(
        "--model",
        default="42AIM15",
        help="Motor model for RPM limiting (default: 42AIM15)",
    )

    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("help", help="Show extended usage examples and tips")
    sub.add_parser("status")
    sub.add_parser("monitor")
    sub.add_parser("zero_here")

    sp = sub.add_parser("init")
    sp.add_argument("--speed", type=int, default=1500)
    sp.add_argument("--accel", type=int, default=5000)
    sp.add_argument("--no-persistent-comm", action="store_true")
    sp.add_argument("--disable-output", action="store_true")

    sp = sub.add_parser("enable")
    sp.add_argument("value", type=int, choices=[0, 1])

    sp = sub.add_parser("modbus")
    sp.add_argument("value", type=int, choices=[0, 1])

    sp = sub.add_parser("speed")
    sp.add_argument("rpm", type=int)

    sp = sub.add_parser("accel")
    sp.add_argument("value", type=int)

    sp = sub.add_parser("read_reg")
    sp.add_argument("reg", type=lambda x: int(x, 0))
    sp.add_argument("--count", type=int, default=1)

    sp = sub.add_parser("write_reg")
    sp.add_argument("reg", type=lambda x: int(x, 0))
    sp.add_argument("value", type=lambda x: int(x, 0))

    sp = sub.add_parser("dir")
    sp.add_argument("value", type=int, choices=[0, 1])

    sp = sub.add_parser("special")
    sp.add_argument("value", type=int)

    sp = sub.add_parser("gear")
    sp.add_argument("numerator", type=int)
    sp.add_argument("denominator", type=int)

    sp = sub.add_parser("move_rel")
    sp.add_argument("pulses", type=int)
    sp.add_argument("--wait", action="store_true")

    sp = sub.add_parser("move_abs")
    sp.add_argument("pulses", type=int)
    sp.add_argument("--wait", action="store_true")
    sp.add_argument("--tol", type=int, default=10)

    sp = sub.add_parser("move_abs_hw")
    sp.add_argument("pulses", type=int)
    sp.add_argument("--wait", action="store_true")
    sp.add_argument("--tol", type=int, default=10)

    sp = sub.add_parser("move_deg")
    sp.add_argument("degrees", type=float)
    sp.add_argument("--wait", action="store_true")

    sp = sub.add_parser("move_turns")
    sp.add_argument("turns", type=float)
    sp.add_argument("--wait", action="store_true")

    sp = sub.add_parser("set_abs_counter")
    sp.add_argument("counts", type=int)

    sp = sub.add_parser("spin")
    sp.add_argument("--rpm", type=int, required=True, help="Starting RPM")
    sp.add_argument("--dir", choices=["cw", "ccw"], default="cw", help="Spin direction")
    sp.add_argument("--step", type=int, default=50, help="RPM adjustment step")

    return p


def main() -> int:
    args = build_parser().parse_args()

    servo = AIMServo(
        port=args.port,
        address=args.addr,
        baudrate=args.baud,
        verbose=args.verbose,
        pulses_per_turn=args.pulses_per_turn,
        model=args.model,
    )

    try:
        if args.cmd == "help":
            servo.print_help_text()

        elif args.cmd == "status":
            st = servo.read_status()
            print(st)
            if st.alarm_code != 0:
                print(f"alarm_text={servo.alarm_text(st.alarm_code)}")

        elif args.cmd == "monitor":
            servo.monitor()

        elif args.cmd == "init":
            servo.initialize_basic_position_mode(
                target_speed_rpm=args.speed,
                acceleration=args.accel,
                persistent_comm_mode=not args.no_persistent_comm,
                enable_output=not args.disable_output,
            )
            print("Initialization complete.")

        elif args.cmd == "enable":
            servo.enable_output(bool(args.value))
            print(f"Output enable set to {args.value}")

        elif args.cmd == "modbus":
            servo.enable_modbus(bool(args.value))
            print(f"Modbus enable set to {args.value}")

        elif args.cmd == "speed":
            servo.set_target_speed(args.rpm)
            print(f"Target speed set to {args.rpm} RPM")

        elif args.cmd == "accel":
            servo.set_acceleration(args.value)
            print(f"Acceleration set to {args.value}")

        elif args.cmd == "read_reg":
            regs = servo.read_registers(args.reg, args.count)
            for i, value in enumerate(regs):
                reg = args.reg + i
                print(f"0x{reg:02X}={value} (0x{value:04X})")

        elif args.cmd == "write_reg":
            servo.write_register(args.reg, args.value)
            print(f"Register 0x{args.reg:02X} set to {args.value} (0x{args.value & 0xFFFF:04X})")

        elif args.cmd == "dir":
            servo.set_dir_polarity(args.value)
            print(f"DIR polarity set to {args.value}")

        elif args.cmd == "special":
            servo.set_special_function(args.value)
            print(f"Special function set to {args.value}")

        elif args.cmd == "gear":
            servo.set_electronic_gear(args.numerator, args.denominator)
            servo.save_parameters()
            print(
                f"Electronic gear set to numerator={args.numerator}, "
                f"denominator={args.denominator}"
            )

        elif args.cmd == "zero_here":
            servo.zero_here()
            print("Current absolute position cleared to zero.")

        elif args.cmd == "set_abs_counter":
            servo.set_absolute_counter(args.counts)
            print(f"Absolute position counter set to {args.counts}.")

        elif args.cmd == "move_rel":
            ack = servo.move_relative(args.pulses)
            print(f"Relative move sent. delta={args.pulses}, ack={ack}")
            if args.wait:
                ok = servo.wait_until_stopped()
                print("Stopped." if ok else "Timed out waiting for stop.")

        elif args.cmd == "move_abs":
            ack = servo.move_absolute(args.pulses)
            print(f"Absolute move sent. target={args.pulses}, ack={ack}")
            if args.wait:
                ok = servo.wait_until_position(args.pulses, tolerance_pulses=args.tol)
                print("In position." if ok else "Timed out waiting for target.")

        elif args.cmd == "move_abs_hw":
            ack = servo.move_absolute_hw(args.pulses)
            print(f"Hardware absolute move sent. target={args.pulses}, ack={ack}")
            if args.wait:
                ok = servo.wait_until_position(args.pulses, tolerance_pulses=args.tol)
                print("In position." if ok else "Timed out waiting for target.")

        elif args.cmd == "move_deg":
            pulses = servo.pulses_from_degrees(args.degrees)
            ack = servo.move_relative(pulses)
            print(f"Relative degree move sent. degrees={args.degrees}, pulses={pulses}, ack={ack}")
            if args.wait:
                ok = servo.wait_until_stopped()
                print("Stopped." if ok else "Timed out waiting for stop.")

        elif args.cmd == "move_turns":
            pulses = servo.pulses_from_turns(args.turns)
            ack = servo.move_relative(pulses)
            print(f"Relative turn move sent. turns={args.turns}, pulses={pulses}, ack={ack}")
            if args.wait:
                ok = servo.wait_until_stopped()
                print("Stopped." if ok else "Timed out waiting for stop.")

        elif args.cmd == "spin":
            servo.spin_interactive(
                rpm=args.rpm,
                direction=args.dir,
                step=args.step,
            )

        else:
            raise RuntimeError("Unknown command")

        return 0

    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 130

    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    finally:
        servo.close()


if __name__ == "__main__":
    raise SystemExit(main())
