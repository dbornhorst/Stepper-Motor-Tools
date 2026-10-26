#!/usr/bin/env python3

import argparse
import shlex
import sys

from aim_servo import AIMServo


class AIMShell:
    def __init__(self, port, addr=1, baud=19200, model="42AIM15"):
        self.servo = AIMServo(
            port=port,
            address=addr,
            baudrate=baud,
            model=model,
        )
        self.running = True
        self.spin_active = False

        self.auto_release_enabled = True
        self.release_current_threshold_a = 1.000
        self.release_dwell_s = 0.0

        self._peak_current = 0.0
        self._prev_pos = None
        self._resistance_timer = None
        self._resist_latched = False
        self._blink = False

    # ---------- commands ----------

    def cmd_init(self, args):
        self.servo.initialize_basic_position_mode()

    def cmd_status(self, args):
        print(self.servo.read_status())

    def cmd_zero(self, args):
        self.servo.zero_here()

    def cmd_move(self, args):
        if not args:
            print("usage: move <target_counts>")
            return
        target = int(args[0])
        self.servo.move_absolute(target)

    def cmd_release(self, args):
        self.servo.enable_output(False)

    def cmd_enable(self, args):
        self.servo.enable_output(True)
        print("Output enabled")

    def cmd_spin(self, args):
        if not args:
            print("usage: spin <rpm>")
            return

        rpm = int(args[0])

        self.servo.start_spin_mode(rpm)
        self.spin_active = True

        # reset per-run telemetry state
        self._peak_current = 0.0
        self._prev_pos = None
        self._blink = False

        print("Spin mode (interactive)")
        print("Keys: ]/+ faster, [/- slower, 0 stop, e disable, E enable, q quit\n")

        import termios
        import tty
        import select
        import sys
        import time

        old_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

        cmd_rpm = rpm
        last_status = None
        last_status_time = 0.0

        try:
            while True:
                # -------- keyboard handling --------
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)

                    if ch in ("]", "+"):
                        cmd_rpm = min(self.servo.max_rpm, cmd_rpm + 50)
                        self.servo.set_target_speed(cmd_rpm)

                    elif ch in ("[", "-"):
                        cmd_rpm = max(0, cmd_rpm - 50)
                        self.servo.set_target_speed(cmd_rpm)

                    elif ch == "0":
                        cmd_rpm = 0
                        self.servo.set_target_speed(0)

                    elif ch == "e":
                        self.servo.enable_output(False)
                        self.spin_active = False
                        print("\nOutput disabled.")
                        break

                    elif ch == "E":
                        self.servo.enable_output(True)
                        self.spin_active = True
                        self.servo.set_target_speed(cmd_rpm)

                    elif ch.lower() == "q":
                        self.servo.set_target_speed(0)
                        time.sleep(0.01)
                        self.servo.set_position_mode()
                        self.spin_active = False
                        print("\nSpin stopped.")
                        break

                # -------- FAST auto-release path: current only --------
                current = self.servo.read_current_a()
                self._peak_current = max(self._peak_current, current)

                if self.auto_release_enabled and current >= self.release_current_threshold_a:
                    # release immediately: do NOT wait, do NOT ramp to zero first
                    self.servo.enable_output(False)
                    self.spin_active = False

                    print("\n*** AUTO-RELEASE: current threshold exceeded ***")
                    print(
                        f"threshold={self.release_current_threshold_a:.3f} A, "
                        f"measured={current:.3f} A"
                    )
                    print("Output disabled. Shaft should now be free.\n")
                    break

                # -------- slower pretty telemetry path --------
                now = time.time()
                if last_status is None or (now - last_status_time) >= 0.05:
                    last_status = self.servo.read_status()
                    last_status_time = now

                    speed = abs(last_status.speed_rpm)
                    pos = last_status.absolute_position

                    def current_bar(i, max_i=1.0, width=12):
                        i = min(max(i, 0.0), max_i)
                        filled = int((i / max_i) * width)
                        return "█" * filled + "░" * (width - filled)

                    bar = current_bar(current, max_i=1.0)

                    # display-only resistance hint
                    resist_flag = ""
                    if self._prev_pos is not None:
                        pos_delta = abs(pos - self._prev_pos)
                        if current >= self.release_current_threshold_a or (
                            current > 0.35 and speed < 20 and pos_delta < 3
                        ):
                            self._blink = not self._blink
                            if self._blink:
                                resist_flag = "⚠ RESIST"

                    self._prev_pos = pos

                    line = (
                        f"\r"
                        f"cmd={cmd_rpm:4d} | "
                        f"act={last_status.speed_rpm:7.1f} rpm | "
                        f"I={current:5.3f}A [{bar}] "
                        f"peak={self._peak_current:5.3f}A | "
                        f"pos={pos:6d} | "
                        f"{resist_flag:<9} | "
                        f"V={last_status.voltage_v:5.2f} | "
                        f"alarm=0x{last_status.alarm_code:04X}   "
                    )

                    sys.stdout.write(line)
                    sys.stdout.flush()

                # very short sleep for responsiveness
                time.sleep(0.005)

        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)

    def cmd_oscillate(self, args):
        """
        Centered oscillation around the current position.

        Usage:
            oscillate
            oscillate 180
            oscillate 180 200
            oscillate 180 200 0.500

        Arguments:
            total_swing_deg     default 180
            speed_rpm           default 300
            release_current_A   default self.release_current_threshold_a

        Example:
            oscillate 180
        means:
            current position is center
            motion swings from center-90deg to center+90deg
        """
        import termios
        import tty
        import select
        import sys
        import time

        total_swing_deg = float(args[0]) if len(args) >= 1 else 180.0
        speed_rpm = int(args[1]) if len(args) >= 2 else 300
        release_current_a = float(args[2]) if len(args) >= 3 else self.release_current_threshold_a

        half_swing_deg = total_swing_deg / 2.0
        half_swing_counts = self.servo.pulses_from_degrees(half_swing_deg)

        # centered around current position
        center = self.servo.read_absolute_position()
        target_a = center - half_swing_counts
        target_b = center + half_swing_counts
        target = target_b

        # configure motion
        self.servo.ensure_position_mode()
        self.servo.enable_output(True)
        self.servo.set_target_speed(min(speed_rpm, self.servo.max_rpm))

        # reset per-run telemetry state
        self._peak_current = 0.0
        self._prev_pos = center
        self._blink = False

        print("Centered oscillate mode (interactive)")
        print("Keys: q quit, e disable output, E enable output, 0 hold current position\n")
        print(
            f"center={center} | total_swing={total_swing_deg:.1f} deg | "
            f"half_swing={half_swing_deg:.1f} deg | speed={speed_rpm} rpm | "
            f"release={release_current_a:.3f} A"
        )
        print(f"range: {target_a} ... {target_b}")

        old_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

        last_status = None
        last_status_time = 0.0
        tolerance = 10

        try:
            # kick off first move
            self.servo.move_absolute(target)

            while True:
                # -------- keyboard handling --------
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1)

                    if ch.lower() == "q":
                        self.servo.enable_output(False)
                        print("\nOscillate stopped. Output disabled.")
                        break

                    elif ch == "e":
                        self.servo.enable_output(False)
                        print("\nOutput disabled.")
                        break

                    elif ch == "E":
                        self.servo.enable_output(True)

                    elif ch == "0":
                        current_pos = self.servo.read_absolute_position()
                        self.servo.move_absolute(current_pos)

                # -------- FAST auto-release path --------
                current = self.servo.read_current_a()
                self._peak_current = max(self._peak_current, current)

                if self.auto_release_enabled and current >= release_current_a:
                    self.servo.enable_output(False)
                    print("\n*** AUTO-RELEASE: current threshold exceeded ***")
                    print(
                        f"threshold={release_current_a:.3f} A, "
                        f"measured={current:.3f} A"
                    )
                    print("Output disabled. Shaft should now be free.\n")
                    break

                # -------- position tracking --------
                pos = self.servo.read_absolute_position()

                # switch direction when current target reached
                if abs(pos - target) <= tolerance:
                    if target == target_b:
                        target = target_a
                    else:
                        target = target_b
                    self.servo.move_absolute(target)

                # -------- slower pretty telemetry --------
                now = time.time()
                if last_status is None or (now - last_status_time) >= 0.05:
                    last_status = self.servo.read_status()
                    last_status_time = now

                    speed = abs(last_status.speed_rpm)

                    def current_bar(i, max_i=1.0, width=12):
                        i = min(max(i, 0.0), max_i)
                        filled = int((i / max_i) * width)
                        return "█" * filled + "░" * (width - filled)

                    bar = current_bar(current, max_i=1.0)

                    resist_flag = ""
                    pos_delta = abs(pos - self._prev_pos)
                    if current >= release_current_a or (current > 0.35 and speed < 20 and pos_delta < 3):
                        self._blink = not self._blink
                        if self._blink:
                            resist_flag = "⚠ RESIST"

                    self._prev_pos = pos

                    line = (
                        f"\r"
                        f"center={center:6d} | "
                        f"target={target:6d} | "
                        f"pos={pos:6d} | "
                        f"act={last_status.speed_rpm:7.1f} rpm | "
                        f"I={current:5.3f}A [{bar}] "
                        f"peak={self._peak_current:5.3f}A | "
                        f"{resist_flag:<9} | "
                        f"V={last_status.voltage_v:5.2f} | "
                        f"alarm=0x{last_status.alarm_code:04X}   "
                    )

                    sys.stdout.write(line)
                    sys.stdout.flush()

                time.sleep(0.005)

        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)

    def cmd_stop(self, args):
        self.servo.set_target_speed(0)
        time.sleep(0.05)
        self.servo.set_position_mode()
        self.spin_active = False
        print("Spin stopped")

    def cmd_quit(self, args):
        self.running = False

    def cmd_help(self, args):
        print("""
    Commands:
    init                    initialize motor
    status                  print status
    zero                    zero current position
    move <counts>           move to absolute position (software absolute)
    release                 disable output (free shaft)
    enable                  enable output
    spin <rpm>              start continuous spin
    stop                    stop spin
    oscillate               centered oscillation, total swing 180 deg at 300 rpm
    oscillate <deg>         centered oscillation by total swing angle in degrees
    oscillate <deg> <rpm>   centered oscillation by total swing angle at rpm
    oscillate <deg> <rpm> <A>
                            centered oscillation with auto-release current threshold
    quit                    exit shell
    """)

    # ---------- dispatcher ----------

    def dispatch(self, line):
        parts = shlex.split(line)
        if not parts:
            return

        cmd = parts[0]
        args = parts[1:]

        commands = {
            "init": self.cmd_init,
            "status": self.cmd_status,
            "zero": self.cmd_zero,
            "move": self.cmd_move,
            "release": self.cmd_release,
            "enable": self.cmd_enable,
            "oscillate": self.cmd_oscillate,
            "spin": self.cmd_spin,
            "stop": self.cmd_stop,
            "quit": self.cmd_quit,
            "exit": self.cmd_quit,
            "help": self.cmd_help,
        }

        if cmd in commands:
            try:
                commands[cmd](args)
            except Exception as e:
                print(f"ERROR: {e}")
        else:
            print(f"Unknown command: {cmd}")

    # ---------- main loop ----------

    def run(self):
        print("AIM Interactive Shell")
        print("Type 'help' for commands\n")

        while self.running:
            try:
                line = input("aim> ")
                self.dispatch(line)
            except KeyboardInterrupt:
                print("\nUse 'quit' to exit.")
            except EOFError:
                break

        self.cleanup()

    def cleanup(self):
        try:
            if self.spin_active:
                self.servo.set_target_speed(0)
        finally:
            self.servo.close()
            print("Disconnected.")


def main():
    parser = argparse.ArgumentParser(description="AIM interactive shell")
    parser.add_argument("--port", required=True)
    parser.add_argument("--addr", type=int, default=1)
    parser.add_argument("--baud", type=int, default=19200)
    parser.add_argument("--model", default="42AIM15")

    args = parser.parse_args()

    shell = AIMShell(
        port=args.port,
        addr=args.addr,
        baud=args.baud,
        model=args.model,
    )

    shell.run()


if __name__ == "__main__":
    main()
