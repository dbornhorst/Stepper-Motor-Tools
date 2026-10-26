// Command wrapper + line-parsing for the iCL_Motor_Test Teensy sketch, built on top of a
// LineSerial transport. See iCL/README.md and
// iCL/Firmware/iCL_Motor_Test/iCL_Motor_Test.ino for the protocol this is derived from -
// that sketch IS the protocol spec, there's no separate vendor document for it (the drive-side
// iCL_Series_User_Manual.pdf only covers the physical PUL/DIR/DIP-switch side, not this firmware).

const DIRECTION = { CW: 'CW', CCW: 'CCW' };

const MOTION_STATE_LABEL = {
  idle: 'Idle',
  accel: 'Accelerating',
  decel: 'Decelerating',
  cruise: 'Cruising',
};

// Parses a sketch "S,<targetRpm>,<currentRpm>,<steps>,<state>" stream line. Returns null if the
// line doesn't match (e.g. it's a "# ..." comment line instead).
function parseStreamLine(line) {
  if (!line.startsWith('S,')) return null;
  const parts = line.split(',');
  if (parts.length !== 5) return null;
  const targetRpm = Number(parts[1]);
  const currentRpm = Number(parts[2]);
  const steps = Number(parts[3]);
  const state = parts[4];
  if (Number.isNaN(targetRpm) || Number.isNaN(currentRpm) || Number.isNaN(steps)) return null;
  return { targetRpm, currentRpm, steps, state };
}

// Opportunistically parses whatever "# ..." lines the sketch's `status` command produced. This
// is a best-effort text scrape (see LineSerial.sendAndCollect's doc comment) - only used to
// pre-fill the Settings panel with the device's actual current values on connect, not for
// anything that gates a control action.
function parseStatusLines(lines) {
  const result = {};
  for (const line of lines) {
    const settingsMatch = line.match(
      /accel=([\d.]+)\s*rpm\/s\s*decel=([\d.]+)\s*rpm\/s\s*max=([\d.]+)\s*rpm\s*spr=(\d+)\s*invert=(\d)/
    );
    if (settingsMatch) {
      result.accelRpmPerSec = Number(settingsMatch[1]);
      result.decelRpmPerSec = Number(settingsMatch[2]);
      result.maxRpm = Number(settingsMatch[3]);
      result.stepsPerRev = Number(settingsMatch[4]);
      result.invert = settingsMatch[5] === '1';
      continue;
    }
    const stateMatch = line.match(/state=(\w+)\s+target=([\d.-]+)\s*rpm\s*current=([\d.-]+)\s*rpm\s*steps=(-?\d+)/);
    if (stateMatch) {
      result.state = stateMatch[1];
      result.targetRpm = Number(stateMatch[2]);
      result.currentRpm = Number(stateMatch[3]);
      result.steps = Number(stateMatch[4]);
    }
  }
  return result;
}

class ICLStepperDrive {
  constructor(lineSerial) {
    this.serial = lineSerial;
  }

  onStreamUpdate(callback) {
    return this.serial.onLine((line) => {
      const parsed = parseStreamLine(line);
      if (parsed) callback(parsed);
    });
  }

  async enableStream(on) {
    await this.serial.send(`stream ${on ? 'on' : 'off'}`);
  }

  async requestStatus() {
    const lines = await this.serial.sendAndCollect('status');
    return parseStatusLines(lines);
  }

  // rpm: magnitude (always >= 0, this method applies the sign). direction: DIRECTION.CW/CCW.
  async setTargetRpm(rpm, direction) {
    const signed = direction === DIRECTION.CCW ? -Math.abs(rpm) : Math.abs(rpm);
    await this.serial.sendAndCollect(`rpm ${signed}`);
  }

  // Ramps down to 0 using the configured decel rate.
  async stop() {
    await this.serial.send('stop');
  }

  // Immediate stop, bypassing the decel ramp entirely (sketch's emergencyStop()).
  async estop() {
    await this.serial.send('x');
  }

  async setAccel(rpmPerSec) {
    return this.serial.sendAndCollect(`accel ${rpmPerSec}`);
  }

  async setDecel(rpmPerSec) {
    return this.serial.sendAndCollect(`decel ${rpmPerSec}`);
  }

  async setMaxRpm(rpm) {
    return this.serial.sendAndCollect(`max ${rpm}`);
  }

  // Stopped-only on the firmware side (it will refuse and print an error otherwise) - the app
  // should also disable this control in the UI while running, not rely on the firmware's refusal.
  async setStepsPerRev(spr) {
    return this.serial.sendAndCollect(`spr ${spr}`);
  }

  // Stopped-only, same caveat as setStepsPerRev.
  async setInvert(on) {
    return this.serial.sendAndCollect(`invert ${on ? 1 : 0}`);
  }

  async zero() {
    await this.serial.send('zero');
  }
}

export { ICLStepperDrive, DIRECTION, MOTION_STATE_LABEL, parseStreamLine, parseStatusLines };
