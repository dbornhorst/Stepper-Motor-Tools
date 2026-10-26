// YZ-AIM integrated servo drive register map and command sequences, built on top of a ModbusRTU
// transport. See AimServo/README.md and AimServo/YZ-AIM_Manual.pdf for the protocol facts this is
// derived from, and AimServo/PythonReference/aim_servo.py for the bench-tested reference this was
// ported from.

const REG = {
  MODBUS_ENABLE: 0x00,      // 0=IO/pulse control, 1=Modbus commands accepted
  OUTPUT_ENABLE: 0x01,      // 0=torque off, 1=torque on
  TARGET_SPEED: 0x02,       // rpm. Manual's documented range is unsigned 0-3000, but confirmed on
                             // real hardware: a signed two's-complement value DOES reverse
                             // direction in speed mode - see setTargetSpeedForDirection().
  ACCEL: 0x03,              // (r/min)/s, 0-60098 validated range per manual/Python reference
  DIR_POLARITY: 0x09,       // 0/1 - confirmed on real hardware to have NO effect on Modbus
                             // speed-mode direction. The manual's description of this register
                             // turned out to be specific to the drive's *external* hardware DIR
                             // pin (used in pulse+direction mode), not Modbus-commanded speed.
                             // Kept implemented as a documented-but-unused fallback (see
                             // setTargetSpeedForDirection()) in case a different unit differs.
  ELEC_GEAR_NUM: 0x0A,      // electronic gear numerator; 0 is a sentinel (see setCommModePersistent)
  ELEC_GEAR_DEN: 0x0B,      // electronic gear denominator
  ALARM: 0x0E,              // read-only, exact-match alarm code (see ALARM_TEXT)
  SYSTEM_CURRENT: 0x0F,     // read-only, raw/2000.0 = amps
  CURRENT_SPEED: 0x10,      // read-only, signed, raw/10.0 = rpm
  SYSTEM_VOLTAGE: 0x11,     // read-only, raw/327.0 = volts
  SYSTEM_TEMP: 0x12,        // read-only, raw = degrees C
  SAVE_FLAG: 0x14,          // write 1 to persist parameters to NVM; advanced/setup only
  DEVICE_ADDRESS: 0x15,     // READ-ONLY per the vendor manual (not writable via FC06 - the
                             // manual references a separate, undocumented-in-examples function
                             // code 0x7A for changing it, which this tool does not implement).
  ABS_POS_LOW: 0x16,        // absolute position, low word (read together with HIGH, FC03 count=2)
  ABS_POS_HIGH: 0x17,
  SPECIAL_FUNCTION: 0x19,   // 0=position mode, 3=speed mode (other values are non-Modbus modes,
                             // e.g. pulse+direction - out of scope for this tool)
};

const FC = {
  READ: 0x03,
  WRITE_SINGLE: 0x06,
  WRITE_MULTI: 0x10,
  MOVE_RELATIVE: 0x78,
  MOVE_ABSOLUTE_HW: 0x7B,   // documented by the vendor, confirmed unreliable on bench hardware by
                             // the Python reference this was ported from - not used by this tool's
                             // moveAbsolute(), which is software-only (see below).
};

const SPECIAL_FUNCTION = {
  POSITION_MODE: 0,
  SPEED_MODE: 3,
};

const DIRECTION = { CW: 'CW', CCW: 'CCW' };

const DIRECTION_METHOD = {
  SIGNED: 'signed',        // write a signed target speed (negative = CCW)
  DIR_POLARITY: 'dir-polarity', // write unsigned magnitude, flip REG.DIR_POLARITY for direction
};

// Exact-match alarm codes (not a bitfield, unlike some other drives).
const ALARM_TEXT = new Map([
  [0x10, 'Battery off alarm'],
  [0x12, 'Overflow alarm'],
  [0x14, 'Blocked rotation alarm'],
  [0x15, 'Overvoltage / overpressure alarm'],
]);

function u16ToI16(raw) {
  return raw >= 0x8000 ? raw - 0x10000 : raw;
}

// Ordinary big-endian 32-bit signed combine, for normal position reads (FC03). NOT the same as
// decodeVendorMoveAck() below - that word-swap only applies to 0x78/0x7B move-command replies.
function combineWordsSigned32(highWord, lowWord) {
  const value = (((highWord & 0xFFFF) << 16) | (lowWord & 0xFFFF)) >>> 0;
  return value >= 0x80000000 ? value - 0x100000000 : value;
}

// The 32-bit ack value in a 0x78/0x7B reply has its two 16-bit words swapped relative to normal
// big-endian order: ack = combine(reply[4..5], reply[2..3]), not combine(reply[2..3], reply[4..5]).
// Confirmed against the bench-tested Python reference (aim_servo.py) - reproduced exactly here.
function decodeVendorMoveAck(replyFrame) {
  const hi = (replyFrame[4] << 8) | replyFrame[5];
  const lo = (replyFrame[2] << 8) | replyFrame[3];
  return combineWordsSigned32(hi, lo);
}

class AIMServoDrive {
  constructor(modbus, slaveId, options = {}) {
    this.modbus = modbus;
    this.slaveId = slaveId;
    this.interWriteDelayMs = options.interWriteDelayMs ?? 20;
  }

  async _delay(ms) {
    if (ms > 0) await new Promise((resolve) => setTimeout(resolve, ms));
  }

  // ---- reads ----
  // No combined status register exists on this drive (unlike some others) - these stay granular;
  // app.js decides per-poll-cycle which to call (see its tiered polling).

  async readModbusEnable() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.MODBUS_ENABLE, 1);
    return raw === 1;
  }

  async readOutputEnable() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.OUTPUT_ENABLE, 1);
    return raw === 1;
  }

  async readAlarm() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.ALARM, 1);
    return {
      raw,
      hasAlarm: raw !== 0,
      message: ALARM_TEXT.get(raw) ?? (raw !== 0 ? `Unknown alarm code 0x${raw.toString(16)}` : null),
    };
  }

  async readCurrentAmps() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.SYSTEM_CURRENT, 1);
    return raw / 2000.0;
  }

  async readSpeedRpm() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.CURRENT_SPEED, 1);
    return u16ToI16(raw) / 10.0;
  }

  async readVoltage() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.SYSTEM_VOLTAGE, 1);
    return raw / 327.0;
  }

  async readTemperature() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.SYSTEM_TEMP, 1);
    return raw;
  }

  async readAbsolutePosition() {
    const [low, high] = await this.modbus.readHoldingRegisters(this.slaveId, REG.ABS_POS_LOW, 2);
    return combineWordsSigned32(high, low);
  }

  // ---- normal run-time controls (never touch EEPROM) ----

  async enableModbus(on) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.MODBUS_ENABLE, on ? 1 : 0);
  }

  async enableOutput(on) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.OUTPUT_ENABLE, on ? 1 : 0);
  }

  async setAccel(value) {
    const clamped = Math.max(0, Math.min(60098, Math.round(value)));
    await this.modbus.writeSingleRegister(this.slaveId, REG.ACCEL, clamped);
  }

  async setPositionMode() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.SPECIAL_FUNCTION, SPECIAL_FUNCTION.POSITION_MODE);
  }

  async setSpeedMode() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.SPECIAL_FUNCTION, SPECIAL_FUNCTION.SPEED_MODE);
  }

  async ensurePositionMode() {
    await this.enableModbus(true);
    await this._delay(this.interWriteDelayMs);
    await this.setPositionMode();
    await this._delay(this.interWriteDelayMs);
    await this.enableOutput(true);
    await this._delay(this.interWriteDelayMs);
  }

  // Relative move via the vendor's 0x78 command - the proven, bench-tested motion primitive.
  async moveRelative(deltaPulses) {
    await this.ensurePositionMode();
    const reply = await this.modbus.sendVendorMoveCommand(this.slaveId, FC.MOVE_RELATIVE, Math.round(deltaPulses));
    return decodeVendorMoveAck(reply);
  }

  // Software absolute move: read current position, compute delta, issue a relative move.
  // Deliberately NOT using the native hardware absolute-move opcode (0x7B) - documented by the
  // vendor but confirmed unreliable on bench hardware by the Python reference this was ported
  // from ("acknowledged but did not produce reliable physical motion").
  async moveAbsolute(targetPulses) {
    await this.ensurePositionMode();
    const current = await this.readAbsolutePosition();
    const delta = Math.round(targetPulses) - current;
    return this.moveRelative(delta);
  }

  // Rewrites the absolute position counter without commanding motion (e.g. "zero here"). Order
  // matters: must be stopped first, then gear-numerator=0, then both position words together.
  async setAbsoluteCounter(positionCounts, { waitUntilStopped } = {}) {
    if (waitUntilStopped) await waitUntilStopped();
    await this.modbus.writeSingleRegister(this.slaveId, REG.ELEC_GEAR_NUM, 0);
    await this._delay(this.interWriteDelayMs);
    const value = Math.round(positionCounts) < 0 ? Math.round(positionCounts) + 0x100000000 : Math.round(positionCounts);
    const low = value & 0xFFFF;
    const high = (value >>> 16) & 0xFFFF;
    await this.modbus.writeMultipleRegisters(this.slaveId, REG.ABS_POS_LOW, [low, high]);
    await this._delay(this.interWriteDelayMs);
  }

  async zeroPosition({ waitUntilStopped } = {}) {
    await this.ensurePositionMode();
    await this.setAbsoluteCounter(0, { waitUntilStopped });
  }

  // direction: DIRECTION.CW or DIRECTION.CCW. Two mechanisms, selectable via settings:
  //   - 'signed' (default, CONFIRMED on real hardware): write a signed target speed (negative =
  //     CCW). Despite the manual documenting REG.TARGET_SPEED's range as unsigned 0-3000 rpm,
  //     the drive does honor a signed two's-complement value and reverses direction correctly.
  //   - 'dir-polarity': write an unsigned magnitude, flip REG.DIR_POLARITY for direction, per the
  //     fallback used by the Python reference's rock_servo.py. CONFIRMED on real hardware to have
  //     NO effect on Modbus speed-mode direction - the manual's description of that register
  //     turned out to be specific to the external hardware DIR pin, not Modbus-commanded speed.
  //     Kept implemented only in case a different unit/firmware revision behaves differently.
  async setTargetSpeedForDirection(rpm, direction, { directionMethod, positiveDirPolarity = 0, maxRpm = 3000 } = {}) {
    const magnitude = Math.max(0, Math.min(maxRpm, Math.round(Math.abs(rpm))));
    if (directionMethod === DIRECTION_METHOD.SIGNED) {
      const signed = direction === DIRECTION.CCW ? -magnitude : magnitude;
      const unsigned = signed < 0 ? signed + 0x10000 : signed;
      await this.modbus.writeSingleRegister(this.slaveId, REG.TARGET_SPEED, unsigned & 0xFFFF);
    } else {
      const polarity = direction === DIRECTION.CCW ? (1 - positiveDirPolarity) : positiveDirPolarity;
      await this.modbus.writeSingleRegister(this.slaveId, REG.DIR_POLARITY, polarity);
      await this._delay(this.interWriteDelayMs);
      await this.modbus.writeSingleRegister(this.slaveId, REG.TARGET_SPEED, magnitude);
    }
  }

  async startSpeedJog(rpm, direction, options = {}) {
    await this.enableModbus(true);
    await this._delay(this.interWriteDelayMs);
    if (options.directionMethod === DIRECTION_METHOD.SIGNED) {
      await this.setTargetSpeedForDirection(rpm, direction, options);
      await this._delay(this.interWriteDelayMs);
    }
    await this.setSpeedMode();
    await this._delay(this.interWriteDelayMs);
    await this.enableOutput(true);
    await this._delay(this.interWriteDelayMs);
    await this.setTargetSpeedForDirection(rpm, direction, options);
  }

  // Debounced live update while jogging - just re-issues the speed/direction write.
  async updateSpeedJog(rpm, direction, options = {}) {
    await this.setTargetSpeedForDirection(rpm, direction, options);
  }

  async stopSpeedJog(options = {}) {
    await this.setTargetSpeedForDirection(0, DIRECTION.CW, options);
    await this._delay(this.interWriteDelayMs);
    await this.setPositionMode();
  }

  // Debounced "wait until done" polling - single-sample checks would false-positive on noisy
  // readings, so require the condition to hold for holdTimeS before declaring success.
  async waitUntilStopped({ speedThresholdRpm = 1.0, holdTimeS = 0.20, timeoutS = 10, pollIntervalMs = 50 } = {}) {
    const start = Date.now();
    let goodSince = null;
    while ((Date.now() - start) / 1000 < timeoutS) {
      const speed = await this.readSpeedRpm();
      if (Math.abs(speed) <= speedThresholdRpm) {
        if (goodSince === null) goodSince = Date.now();
        if ((Date.now() - goodSince) / 1000 >= holdTimeS) return true;
      } else {
        goodSince = null;
      }
      await this._delay(pollIntervalMs);
    }
    return false;
  }

  async waitUntilPositionReached(targetPulses, { tolerancePulses = 10, holdTimeS = 0.10, timeoutS = 10, pollIntervalMs = 50 } = {}) {
    const start = Date.now();
    let goodSince = null;
    while ((Date.now() - start) / 1000 < timeoutS) {
      const position = await this.readAbsolutePosition();
      if (Math.abs(position - targetPulses) <= tolerancePulses) {
        if (goodSince === null) goodSince = Date.now();
        if ((Date.now() - goodSince) / 1000 >= holdTimeS) return true;
      } else {
        goodSince = null;
      }
      await this._delay(pollIntervalMs);
    }
    return false;
  }

  // Emergency-style output disable, bypassing the request queue - used by the current-cutoff
  // safety trip so it's never stuck behind a debounced jog update or a slow poll.
  async emergencyDisableOutput() {
    await this.modbus.jumpQueueWriteSingleRegister(this.slaveId, REG.OUTPUT_ENABLE, 0);
  }

  // ---- advanced/one-time setup only - always pair with saveParameters() and a confirmation UI.
  // Never call these from a normal run-time control path (finite EEPROM write cycles). ----

  async saveParameters() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.SAVE_FLAG, 1);
    await this._delay(100); // NVM write completion, per the vendor reference
  }

  async setElectronicGear(numerator, denominator) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.ELEC_GEAR_NUM, numerator & 0xFFFF);
    await this._delay(this.interWriteDelayMs);
    await this.modbus.writeSingleRegister(this.slaveId, REG.ELEC_GEAR_DEN, denominator & 0xFFFF);
  }

  // Sentinel value meaning "enter persistent Modbus-comm-mode on power-up" (electronic gear
  // numerator = 0). Pair with saveParameters() to persist across power cycles.
  async setCommModePersistent() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.ELEC_GEAR_NUM, 0);
  }

  // Device address (slave ID) is READ-ONLY per the vendor manual - no write method is provided.
  // Changing it requires the vendor's own tool/DIP switches, or function code 0x7A, which the
  // manual references but does not document with a worked example - not implemented here.
  async readDeviceAddress() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.DEVICE_ADDRESS, 1);
    return raw;
  }

  // ---- raw register access, for the Advanced panel's debug tool ----

  async readRawRegister(addr) {
    const [value] = await this.modbus.readHoldingRegisters(this.slaveId, addr, 1);
    return value;
  }

  async writeRawRegister(addr, value) {
    await this.modbus.writeSingleRegister(this.slaveId, addr, value);
  }
}

export {
  AIMServoDrive,
  REG,
  FC,
  SPECIAL_FUNCTION,
  DIRECTION,
  DIRECTION_METHOD,
  ALARM_TEXT,
  u16ToI16,
  combineWordsSigned32,
  decodeVendorMoveAck,
};
