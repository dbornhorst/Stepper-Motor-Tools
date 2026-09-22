// DM556RS register map and command sequences, built on top of a ModbusRTU transport.
// See DM556RS_Manual.pdf and the plan doc for the protocol facts this is derived from.

const REG = {
  SW_ENABLE: 0x000F,        // Pr0.07 - RS485 force-enable override (0=follow DI1, 1=force enabled)
  DIRECTION_SENSE: 0x0007,  // Pr0.03 - fallback-only direction mechanism, see startRun()
  SLAVE_ID: 0x01BF,         // Pr5.23 - advanced/setup only
  BAUD: 0x01BD,             // Pr5.22 - advanced/setup only
  DATA_FORMAT: 0x01C1,      // Pr5.24 - advanced/setup only
  JOG_VEL: 0x01E1,          // Pr6.00 - fallback-only, see startRun()
  JOG_ACCDEC: 0x01E7,       // Pr6.03 - fallback-only, see startRun()
  MOTION_STATUS: 0x1003,    // read-only bitfield
  VEL_FEEDBACK_HI: 0x1046,
  VEL_FEEDBACK_LO: 0x1047,
  ALARM: 0x2203,            // read-only
  CONTROL_WORD: 0x1801,
  PR0_MODE: 0x6200,         // Pr9.00
  PR0_POS_HI: 0x6201,       // Pr9.01 - unused (velocity mode)
  PR0_POS_LO: 0x6202,       // Pr9.02 - unused (velocity mode)
  PR0_VEL: 0x6203,          // Pr9.03 - signed 16-bit, rpm
  PR0_ACC: 0x6204,          // Pr9.04 - ease-in, ms per 1000rpm
  PR0_DEC: 0x6205,          // Pr9.05 - ease-out, ms per 1000rpm
  PR0_DWELL: 0x6206,        // Pr9.06 - unused
  PR_TRIGGER: 0x6002,       // Pr8.02
};

const CONTROL_WORD_VALUE = {
  RESET_ALARM: 0x1111,
  SAVE_EEPROM: 0x2211,
  JOG_CW: 0x4001,   // fallback-only; requires resend <50ms to sustain motion
  JOG_CCW: 0x4002,  // fallback-only
};

const PR_MODE = {
  NONE: 0x0000,
  VELOCITY: 0x0002,
};

const PR_TRIGGER_VALUE = {
  RUN_PATH0: 0x0010,
  ESTOP: 0x0040,
};

const MOTION_STATUS_BIT = {
  FAULT: 0x01,
  ENABLED: 0x02,
  RUNNING: 0x04,
  CMD_COMPLETE: 0x10,
  PATH_COMPLETE: 0x20,
  HOMING_COMPLETE: 0x40,
};

const ALARM_BIT = {
  OVER_CURRENT: 0x001,
  OVER_VOLTAGE: 0x002,
  CURRENT_SAMPLING_FAULT: 0x040,
  SHAFT_LOCK_FAULT: 0x080,
  AUTO_TUNING_FAULT: 0x100,
  EEPROM_FAULT: 0x200,
};

const ALARM_MESSAGES = [
  [ALARM_BIT.OVER_CURRENT, 'Over-current'],
  [ALARM_BIT.OVER_VOLTAGE, 'Over-voltage'],
  [ALARM_BIT.CURRENT_SAMPLING_FAULT, 'Current sampling fault'],
  [ALARM_BIT.SHAFT_LOCK_FAULT, 'Failed to lock shaft (check motor wiring)'],
  [ALARM_BIT.AUTO_TUNING_FAULT, 'Auto-tuning fault'],
  [ALARM_BIT.EEPROM_FAULT, 'EEPROM fault'],
];

const DIRECTION = { CW: 'CW', CCW: 'CCW' };

function toSigned16(value) {
  // Two's complement encode a signed rpm value into an unsigned 16-bit register.
  return value < 0 ? (0x10000 + value) & 0xFFFF : value & 0xFFFF;
}

function fromSigned16(raw) {
  return raw >= 0x8000 ? raw - 0x10000 : raw;
}

function combineHiLo(hi, lo) {
  const value = ((hi << 16) | lo) >>> 0;
  return value >= 0x80000000 ? value - 0x100000000 : value;
}

class DM556RSDrive {
  constructor(modbus, slaveId, options = {}) {
    this.modbus = modbus;
    this.slaveId = slaveId;
    this.interWriteDelayMs = options.interWriteDelayMs ?? 5;
  }

  async _delay(ms) {
    if (ms > 0) await new Promise((resolve) => setTimeout(resolve, ms));
  }

  // ---- status / reads ----

  async readStatus() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.MOTION_STATUS, 1);
    return {
      raw,
      fault: !!(raw & MOTION_STATUS_BIT.FAULT),
      enabled: !!(raw & MOTION_STATUS_BIT.ENABLED),
      running: !!(raw & MOTION_STATUS_BIT.RUNNING),
      cmdComplete: !!(raw & MOTION_STATUS_BIT.CMD_COMPLETE),
      pathComplete: !!(raw & MOTION_STATUS_BIT.PATH_COMPLETE),
      homingComplete: !!(raw & MOTION_STATUS_BIT.HOMING_COMPLETE),
    };
  }

  async readAlarm() {
    const [raw] = await this.modbus.readHoldingRegisters(this.slaveId, REG.ALARM, 1);
    const messages = ALARM_MESSAGES.filter(([bit]) => raw & bit).map(([, msg]) => msg);
    return {
      raw,
      hasAlarm: raw !== 0,
      messages,
      overCurrent: !!(raw & ALARM_BIT.OVER_CURRENT),
      overVoltage: !!(raw & ALARM_BIT.OVER_VOLTAGE),
      currentSamplingFault: !!(raw & ALARM_BIT.CURRENT_SAMPLING_FAULT),
      shaftLockFault: !!(raw & ALARM_BIT.SHAFT_LOCK_FAULT),
      autoTuningFault: !!(raw & ALARM_BIT.AUTO_TUNING_FAULT),
      eepromFault: !!(raw & ALARM_BIT.EEPROM_FAULT),
    };
  }

  async readFeedbackVelocityRpm() {
    const [hi, lo] = await this.modbus.readHoldingRegisters(this.slaveId, REG.VEL_FEEDBACK_HI, 2);
    return combineHiLo(hi, lo);
  }

  // ---- normal run-time controls (never touch EEPROM) ----

  // Caveat: DI1 is normally-closed and self-enables the drive at power-up by default, so this
  // write is very likely a no-op on typical wiring. Callers must trust readStatus().enabled,
  // not the success of this write, to know the drive's real enabled state.
  async setEnable(on) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.SW_ENABLE, on ? 1 : 0);
  }

  async configurePath0Velocity({ rpm, accelMs, decelMs }) {
    const signedRpm = toSigned16(rpm);
    await this.modbus.writeSingleRegister(this.slaveId, REG.PR0_MODE, PR_MODE.VELOCITY);
    await this._delay(this.interWriteDelayMs);
    await this.modbus.writeSingleRegister(this.slaveId, REG.PR0_VEL, signedRpm);
    await this._delay(this.interWriteDelayMs);
    await this.modbus.writeSingleRegister(this.slaveId, REG.PR0_ACC, accelMs);
    await this._delay(this.interWriteDelayMs);
    await this.modbus.writeSingleRegister(this.slaveId, REG.PR0_DEC, decelMs);
  }

  async triggerPath0() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.PR_TRIGGER, PR_TRIGGER_VALUE.RUN_PATH0);
  }

  // direction: DIRECTION.CW or DIRECTION.CCW. Uses a signed velocity register (negative = CCW) -
  // confirmed against real hardware (positive rpm spins CW, negative spins CCW). If a different
  // unit/firmware revision doesn't honor the sign, swap this method's body for the fallback
  // (setDirectionSense() once while idle, or jogCW()/jogCCW()) without touching any other module.
  async startRun(rpm, direction, { accelMs, decelMs }) {
    const magnitude = Math.abs(rpm);
    const signedRpm = direction === DIRECTION.CCW ? -magnitude : magnitude;
    await this.configurePath0Velocity({ rpm: signedRpm, accelMs, decelMs });
    await this.triggerPath0();
  }

  // Graceful stop: re-triggers Path0 at 0 rpm so the configured ease-out ramp (PR0_DEC) is
  // honored, instead of the drive's own quick-stop deceleration used by estop().
  async stopGraceful({ accelMs, decelMs }) {
    await this.configurePath0Velocity({ rpm: 0, accelMs, decelMs });
    await this.triggerPath0();
  }

  // Emergency stop: jumps the queue and uses the drive's own quick-stop deceleration
  // (Pr8.23 / register 0x6017), not our configured ease-out ramp. Available from any state.
  async estop() {
    await this.modbus.jumpQueueWriteSingleRegister(this.slaveId, REG.PR_TRIGGER, PR_TRIGGER_VALUE.ESTOP);
  }

  async clearAlarm() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.CONTROL_WORD, CONTROL_WORD_VALUE.RESET_ALARM);
  }

  // ---- advanced/one-time setup only - always pair with saveToEeprom() and a confirmation UI.
  // Never call these from a normal Start/Stop/RPM/direction code path (finite EEPROM write cycles). ----

  async saveToEeprom() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.CONTROL_WORD, CONTROL_WORD_VALUE.SAVE_EEPROM);
  }

  async setSlaveIdOnWire(id) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.SLAVE_ID, id);
  }

  async setBaudRateOnWire(code) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.BAUD, code);
  }

  async setDataFormatOnWire(code) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.DATA_FORMAT, code);
  }

  // ---- documented-but-not-wired-up fallback primitives (see startRun() note above) ----

  async jogCW() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.CONTROL_WORD, CONTROL_WORD_VALUE.JOG_CW);
  }

  async jogCCW() {
    await this.modbus.writeSingleRegister(this.slaveId, REG.CONTROL_WORD, CONTROL_WORD_VALUE.JOG_CCW);
  }

  async setDirectionSense(direction) {
    await this.modbus.writeSingleRegister(this.slaveId, REG.DIRECTION_SENSE, direction === DIRECTION.CCW ? 1 : 0);
  }

  // ---- raw register access, for the Advanced panel's debug tool (mirrors the Teensy
  // firmware's "read <hex_addr>" / "write <hex_addr> <hex_val>" console commands) ----

  async readRawRegister(addr) {
    const [value] = await this.modbus.readHoldingRegisters(this.slaveId, addr, 1);
    return value;
  }

  async writeRawRegister(addr, value) {
    await this.modbus.writeSingleRegister(this.slaveId, addr, value);
  }
}

export {
  DM556RSDrive,
  REG,
  CONTROL_WORD_VALUE,
  PR_MODE,
  PR_TRIGGER_VALUE,
  MOTION_STATUS_BIT,
  ALARM_BIT,
  ALARM_MESSAGES,
  DIRECTION,
  toSigned16,
  fromSigned16,
  combineHiLo,
};
