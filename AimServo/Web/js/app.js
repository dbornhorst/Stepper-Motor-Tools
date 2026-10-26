import { SerialTransport, ModbusRTU, ModbusTimeoutError, ModbusCrcError, ModbusFramingError, ModbusExceptionError } from './aim-modbus.js';
import { AIMServoDrive, DIRECTION } from './aim-servo.js';
import { loadSettings, saveSettings, resetSettings, MODEL_PRESETS } from './settings.js';

const STATE = {
  DISCONNECTED: 'DISCONNECTED',
  CONNECTED_IDLE: 'CONNECTED_IDLE',
  MOVING: 'MOVING',
  JOGGING: 'JOGGING',
  FAULTED: 'FAULTED',
  CURRENT_TRIP: 'CURRENT_TRIP',
};

// Speed (rpm) at or below this magnitude is treated as "not actually spinning" when reconciling
// state on connect.
const IDLE_RPM_EPSILON = 1;

const el = (id) => document.getElementById(id);

const ui = {
  btnConnect: el('btn-connect'),
  btnDisconnect: el('btn-disconnect'),
  btnChoosePort: el('btn-choose-port'),
  connectionState: el('connection-state'),
  linkLostBanner: el('link-lost-banner'),
  btnReconnect: el('btn-reconnect'),

  statusLight: el('status-light'),
  statusText: el('status-text'),
  statusSpeed: el('status-speed'),
  statusPosition: el('status-position'),
  statusVoltage: el('status-voltage'),
  statusTemp: el('status-temp'),
  statusCurrent: el('status-current'),
  currentBarFill: el('current-bar-fill'),
  statusAlarm: el('status-alarm'),

  stepSize: el('step-size'),
  btnJogNeg: el('btn-jog-neg'),
  btnJogPos: el('btn-jog-pos'),
  absoluteTarget: el('absolute-target'),
  btnMoveAbsolute: el('btn-move-absolute'),
  btnZeroPosition: el('btn-zero-position'),

  jogRpmSlider: el('jog-rpm-slider'),
  jogRpmNumber: el('jog-rpm-number'),
  jogDirectionRadios: document.getElementsByName('jog-direction'),
  jogDirectionHint: el('jog-direction-hint'),
  btnJogStart: el('btn-jog-start'),
  btnJogStop: el('btn-jog-stop'),

  btnDisableOutput: el('btn-disable-output'),

  faultPanel: el('fault-panel'),
  faultMessage: el('fault-message'),
  btnClearFault: el('btn-clear-fault'),

  tripPanel: el('trip-panel'),
  btnResumeTrip: el('btn-resume-trip'),

  settingSlaveId: el('setting-slave-id'),
  settingModel: el('setting-model'),
  settingRpmMin: el('setting-rpm-min'),
  settingRpmMax: el('setting-rpm-max'),
  settingAccel: el('setting-accel'),
  settingCurrentCutoff: el('setting-current-cutoff'),
  settingDirectionMethod: el('setting-direction-method'),
  settingPositiveDirPolarity: el('setting-positive-dir-polarity'),
  settingPollInterval: el('setting-poll-interval'),
  settingRequestTimeout: el('setting-request-timeout'),
  settingInterWriteDelay: el('setting-inter-write-delay'),
  settingLinkLostThreshold: el('setting-link-lost-threshold'),
  btnSaveSettings: el('btn-save-settings'),
  btnResetSettings: el('btn-reset-settings'),

  dangerGearNum: el('danger-gear-num'),
  dangerGearDen: el('danger-gear-den'),
  btnDangerGear: el('btn-danger-gear'),
  btnDangerPersistentComm: el('btn-danger-persistent-comm'),

  rawReadAddr: el('raw-read-addr'),
  btnRawRead: el('btn-raw-read'),
  rawReadResult: el('raw-read-result'),
  rawWriteAddr: el('raw-write-addr'),
  rawWriteValue: el('raw-write-value'),
  btnRawWrite: el('btn-raw-write'),
  rawWriteResult: el('raw-write-result'),
};

const app = {
  settings: loadSettings(),
  state: STATE.DISCONNECTED,
  linkLost: false,
  missedPollCount: 0,
  pollTimer: null,
  pollCycle: 0,
  jogDebounceTimer: null,

  transport: null,
  modbus: null,
  drive: null,

  currentJogRpm: 100,
  currentJogDirection: DIRECTION.CW,
};

function main() {
  if (!SerialTransport.isSupported()) {
    el('unsupported-notice').hidden = false;
    el('app').hidden = true;
    return;
  }

  populateSettingSelects();
  applySettingsToForm();
  applySettingsToControlLimits();
  wireEvents();
  renderState();

  // Best-effort: closing the port on navigation/tab-close isn't guaranteed to finish, but
  // attempting it reduces the chance of leaving the OS-level port handle locked.
  window.addEventListener('pagehide', () => {
    if (app.transport) app.transport.close().catch(() => {});
  });
}

function populateSettingSelects() {
  ui.settingModel.innerHTML = '';
  for (const model of Object.keys(MODEL_PRESETS)) {
    const opt = document.createElement('option');
    opt.value = model;
    opt.textContent = model;
    ui.settingModel.appendChild(opt);
  }
}

function applySettingsToForm() {
  ui.settingSlaveId.value = app.settings.slaveId;
  ui.settingModel.value = app.settings.model;
  ui.settingRpmMin.value = app.settings.rpmMin;
  ui.settingRpmMax.value = app.settings.rpmMax;
  ui.settingAccel.value = app.settings.accel;
  ui.settingCurrentCutoff.value = app.settings.currentCutoffA;
  ui.settingDirectionMethod.value = app.settings.directionMethod;
  ui.settingPositiveDirPolarity.value = String(app.settings.positiveDirPolarity);
  ui.settingPollInterval.value = app.settings.pollIntervalMs;
  ui.settingRequestTimeout.value = app.settings.requestTimeoutMs;
  ui.settingInterWriteDelay.value = app.settings.interWriteDelayMs;
  ui.settingLinkLostThreshold.value = app.settings.linkLostThreshold;
}

function applySettingsToControlLimits() {
  ui.jogRpmSlider.min = app.settings.rpmMin;
  ui.jogRpmSlider.max = app.settings.rpmMax;
  ui.jogRpmNumber.min = app.settings.rpmMin;
  ui.jogRpmNumber.max = app.settings.rpmMax;
  const clamped = clamp(app.currentJogRpm, app.settings.rpmMin, app.settings.rpmMax);
  app.currentJogRpm = clamped;
  ui.jogRpmSlider.value = clamped;
  ui.jogRpmNumber.value = clamped;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function directionOptions() {
  return {
    directionMethod: app.settings.directionMethod,
    positiveDirPolarity: app.settings.positiveDirPolarity,
    maxRpm: app.settings.rpmMax,
  };
}

// ---------------------------------------------------------------------------
// Connection lifecycle
// ---------------------------------------------------------------------------

async function connect() {
  ui.btnConnect.disabled = true;
  try {
    app.transport = new SerialTransport();
    app.transport.onDisconnect(onLinkDown);
    const portOptions = { baudRate: 19200, parity: 'none', stopBits: 1 };
    // Reuse a previously-granted port silently if one exists, only falling back to the picker
    // if there isn't one - avoids re-prompting on every reload for a device already approved.
    const reused = await app.transport.openKnownPort(portOptions);
    if (!reused) {
      await app.transport.requestAndOpen(portOptions);
    }

    app.modbus = new ModbusRTU(app.transport, { requestTimeoutMs: app.settings.requestTimeoutMs });
    app.drive = new AIMServoDrive(app.modbus, app.settings.slaveId, {
      interWriteDelayMs: app.settings.interWriteDelayMs,
    });

    // Sync real state rather than assuming idle.
    const alarm = await app.drive.readAlarm();
    const outputEnable = await app.drive.readOutputEnable();
    const speed = await app.drive.readSpeedRpm();
    updateSpeedDisplay(speed);

    if (alarm.hasAlarm) {
      app.state = STATE.FAULTED;
      showFault(alarm);
    } else if (outputEnable && Math.abs(speed) > IDLE_RPM_EPSILON) {
      app.state = STATE.JOGGING;
      app.currentJogDirection = speed < 0 ? DIRECTION.CCW : DIRECTION.CW;
      app.currentJogRpm = clamp(Math.abs(speed), app.settings.rpmMin, app.settings.rpmMax);
    } else {
      app.state = STATE.CONNECTED_IDLE;
    }

    app.linkLost = false;
    app.missedPollCount = 0;
    startPolling();
    renderState();
  } catch (err) {
    handleError(err, 'connect');
    await teardownConnection();
  } finally {
    ui.btnConnect.disabled = false;
  }
}

async function disconnect() {
  stopPolling();
  await teardownConnection();
  app.state = STATE.DISCONNECTED;
  app.linkLost = false;
  renderState();
}

async function teardownConnection() {
  if (app.modbus) { app.modbus.dispose(); app.modbus = null; }
  if (app.transport) { await app.transport.close().catch(() => {}); app.transport = null; }
  app.drive = null;
}

function onLinkDown() {
  app.linkLost = true;
  renderState();
}

async function reconnect() {
  ui.btnReconnect.disabled = true;
  try {
    await teardownConnection();
    stopPolling();
    app.state = STATE.DISCONNECTED;
    await connect();
  } finally {
    ui.btnReconnect.disabled = false;
  }
}

async function chooseDifferentPort() {
  ui.btnChoosePort.disabled = true;
  try {
    if (app.state !== STATE.DISCONNECTED) {
      await disconnect();
    }
    await SerialTransport.forgetAllKnownPorts();
    await connect();
  } finally {
    ui.btnChoosePort.disabled = false;
  }
}

// ---------------------------------------------------------------------------
// Status polling (tiered: current/alarm/output/speed every cycle, voltage/temp every 2nd,
// position every 4th or continuously while a move is being watched separately)
// ---------------------------------------------------------------------------

function startPolling() {
  stopPolling();
  app.pollCycle = 0;
  scheduleNextPoll(0);
}

function stopPolling() {
  if (app.pollTimer) { clearTimeout(app.pollTimer); app.pollTimer = null; }
}

function scheduleNextPoll(delay) {
  app.pollTimer = setTimeout(pollOnce, delay);
}

async function pollOnce() {
  if (!app.drive) return;
  app.pollCycle++;
  try {
    const current = await app.drive.readCurrentAmps();
    const alarm = await app.drive.readAlarm();
    const outputEnable = await app.drive.readOutputEnable();
    const speed = await app.drive.readSpeedRpm();
    let voltage = null;
    let temp = null;
    let position = null;
    if (app.pollCycle % 2 === 0) {
      voltage = await app.drive.readVoltage();
      temp = await app.drive.readTemperature();
    }
    if (app.pollCycle % 4 === 0 || app.state === STATE.MOVING) {
      position = await app.drive.readAbsolutePosition();
    }

    app.missedPollCount = 0;
    if (app.linkLost) app.linkLost = false;

    await applyPollResult({ current, alarm, outputEnable, speed, voltage, temp, position });
  } catch (err) {
    app.missedPollCount++;
    if (app.missedPollCount >= app.settings.linkLostThreshold) {
      app.linkLost = true;
    }
  } finally {
    renderState();
    if (app.state !== STATE.DISCONNECTED) {
      scheduleNextPoll(app.settings.pollIntervalMs);
    }
  }
}

async function applyPollResult(result) {
  updateCurrentDisplay(result.current);
  updateSpeedDisplay(result.speed);
  if (result.voltage !== null) ui.statusVoltage.textContent = `${result.voltage.toFixed(1)} V`;
  if (result.temp !== null) ui.statusTemp.textContent = `${result.temp} °C`;
  if (result.position !== null) ui.statusPosition.textContent = String(result.position);

  const inTripLikeState = app.state === STATE.FAULTED || app.state === STATE.CURRENT_TRIP;

  // Current-cutoff safety check runs every poll regardless of believed state, for defense in
  // depth - not gated on whether the app thinks something is moving.
  if (result.outputEnable && result.current >= app.settings.currentCutoffA && app.state !== STATE.CURRENT_TRIP) {
    try {
      await app.drive.emergencyDisableOutput();
    } catch (err) {
      // Still transition to CURRENT_TRIP even if the protective write itself failed - the
      // operator needs to know something is wrong either way.
    }
    app.state = STATE.CURRENT_TRIP;
    return;
  }

  if (result.alarm.hasAlarm && !inTripLikeState) {
    app.state = STATE.FAULTED;
    showFault(result.alarm);
  }
}

function updateCurrentDisplay(current) {
  ui.statusCurrent.textContent = `${current.toFixed(3)} A`;
  const pct = clamp((current / app.settings.currentCutoffA) * 100, 0, 100);
  ui.currentBarFill.style.width = `${pct}%`;
  ui.currentBarFill.classList.remove('current-high', 'current-trip');
  if (current >= app.settings.currentCutoffA) {
    ui.currentBarFill.classList.add('current-trip');
  } else if (pct >= 70) {
    ui.currentBarFill.classList.add('current-high');
  }
}

function updateSpeedDisplay(speed) {
  ui.statusSpeed.textContent = `${speed.toFixed(1)} rpm`;
}

// ---------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------

function getSelectedJogDirection() {
  for (const radio of ui.jogDirectionRadios) {
    if (radio.checked) return radio.value;
  }
  return DIRECTION.CW;
}

async function handleMoveRelative(sign) {
  if (app.state !== STATE.CONNECTED_IDLE) return;
  const step = Number(ui.stepSize.value) * sign;
  app.state = STATE.MOVING;
  renderState();
  try {
    await app.drive.moveRelative(step);
    const ok = await app.drive.waitUntilStopped();
    if (!ok) showTransientMessage('Move sent, but the drive never reported a clean stop - check position.');
  } catch (err) {
    handleError(err, 'move-relative');
  } finally {
    app.state = STATE.CONNECTED_IDLE;
    renderState();
  }
}

async function handleMoveAbsolute() {
  if (app.state !== STATE.CONNECTED_IDLE) return;
  const target = Number(ui.absoluteTarget.value);
  app.state = STATE.MOVING;
  renderState();
  try {
    await app.drive.moveAbsolute(target);
    const ok = await app.drive.waitUntilPositionReached(target);
    if (!ok) showTransientMessage('Move sent, but the drive never settled within tolerance - check position.');
  } catch (err) {
    handleError(err, 'move-absolute');
  } finally {
    app.state = STATE.CONNECTED_IDLE;
    renderState();
  }
}

async function handleZeroPosition() {
  if (app.state !== STATE.CONNECTED_IDLE) return;
  if (!window.confirm('Zero the position counter at the current shaft position?')) return;
  try {
    await app.drive.zeroPosition({ waitUntilStopped: () => app.drive.waitUntilStopped() });
  } catch (err) {
    handleError(err, 'zero-position');
  } finally {
    renderState();
  }
}

async function handleJogStart() {
  if (app.state !== STATE.CONNECTED_IDLE) return;
  const rpm = clamp(Number(ui.jogRpmNumber.value), app.settings.rpmMin, app.settings.rpmMax);
  const direction = getSelectedJogDirection();
  ui.btnJogStart.disabled = true;
  try {
    await app.drive.startSpeedJog(rpm, direction, directionOptions());
    app.currentJogRpm = rpm;
    app.currentJogDirection = direction;
    app.state = STATE.JOGGING;
  } catch (err) {
    handleError(err, 'jog-start');
  } finally {
    renderState();
  }
}

async function handleJogStop() {
  if (app.state !== STATE.JOGGING) return;
  ui.btnJogStop.disabled = true;
  try {
    await app.drive.stopSpeedJog(directionOptions());
    app.state = STATE.CONNECTED_IDLE;
  } catch (err) {
    handleError(err, 'jog-stop');
  } finally {
    renderState();
  }
}

function scheduleJogUpdate() {
  if (app.state !== STATE.JOGGING) return;
  clearTimeout(app.jogDebounceTimer);
  app.jogDebounceTimer = setTimeout(async () => {
    try {
      await app.drive.updateSpeedJog(app.currentJogRpm, app.currentJogDirection, directionOptions());
    } catch (err) {
      handleError(err, 'jog-update');
    }
  }, 200);
}

async function handleDisableOutputNow() {
  if (app.state === STATE.DISCONNECTED) return;
  try {
    await app.drive.emergencyDisableOutput();
    // Also reset target speed / return to position mode so a later re-enable doesn't resume
    // spinning at whatever speed was last commanded.
    await app.drive.setTargetSpeedForDirection(0, DIRECTION.CW, directionOptions()).catch(() => {});
    await app.drive.setPositionMode().catch(() => {});
  } catch (err) {
    handleError(err, 'disable-output');
  } finally {
    if (app.state !== STATE.FAULTED && app.state !== STATE.CURRENT_TRIP) {
      app.state = STATE.CONNECTED_IDLE;
    }
    renderState();
  }
}

async function handleClearFault() {
  if (app.state !== STATE.FAULTED) return;
  ui.btnClearFault.disabled = true;
  try {
    const alarm = await app.drive.readAlarm();
    if (!alarm.hasAlarm) {
      app.state = STATE.CONNECTED_IDLE;
    } else {
      showFault(alarm);
      showTransientMessage('Fault persists - check the hardware before resuming.');
    }
  } catch (err) {
    handleError(err, 'clear-fault');
  } finally {
    ui.btnClearFault.disabled = false;
    renderState();
  }
}

function handleResumeTrip() {
  if (app.state !== STATE.CURRENT_TRIP) return;
  app.state = STATE.CONNECTED_IDLE;
  renderState();
}

function showFault(alarm) {
  app.state = STATE.FAULTED;
  ui.faultMessage.textContent = alarm.message ? `Fault: ${alarm.message}` : 'Fault reported by the drive.';
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

function renderState() {
  const connected = app.state !== STATE.DISCONNECTED;

  ui.btnConnect.hidden = connected;
  ui.btnDisconnect.hidden = !connected;
  ui.connectionState.textContent = connected ? `Connected (state: ${app.state})` : 'Not connected';

  ui.linkLostBanner.hidden = !app.linkLost;

  const idleAndReady = app.state === STATE.CONNECTED_IDLE && !app.linkLost;
  ui.btnJogNeg.disabled = !idleAndReady;
  ui.btnJogPos.disabled = !idleAndReady;
  ui.btnMoveAbsolute.disabled = !idleAndReady;
  ui.btnZeroPosition.disabled = !idleAndReady;

  ui.btnJogStart.disabled = !idleAndReady;
  ui.btnJogStop.disabled = app.state !== STATE.JOGGING || app.linkLost;
  const directionDisabled = app.state === STATE.JOGGING;
  for (const radio of ui.jogDirectionRadios) radio.disabled = directionDisabled;
  ui.jogDirectionHint.hidden = !directionDisabled;

  ui.btnDisableOutput.disabled = !connected;

  ui.faultPanel.hidden = app.state !== STATE.FAULTED;
  ui.tripPanel.hidden = app.state !== STATE.CURRENT_TRIP;

  const rawToolsEnabled = connected;
  ui.btnRawRead.disabled = !rawToolsEnabled;
  ui.btnRawWrite.disabled = !rawToolsEnabled;
  ui.btnDangerGear.disabled = !rawToolsEnabled;
  ui.btnDangerPersistentComm.disabled = !rawToolsEnabled;

  renderStatusLight();
}

function renderStatusLight() {
  ui.statusLight.classList.remove('status-idle', 'status-running', 'status-fault');
  switch (app.state) {
    case STATE.JOGGING:
      ui.statusLight.classList.add('status-running');
      ui.statusText.textContent = 'Jogging';
      break;
    case STATE.MOVING:
      ui.statusLight.classList.add('status-running');
      ui.statusText.textContent = 'Moving…';
      break;
    case STATE.FAULTED:
      ui.statusLight.classList.add('status-fault');
      ui.statusText.textContent = 'Fault';
      break;
    case STATE.CURRENT_TRIP:
      ui.statusLight.classList.add('status-fault');
      ui.statusText.textContent = 'Current trip';
      break;
    case STATE.DISCONNECTED:
      ui.statusLight.classList.add('status-idle');
      ui.statusText.textContent = 'Disconnected';
      break;
    default:
      ui.statusLight.classList.add('status-idle');
      ui.statusText.textContent = 'Idle';
  }
}

function showTransientMessage(message) {
  console.warn(message);
  ui.connectionState.textContent = message;
  setTimeout(() => renderState(), 4000);
}

const ERROR_MESSAGES = {
  ModbusExceptionError: (err) => {
    switch (err.exceptionCode) {
      case 0x01: return 'The drive rejected that command (unsupported function). Please report this as a bug.';
      case 0x02: return "The drive said that setting doesn't exist. Check the Slave ID in Settings.";
      case 0x03: return 'The drive rejected that value as out of range.';
      case 0x08: return 'Communication error (bad CRC reported by the drive). Check wiring.';
      default: return `Drive reported an error (code 0x${err.exceptionCode.toString(16)}).`;
    }
  },
  ModbusTimeoutError: () => 'No response from the drive. Check: correct serial port, Slave ID, and RS-485 wiring (this drive is fixed at 19200 8N1).',
  ModbusCrcError: () => 'Communication error (corrupted data). Check RS-485 wiring and the Slave ID in Settings.',
  ModbusFramingError: () => 'Communication error (unexpected response). Check wiring and Settings.',
};

function handleError(err, context) {
  console.error(`[${context}]`, err);
  const formatter = ERROR_MESSAGES[err && err.name];
  const message = formatter ? formatter(err) : (err && err.message) || 'Unknown error.';
  showTransientMessage(message);
}

// ---------------------------------------------------------------------------
// Event wiring
// ---------------------------------------------------------------------------

function wireEvents() {
  ui.btnConnect.addEventListener('click', connect);
  ui.btnDisconnect.addEventListener('click', disconnect);
  ui.btnChoosePort.addEventListener('click', chooseDifferentPort);
  ui.btnReconnect.addEventListener('click', reconnect);

  ui.btnJogNeg.addEventListener('click', () => handleMoveRelative(-1));
  ui.btnJogPos.addEventListener('click', () => handleMoveRelative(1));
  ui.btnMoveAbsolute.addEventListener('click', handleMoveAbsolute);
  ui.btnZeroPosition.addEventListener('click', handleZeroPosition);

  ui.jogRpmSlider.addEventListener('input', () => {
    app.currentJogRpm = clamp(Number(ui.jogRpmSlider.value), app.settings.rpmMin, app.settings.rpmMax);
    ui.jogRpmNumber.value = app.currentJogRpm;
    scheduleJogUpdate();
  });
  ui.jogRpmNumber.addEventListener('change', () => {
    app.currentJogRpm = clamp(Number(ui.jogRpmNumber.value), app.settings.rpmMin, app.settings.rpmMax);
    ui.jogRpmSlider.value = app.currentJogRpm;
    ui.jogRpmNumber.value = app.currentJogRpm;
    scheduleJogUpdate();
  });
  for (const radio of ui.jogDirectionRadios) {
    radio.addEventListener('change', () => {
      app.currentJogDirection = getSelectedJogDirection();
    });
  }
  ui.btnJogStart.addEventListener('click', handleJogStart);
  ui.btnJogStop.addEventListener('click', handleJogStop);

  ui.btnDisableOutput.addEventListener('click', handleDisableOutputNow);
  ui.btnClearFault.addEventListener('click', handleClearFault);
  ui.btnResumeTrip.addEventListener('click', handleResumeTrip);

  ui.settingModel.addEventListener('change', () => {
    const preset = MODEL_PRESETS[ui.settingModel.value];
    if (preset !== null && preset !== undefined) {
      ui.settingRpmMax.value = preset;
    }
  });

  ui.btnSaveSettings.addEventListener('click', () => {
    app.settings.slaveId = Number(ui.settingSlaveId.value);
    app.settings.model = ui.settingModel.value;
    app.settings.rpmMin = Number(ui.settingRpmMin.value);
    app.settings.rpmMax = Number(ui.settingRpmMax.value);
    app.settings.accel = Number(ui.settingAccel.value);
    app.settings.currentCutoffA = Number(ui.settingCurrentCutoff.value);
    app.settings.directionMethod = ui.settingDirectionMethod.value;
    app.settings.positiveDirPolarity = Number(ui.settingPositiveDirPolarity.value);
    app.settings.pollIntervalMs = Number(ui.settingPollInterval.value);
    app.settings.requestTimeoutMs = Number(ui.settingRequestTimeout.value);
    app.settings.interWriteDelayMs = Number(ui.settingInterWriteDelay.value);
    app.settings.linkLostThreshold = Number(ui.settingLinkLostThreshold.value);
    if (app.settings.rpmMin >= app.settings.rpmMax) {
      showTransientMessage('RPM min must be less than RPM max - settings not saved.');
      return;
    }
    saveSettings(app.settings);
    applySettingsToControlLimits();
    showTransientMessage('Settings saved. Reconnect for slave ID changes to take effect.');
  });

  ui.btnResetSettings.addEventListener('click', () => {
    app.settings = resetSettings();
    applySettingsToForm();
    applySettingsToControlLimits();
  });

  ui.btnDangerGear.addEventListener('click', () => runDangerAction(
    `Write electronic gear ${ui.dangerGearNum.value}/${ui.dangerGearDen.value} to the drive and save to EEPROM?`,
    async () => {
      await app.drive.setElectronicGear(Number(ui.dangerGearNum.value), Number(ui.dangerGearDen.value));
      await app.drive.saveParameters();
    },
  ));

  ui.btnDangerPersistentComm.addEventListener('click', () => runDangerAction(
    'Set persistent Modbus comm-mode on power-up (electronic gear numerator = 0) and save to EEPROM?',
    async () => {
      await app.drive.setCommModePersistent();
      await app.drive.saveParameters();
    },
  ));

  ui.btnRawRead.addEventListener('click', async () => {
    try {
      const addr = parseInt(ui.rawReadAddr.value, 16);
      const value = await app.drive.readRawRegister(addr);
      ui.rawReadResult.textContent = `= 0x${value.toString(16).padStart(4, '0')} (${value})`;
    } catch (err) {
      ui.rawReadResult.textContent = `error: ${err.message}`;
    }
  });

  ui.btnRawWrite.addEventListener('click', async () => {
    try {
      const addr = parseInt(ui.rawWriteAddr.value, 16);
      const value = parseInt(ui.rawWriteValue.value, 16);
      await app.drive.writeRawRegister(addr, value);
      ui.rawWriteResult.textContent = 'OK';
    } catch (err) {
      ui.rawWriteResult.textContent = `error: ${err.message}`;
    }
  });
}

async function runDangerAction(confirmMessage, action) {
  if (!window.confirm(confirmMessage)) return;
  stopPolling();
  try {
    await action();
    window.alert('Write complete.');
  } catch (err) {
    handleError(err, 'danger-zone');
  } finally {
    if (app.state !== STATE.DISCONNECTED) startPolling();
  }
}

main();
