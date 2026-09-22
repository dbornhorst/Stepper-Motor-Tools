import { SerialTransport, ModbusRTU, ModbusTimeoutError, ModbusCrcError, ModbusFramingError, ModbusExceptionError } from './modbus-serial.js';
import { DM556RSDrive, DIRECTION } from './dm556rs.js';
import { loadSettings, saveSettings, resetSettings, DATA_FORMAT_OPTIONS, BAUD_RATE_OPTIONS, BAUD_RATE_CODES } from './settings.js';

// Feedback velocity (rpm) at or below this magnitude is treated as "actually stopped" - see
// the STOPPING case in applyPollResult().
const STOP_RPM_EPSILON = 1;

const STATE = {
  DISCONNECTED: 'DISCONNECTED',
  CONNECTED_IDLE: 'CONNECTED_IDLE',
  RUNNING: 'RUNNING',
  STOPPING: 'STOPPING',
  FAULTED: 'FAULTED',
  ESTOP_LATCHED: 'ESTOP_LATCHED',
};

const el = (id) => document.getElementById(id);

const ui = {
  btnConnect: el('btn-connect'),
  btnDisconnect: el('btn-disconnect'),
  connectionState: el('connection-state'),
  linkLostBanner: el('link-lost-banner'),
  btnReconnect: el('btn-reconnect'),

  statusLight: el('status-light'),
  statusText: el('status-text'),
  statusCommanded: el('status-commanded'),
  statusFeedbackRpm: el('status-feedback-rpm'),
  statusAlarm: el('status-alarm'),

  rpmSlider: el('rpm-slider'),
  rpmNumber: el('rpm-number'),
  directionRadios: document.getElementsByName('direction'),
  easeInSlider: el('ease-in-slider'),
  easeInNumber: el('ease-in-number'),
  easeOutSlider: el('ease-out-slider'),
  easeOutNumber: el('ease-out-number'),

  btnStart: el('btn-start'),
  btnStop: el('btn-stop'),
  btnEstop: el('btn-estop'),

  faultPanel: el('fault-panel'),
  faultMessage: el('fault-message'),
  btnClearFault: el('btn-clear-fault'),

  estopPanel: el('estop-panel'),
  btnResume: el('btn-resume'),

  settingSlaveId: el('setting-slave-id'),
  settingBaudRate: el('setting-baud-rate'),
  settingDataFormat: el('setting-data-format'),
  settingRpmMin: el('setting-rpm-min'),
  settingRpmMax: el('setting-rpm-max'),
  settingPollInterval: el('setting-poll-interval'),
  settingRequestTimeout: el('setting-request-timeout'),
  settingInterWriteDelay: el('setting-inter-write-delay'),
  settingLinkLostThreshold: el('setting-link-lost-threshold'),
  btnSaveSettings: el('btn-save-settings'),
  btnResetSettings: el('btn-reset-settings'),

  dangerSlaveId: el('danger-slave-id'),
  btnDangerSlaveId: el('btn-danger-slave-id'),
  dangerBaudRate: el('danger-baud-rate'),
  btnDangerBaudRate: el('btn-danger-baud-rate'),
  dangerDataFormat: el('danger-data-format'),
  btnDangerDataFormat: el('btn-danger-data-format'),

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
  stopTimeoutTimer: null,
  rpmDebounceTimer: null,
  easeDebounceTimer: null,

  transport: null,
  modbus: null,
  drive: null,

  currentRpm: 60,
  currentDirection: DIRECTION.CW,
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

  // Best-effort: closing the port on navigation/tab-close isn't guaranteed to finish (async
  // work during unload has no completion guarantee), but attempting it reduces the chance of
  // leaving the OS-level port handle locked, which otherwise requires unplugging/replugging the
  // adapter before it can be opened again (confirmed hitting this during manual testing).
  window.addEventListener('pagehide', () => {
    if (app.transport) app.transport.close().catch(() => {});
  });
}

function populateSettingSelects() {
  for (const select of [ui.settingBaudRate, ui.dangerBaudRate]) {
    select.innerHTML = '';
    for (const rate of BAUD_RATE_OPTIONS) {
      const opt = document.createElement('option');
      opt.value = String(rate);
      opt.textContent = String(rate);
      select.appendChild(opt);
    }
  }
  for (const select of [ui.settingDataFormat, ui.dangerDataFormat]) {
    select.innerHTML = '';
    for (const [key, info] of Object.entries(DATA_FORMAT_OPTIONS)) {
      const opt = document.createElement('option');
      opt.value = key;
      opt.textContent = info.label;
      select.appendChild(opt);
    }
  }
}

function applySettingsToForm() {
  ui.settingSlaveId.value = app.settings.slaveId;
  ui.settingBaudRate.value = String(app.settings.baudRate);
  ui.settingDataFormat.value = app.settings.dataFormat;
  ui.settingRpmMin.value = app.settings.rpmMin;
  ui.settingRpmMax.value = app.settings.rpmMax;
  ui.settingPollInterval.value = app.settings.pollIntervalMs;
  ui.settingRequestTimeout.value = app.settings.requestTimeoutMs;
  ui.settingInterWriteDelay.value = app.settings.interWriteDelayMs;
  ui.settingLinkLostThreshold.value = app.settings.linkLostThreshold;

  ui.easeInSlider.value = app.settings.easeInMs;
  ui.easeInNumber.value = app.settings.easeInMs;
  ui.easeOutSlider.value = app.settings.easeOutMs;
  ui.easeOutNumber.value = app.settings.easeOutMs;
}

function applySettingsToControlLimits() {
  ui.rpmSlider.min = app.settings.rpmMin;
  ui.rpmSlider.max = app.settings.rpmMax;
  ui.rpmNumber.min = app.settings.rpmMin;
  ui.rpmNumber.max = app.settings.rpmMax;
  const clamped = clamp(app.currentRpm, app.settings.rpmMin, app.settings.rpmMax);
  app.currentRpm = clamped;
  ui.rpmSlider.value = clamped;
  ui.rpmNumber.value = clamped;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

// ---------------------------------------------------------------------------
// Connection lifecycle
// ---------------------------------------------------------------------------

async function connect() {
  ui.btnConnect.disabled = true;
  try {
    const formatInfo = DATA_FORMAT_OPTIONS[app.settings.dataFormat];
    app.transport = new SerialTransport();
    app.transport.onDisconnect(onLinkDown);
    const portOptions = {
      baudRate: app.settings.baudRate,
      parity: formatInfo.parity,
      stopBits: formatInfo.stopBits,
    };
    // Reuse a previously-granted port silently if one exists (Chrome remembers the grant per
    // origin+device), only falling back to the picker if there isn't one - avoids re-prompting
    // on every reload/reconnect for a device already approved.
    const reused = await app.transport.openKnownPort(portOptions);
    if (!reused) {
      await app.transport.requestAndOpen(portOptions);
    }

    app.modbus = new ModbusRTU(app.transport, {
      requestTimeoutMs: app.settings.requestTimeoutMs,
    });
    app.drive = new DM556RSDrive(app.modbus, app.settings.slaveId, {
      interWriteDelayMs: app.settings.interWriteDelayMs,
    });

    // Sync real state rather than assuming idle - another controller or a prior session may
    // have left the drive running or faulted.
    const status = await app.drive.readStatus();
    const alarm = await app.drive.readAlarm();
    // The "running" bit reflects "a motion path is active," which stays set even at 0 rpm (the
    // same quirk that affects graceful-stop detection - see applyPollResult()). Confirmed on
    // real hardware: reconnecting after a stop showed running=true with the shaft stationary.
    // So corroborate with actual feedback speed before trusting the bit here too.
    const feedbackRpm = await app.drive.readFeedbackVelocityRpm();
    ui.statusFeedbackRpm.textContent = `${feedbackRpm} rpm`;
    const actuallyRunning = status.running && Math.abs(feedbackRpm) > STOP_RPM_EPSILON;

    if (status.fault || alarm.hasAlarm) {
      app.state = STATE.FAULTED;
      showFault(alarm);
    } else if (actuallyRunning) {
      app.state = STATE.RUNNING;
      ui.statusCommanded.textContent = 'Running (reconnected - speed/direction as last set on the drive)';
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

// ---------------------------------------------------------------------------
// Status polling
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
    const status = await app.drive.readStatus();
    const alarm = await app.drive.readAlarm();
    let feedbackRpm = null;
    // While stopping, we need actual speed every cycle (not just every 4th) to detect real
    // standstill - see the STOPPING handling in applyPollResult() for why.
    if (app.pollCycle % 4 === 0 || app.state === STATE.STOPPING) {
      feedbackRpm = await app.drive.readFeedbackVelocityRpm();
    }

    app.missedPollCount = 0;
    if (app.linkLost) app.linkLost = false;

    applyPollResult(status, alarm, feedbackRpm);
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

function applyPollResult(status, alarm, feedbackRpm) {
  if (feedbackRpm !== null) {
    ui.statusFeedbackRpm.textContent = `${feedbackRpm} rpm`;
  }

  const inFaultLikeState = app.state === STATE.FAULTED || app.state === STATE.ESTOP_LATCHED;

  if ((status.fault || alarm.hasAlarm) && !inFaultLikeState) {
    app.state = STATE.FAULTED;
    showFault(alarm);
    return;
  }

  if (app.state === STATE.STOPPING) {
    // The "running" status bit reflects whether a motion path is active, not whether the shaft
    // is actually turning - on this driver it can stay set even after a velocity-mode path is
    // re-triggered at 0 rpm (which is how a graceful stop is implemented). So treat "actual
    // speed reads ~0" as the real stop signal, and the status bit clearing as a bonus signal,
    // whichever comes first.
    const atStandstill = feedbackRpm !== null && Math.abs(feedbackRpm) <= STOP_RPM_EPSILON;
    if (!status.running || atStandstill) {
      clearStopTimeout();
      app.state = STATE.CONNECTED_IDLE;
    }
    return;
  }

  if (app.state === STATE.RUNNING && !status.running) {
    // Drive stopped on its own (fault cleared elsewhere, external stop, etc.)
    app.state = STATE.CONNECTED_IDLE;
  }
}

// ---------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------

function getSelectedDirection() {
  for (const radio of ui.directionRadios) {
    if (radio.checked) return radio.value;
  }
  return DIRECTION.CW;
}

async function handleStart() {
  if (app.state !== STATE.CONNECTED_IDLE) return;
  const rpm = clamp(app.currentRpm, app.settings.rpmMin, app.settings.rpmMax);
  const direction = getSelectedDirection();
  ui.btnStart.disabled = true;
  try {
    await app.drive.startRun(rpm, direction, {
      accelMs: app.settings.easeInMs,
      decelMs: app.settings.easeOutMs,
    });
    app.currentRpm = rpm;
    app.currentDirection = direction;
    app.state = STATE.RUNNING;
  } catch (err) {
    handleError(err, 'start');
  } finally {
    renderState();
  }
}

async function handleStop() {
  if (app.state !== STATE.RUNNING) return;
  ui.btnStop.disabled = true;
  try {
    await app.drive.stopGraceful({
      accelMs: app.settings.easeInMs,
      decelMs: app.settings.easeOutMs,
    });
    app.state = STATE.STOPPING;
    const timeoutMs = Math.max(3000, app.settings.easeOutMs + 2000);
    clearStopTimeout();
    app.stopTimeoutTimer = setTimeout(() => {
      if (app.state === STATE.STOPPING) {
        showTransientMessage('Stop was sent but the driver still reports running. Consider pressing E-Stop.');
      }
    }, timeoutMs);
  } catch (err) {
    handleError(err, 'stop');
  } finally {
    renderState();
  }
}

function clearStopTimeout() {
  if (app.stopTimeoutTimer) { clearTimeout(app.stopTimeoutTimer); app.stopTimeoutTimer = null; }
}

async function handleEstop() {
  if (app.state === STATE.DISCONNECTED) return;
  try {
    await app.drive.estop();
  } catch (err) {
    // Even if the write failed/timed out, we still latch ESTOP_LATCHED locally so the operator
    // sees the app treat this as an emergency, and must explicitly Resume.
    handleError(err, 'estop');
  } finally {
    clearStopTimeout();
    app.state = STATE.ESTOP_LATCHED;
    renderState();
  }
}

async function handleResume() {
  if (app.state !== STATE.ESTOP_LATCHED && app.state !== STATE.FAULTED) return;
  ui.btnResume.disabled = true;
  ui.btnClearFault.disabled = true;
  try {
    const alarm = await app.drive.readAlarm();
    if (alarm.hasAlarm) {
      await app.drive.clearAlarm();
    }
    const confirmStatus = await app.drive.readStatus();
    const confirmAlarm = await app.drive.readAlarm();
    if (!confirmStatus.fault && !confirmAlarm.hasAlarm) {
      app.state = STATE.CONNECTED_IDLE;
    } else {
      showFault(confirmAlarm);
      showTransientMessage('Fault persists after clearing - check the hardware.');
    }
  } catch (err) {
    handleError(err, 'resume');
  } finally {
    ui.btnResume.disabled = false;
    ui.btnClearFault.disabled = false;
    renderState();
  }
}

function showFault(alarm) {
  app.state = STATE.FAULTED;
  ui.faultMessage.textContent = alarm.messages.length
    ? `Fault: ${alarm.messages.join(', ')}`
    : 'Fault reported by the driver (status fault bit set).';
}

function scheduleRunUpdate() {
  if (app.state !== STATE.RUNNING) return;
  clearTimeout(app.rpmDebounceTimer);
  app.rpmDebounceTimer = setTimeout(async () => {
    try {
      await app.drive.startRun(app.currentRpm, app.currentDirection, {
        accelMs: app.settings.easeInMs,
        decelMs: app.settings.easeOutMs,
      });
    } catch (err) {
      handleError(err, 'update-run');
    }
  }, 200);
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

  ui.btnStart.disabled = app.state !== STATE.CONNECTED_IDLE || app.linkLost;
  ui.btnStop.disabled = app.state !== STATE.RUNNING || app.linkLost;
  ui.btnEstop.disabled = !connected;

  const directionDisabled = app.state === STATE.RUNNING || app.state === STATE.STOPPING;
  for (const radio of ui.directionRadios) radio.disabled = directionDisabled;
  el('direction-hint').hidden = !directionDisabled;

  ui.faultPanel.hidden = app.state !== STATE.FAULTED;
  ui.estopPanel.hidden = app.state !== STATE.ESTOP_LATCHED;

  const rawToolsEnabled = connected;
  ui.btnRawRead.disabled = !rawToolsEnabled;
  ui.btnRawWrite.disabled = !rawToolsEnabled;
  ui.btnDangerSlaveId.disabled = !rawToolsEnabled;
  ui.btnDangerBaudRate.disabled = !rawToolsEnabled;
  ui.btnDangerDataFormat.disabled = !rawToolsEnabled;

  renderStatusLight();

  if (app.state !== STATE.RUNNING) {
    ui.statusCommanded.textContent = '—';
  } else if (!ui.statusCommanded.textContent.startsWith('Running (reconnected')) {
    ui.statusCommanded.textContent = `${app.currentRpm} rpm, ${app.currentDirection}`;
  }
}

function renderStatusLight() {
  ui.statusLight.classList.remove('status-idle', 'status-running', 'status-fault');
  switch (app.state) {
    case STATE.RUNNING:
      ui.statusLight.classList.add('status-running');
      ui.statusText.textContent = 'Running';
      break;
    case STATE.STOPPING:
      ui.statusLight.classList.add('status-running');
      ui.statusText.textContent = 'Stopping…';
      break;
    case STATE.FAULTED:
      ui.statusLight.classList.add('status-fault');
      ui.statusText.textContent = 'Fault';
      break;
    case STATE.ESTOP_LATCHED:
      ui.statusLight.classList.add('status-fault');
      ui.statusText.textContent = 'E-STOP';
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
  // Simple, non-blocking: reuse the fault panel area as a general message surface without
  // pretending it's a fault when it's just informational.
  console.warn(message);
  ui.connectionState.textContent = message;
  setTimeout(() => renderState(), 4000);
}

const ERROR_MESSAGES = {
  ModbusExceptionError: (err) => {
    switch (err.exceptionCode) {
      case 0x01: return 'The driver rejected that command (unsupported function). Please report this as a bug.';
      case 0x02: return "The driver said that setting doesn't exist. Check the Slave ID in Settings.";
      case 0x03: return 'The driver rejected that value as out of range for this motor. Try a smaller RPM or ramp value.';
      case 0x08: return 'Communication error (bad CRC reported by the driver). Check baud rate/data format in Settings against the DIP switches.';
      default: return `Driver reported an error (code 0x${err.exceptionCode.toString(16)}).`;
    }
  },
  ModbusTimeoutError: () => 'No response from the driver. Check: correct serial port, Slave ID, baud/format match the DIP switches, and RS-485 wiring.',
  ModbusCrcError: () => 'Communication error (corrupted data). Usually means the baud rate/data format in Settings doesn’t match the drive’s DIP switches, or a wiring issue.',
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
  ui.btnReconnect.addEventListener('click', reconnect);

  ui.rpmSlider.addEventListener('input', () => {
    app.currentRpm = clamp(Number(ui.rpmSlider.value), app.settings.rpmMin, app.settings.rpmMax);
    ui.rpmNumber.value = app.currentRpm;
    scheduleRunUpdate();
  });
  ui.rpmNumber.addEventListener('change', () => {
    app.currentRpm = clamp(Number(ui.rpmNumber.value), app.settings.rpmMin, app.settings.rpmMax);
    ui.rpmSlider.value = app.currentRpm;
    ui.rpmNumber.value = app.currentRpm;
    scheduleRunUpdate();
  });

  for (const radio of ui.directionRadios) {
    radio.addEventListener('change', () => {
      app.currentDirection = getSelectedDirection();
    });
  }

  ui.easeInSlider.addEventListener('input', () => {
    app.settings.easeInMs = Number(ui.easeInSlider.value);
    ui.easeInNumber.value = app.settings.easeInMs;
    scheduleRunUpdate();
  });
  ui.easeInNumber.addEventListener('change', () => {
    app.settings.easeInMs = Number(ui.easeInNumber.value);
    ui.easeInSlider.value = app.settings.easeInMs;
    scheduleRunUpdate();
  });
  ui.easeOutSlider.addEventListener('input', () => {
    app.settings.easeOutMs = Number(ui.easeOutSlider.value);
    ui.easeOutNumber.value = app.settings.easeOutMs;
    scheduleRunUpdate();
  });
  ui.easeOutNumber.addEventListener('change', () => {
    app.settings.easeOutMs = Number(ui.easeOutNumber.value);
    ui.easeOutSlider.value = app.settings.easeOutMs;
    scheduleRunUpdate();
  });

  ui.btnStart.addEventListener('click', handleStart);
  ui.btnStop.addEventListener('click', handleStop);
  ui.btnEstop.addEventListener('click', handleEstop);
  ui.btnClearFault.addEventListener('click', handleResume);
  ui.btnResume.addEventListener('click', handleResume);

  document.addEventListener('keydown', (event) => {
    if (event.code === 'Space' && app.state !== STATE.DISCONNECTED) {
      event.preventDefault();
      handleEstop();
    }
  });

  ui.btnSaveSettings.addEventListener('click', () => {
    app.settings.slaveId = Number(ui.settingSlaveId.value);
    app.settings.baudRate = Number(ui.settingBaudRate.value);
    app.settings.dataFormat = ui.settingDataFormat.value;
    app.settings.rpmMin = Number(ui.settingRpmMin.value);
    app.settings.rpmMax = Number(ui.settingRpmMax.value);
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
    showTransientMessage('Settings saved. Reconnect for serial-port changes to take effect.');
  });

  ui.btnResetSettings.addEventListener('click', () => {
    app.settings = resetSettings();
    applySettingsToForm();
    applySettingsToControlLimits();
  });

  ui.btnDangerSlaveId.addEventListener('click', () => runDangerAction(
    `Write slave ID ${ui.dangerSlaveId.value} to the drive and save to EEPROM?`,
    async () => {
      const id = Number(ui.dangerSlaveId.value);
      await app.drive.setSlaveIdOnWire(id);
      await app.drive.saveToEeprom();
    },
  ));

  ui.btnDangerBaudRate.addEventListener('click', () => runDangerAction(
    `Write baud rate ${ui.dangerBaudRate.value} to the drive and save to EEPROM? The drive will stop responding at the old baud rate immediately.`,
    async () => {
      const code = BAUD_RATE_CODES[Number(ui.dangerBaudRate.value)];
      await app.drive.setBaudRateOnWire(code);
      await app.drive.saveToEeprom();
    },
  ));

  ui.btnDangerDataFormat.addEventListener('click', () => runDangerAction(
    `Write data format ${ui.dangerDataFormat.value} to the drive and save to EEPROM?`,
    async () => {
      const code = DATA_FORMAT_OPTIONS[ui.dangerDataFormat.value].code;
      await app.drive.setDataFormatOnWire(code);
      await app.drive.saveToEeprom();
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
    window.alert('Write complete. Update Settings above to match, then disconnect and reconnect.');
  } catch (err) {
    handleError(err, 'danger-zone');
  } finally {
    if (app.state !== STATE.DISCONNECTED) startPolling();
  }
}

main();
