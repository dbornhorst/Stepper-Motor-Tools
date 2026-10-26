import { SerialTransport, LineSerial } from './icl-serial.js';
import { ICLStepperDrive, DIRECTION, MOTION_STATE_LABEL } from './icl-stepper.js';
import { loadSettings, saveSettings, resetSettings } from './settings.js';

const STATE = {
  DISCONNECTED: 'DISCONNECTED',
  CONNECTED_IDLE: 'CONNECTED_IDLE',
  RUNNING: 'RUNNING',
};

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
  statusTarget: el('status-target'),
  statusActual: el('status-actual'),
  statusSteps: el('status-steps'),
  btnZeroSteps: el('btn-zero-steps'),

  rpmSlider: el('rpm-slider'),
  rpmNumber: el('rpm-number'),
  directionRadios: document.getElementsByName('direction'),
  directionHint: el('direction-hint'),
  accelNumber: el('accel-number'),
  decelNumber: el('decel-number'),

  btnStart: el('btn-start'),
  btnStop: el('btn-stop'),
  btnEstop: el('btn-estop'),

  settingRpmMin: el('setting-rpm-min'),
  settingRpmMax: el('setting-rpm-max'),
  settingStepsPerRev: el('setting-steps-per-rev'),
  settingInvert: el('setting-invert'),
  settingLinkLostTimeout: el('setting-link-lost-timeout'),
  settingCommandQuiet: el('setting-command-quiet'),
  settingCommandTimeout: el('setting-command-timeout'),
  btnSaveSettings: el('btn-save-settings'),
  btnResetSettings: el('btn-reset-settings'),

  rawCommand: el('raw-command'),
  btnRawSend: el('btn-raw-send'),
  rawCommandLog: el('raw-command-log'),
};

const app = {
  settings: loadSettings(),
  state: STATE.DISCONNECTED,
  linkLost: false,
  lastStreamAt: 0,
  linkWatchdogTimer: null,
  rpmDebounceTimer: null,

  transport: null,
  lineSerial: null,
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

  applySettingsToForm();
  applySettingsToControlLimits();
  wireEvents();
  renderState();

  window.addEventListener('pagehide', () => {
    if (app.transport) app.transport.close().catch(() => {});
  });
}

function applySettingsToForm() {
  ui.settingRpmMin.value = app.settings.rpmMin;
  ui.settingRpmMax.value = app.settings.rpmMax;
  ui.settingStepsPerRev.value = app.settings.stepsPerRev;
  ui.settingInvert.value = String(app.settings.invert);
  ui.settingLinkLostTimeout.value = app.settings.linkLostTimeoutMs;
  ui.settingCommandQuiet.value = app.settings.commandQuietMs;
  ui.settingCommandTimeout.value = app.settings.commandTimeoutMs;
  ui.accelNumber.value = app.settings.accelRpmPerSec;
  ui.decelNumber.value = app.settings.decelRpmPerSec;
}

function applySettingsToControlLimits() {
  const hardMax = app.settings.hardMaxRpm;
  const rpmMax = clamp(app.settings.rpmMax, 1, hardMax);
  ui.rpmSlider.min = app.settings.rpmMin;
  ui.rpmSlider.max = rpmMax;
  ui.rpmNumber.min = app.settings.rpmMin;
  ui.rpmNumber.max = rpmMax;
  const clamped = clamp(app.currentRpm, app.settings.rpmMin, rpmMax);
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
    app.transport = new SerialTransport();
    app.transport.onDisconnect(onLinkDown);
    const portOptions = { baudRate: 115200, parity: 'none', stopBits: 1 };
    const reused = await app.transport.openKnownPort(portOptions);
    if (!reused) {
      await app.transport.requestAndOpen(portOptions);
    }

    app.lineSerial = new LineSerial(app.transport);
    app.drive = new ICLStepperDrive(app.lineSerial);
    app.lineSerial.onLine((line) => appendRawLog(line));
    app.drive.onStreamUpdate(onStreamUpdate);

    app.state = STATE.CONNECTED_IDLE;
    app.linkLost = false;
    app.lastStreamAt = Date.now(); // seed it so the watchdog doesn't fire before the first line
    await app.drive.enableStream(true);

    // Best-effort sync of the device's actual current settings (it has no persistent storage,
    // so these reset to the sketch's compiled-in defaults every power-up and may not match
    // whatever was last saved in this browser).
    try {
      const status = await app.drive.requestStatus();
      if (status.accelRpmPerSec !== undefined) app.settings.accelRpmPerSec = status.accelRpmPerSec;
      if (status.decelRpmPerSec !== undefined) app.settings.decelRpmPerSec = status.decelRpmPerSec;
      if (status.maxRpm !== undefined) app.settings.rpmMax = status.maxRpm;
      if (status.stepsPerRev !== undefined) app.settings.stepsPerRev = status.stepsPerRev;
      if (status.invert !== undefined) app.settings.invert = status.invert;
      applySettingsToForm();
      applySettingsToControlLimits();
      if (status.targetRpm !== undefined) {
        app.currentDirection = status.targetRpm < 0 ? DIRECTION.CCW : DIRECTION.CW;
        app.currentRpm = clamp(Math.abs(status.targetRpm), app.settings.rpmMin, app.settings.rpmMax);
        for (const radio of ui.directionRadios) radio.checked = radio.value === app.currentDirection;
        ui.rpmSlider.value = app.currentRpm;
        ui.rpmNumber.value = app.currentRpm;
      }
      if (status.state && status.state !== 'idle') app.state = STATE.RUNNING;
    } catch (err) {
      // Non-fatal - just means the Settings panel shows this browser's last-known values
      // instead of the device's actual current ones until the next command updates them.
    }

    startLinkWatchdog();
    renderState();
  } catch (err) {
    handleError(err, 'connect');
    await teardownConnection();
  } finally {
    ui.btnConnect.disabled = false;
  }
}

async function disconnect() {
  stopLinkWatchdog();
  await teardownConnection();
  app.state = STATE.DISCONNECTED;
  app.linkLost = false;
  renderState();
}

async function teardownConnection() {
  if (app.lineSerial) { app.lineSerial.dispose(); app.lineSerial = null; }
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
    stopLinkWatchdog();
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
// Telemetry (push-based - the firmware streams "S,..." lines at ~20Hz once enabled, so there's
// no polling loop here, just a reaction to each line plus an activity watchdog for link-loss).
// ---------------------------------------------------------------------------

function onStreamUpdate(update) {
  app.lastStreamAt = Date.now();
  if (app.linkLost) app.linkLost = false;

  ui.statusTarget.textContent = `${update.targetRpm.toFixed(1)} rpm`;
  ui.statusActual.textContent = `${update.currentRpm.toFixed(1)} rpm`;
  ui.statusSteps.textContent = String(update.steps);

  if (app.state !== STATE.DISCONNECTED) {
    app.state = update.state === 'idle' ? STATE.CONNECTED_IDLE : STATE.RUNNING;
  }
  ui.statusText.textContent = MOTION_STATE_LABEL[update.state] ?? update.state;
  renderStatusLight();
  renderControlAvailability();
}

function startLinkWatchdog() {
  stopLinkWatchdog();
  app.linkWatchdogTimer = setInterval(() => {
    if (!app.drive) return;
    const silentFor = Date.now() - app.lastStreamAt;
    const wasLost = app.linkLost;
    app.linkLost = silentFor > app.settings.linkLostTimeoutMs;
    if (app.linkLost !== wasLost) renderState();
  }, 300);
}

function stopLinkWatchdog() {
  if (app.linkWatchdogTimer) { clearInterval(app.linkWatchdogTimer); app.linkWatchdogTimer = null; }
}

function appendRawLog(line) {
  ui.rawCommandLog.textContent += line + '\n';
  const maxChars = 4000;
  if (ui.rawCommandLog.textContent.length > maxChars) {
    ui.rawCommandLog.textContent = ui.rawCommandLog.textContent.slice(-maxChars);
  }
  ui.rawCommandLog.scrollTop = ui.rawCommandLog.scrollHeight;
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
  const rpm = clamp(Number(ui.rpmNumber.value), app.settings.rpmMin, app.settings.rpmMax);
  const direction = getSelectedDirection();
  try {
    await app.drive.setTargetRpm(rpm, direction);
    app.currentRpm = rpm;
    app.currentDirection = direction;
    app.state = STATE.RUNNING; // optimistic; the next stream line confirms/corrects it
  } catch (err) {
    handleError(err, 'start');
  } finally {
    renderState();
  }
}

async function handleStop() {
  if (app.state !== STATE.RUNNING) return;
  try {
    await app.drive.stop();
  } catch (err) {
    handleError(err, 'stop');
  }
}

async function handleEstop() {
  if (app.state === STATE.DISCONNECTED) return;
  try {
    await app.drive.estop();
  } catch (err) {
    handleError(err, 'estop');
  }
}

function scheduleRunUpdate() {
  if (app.state !== STATE.RUNNING) return;
  clearTimeout(app.rpmDebounceTimer);
  app.rpmDebounceTimer = setTimeout(async () => {
    try {
      await app.drive.setTargetRpm(app.currentRpm, app.currentDirection);
    } catch (err) {
      handleError(err, 'update-run');
    }
  }, 200);
}

async function handleAccelChange() {
  if (!app.drive) return;
  const value = Number(ui.accelNumber.value);
  try {
    await app.drive.setAccel(value);
    app.settings.accelRpmPerSec = value;
    saveSettings(app.settings);
  } catch (err) {
    handleError(err, 'set-accel');
  }
}

async function handleDecelChange() {
  if (!app.drive) return;
  const value = Number(ui.decelNumber.value);
  try {
    await app.drive.setDecel(value);
    app.settings.decelRpmPerSec = value;
    saveSettings(app.settings);
  } catch (err) {
    handleError(err, 'set-decel');
  }
}

async function handleZeroSteps() {
  if (!app.drive) return;
  try {
    await app.drive.zero();
  } catch (err) {
    handleError(err, 'zero');
  }
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

  renderControlAvailability();
  renderStatusLight();

  ui.btnRawSend.disabled = !connected;
}

function renderControlAvailability() {
  const idleAndReady = app.state === STATE.CONNECTED_IDLE && !app.linkLost;
  const connected = app.state !== STATE.DISCONNECTED;

  ui.btnStart.disabled = !idleAndReady;
  ui.btnStop.disabled = app.state !== STATE.RUNNING || app.linkLost;
  ui.btnEstop.disabled = !connected;
  ui.btnZeroSteps.disabled = !connected;

  const directionDisabled = app.state === STATE.RUNNING;
  for (const radio of ui.directionRadios) radio.disabled = directionDisabled;
  ui.directionHint.hidden = !directionDisabled;
}

function renderStatusLight() {
  ui.statusLight.classList.remove('status-idle', 'status-running', 'status-fault');
  if (app.state === STATE.DISCONNECTED) {
    ui.statusLight.classList.add('status-idle');
    ui.statusText.textContent = 'Disconnected';
  } else if (app.state === STATE.RUNNING) {
    ui.statusLight.classList.add('status-running');
  } else {
    ui.statusLight.classList.add('status-idle');
  }
}

function showTransientMessage(message) {
  console.warn(message);
  ui.connectionState.textContent = message;
  setTimeout(() => renderState(), 4000);
}

function handleError(err, context) {
  console.error(`[${context}]`, err);
  showTransientMessage((err && err.message) || 'Unknown error.');
}

// ---------------------------------------------------------------------------
// Event wiring
// ---------------------------------------------------------------------------

function wireEvents() {
  ui.btnConnect.addEventListener('click', connect);
  ui.btnDisconnect.addEventListener('click', disconnect);
  ui.btnChoosePort.addEventListener('click', chooseDifferentPort);
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

  ui.accelNumber.addEventListener('change', handleAccelChange);
  ui.decelNumber.addEventListener('change', handleDecelChange);

  ui.btnStart.addEventListener('click', handleStart);
  ui.btnStop.addEventListener('click', handleStop);
  ui.btnEstop.addEventListener('click', handleEstop);
  ui.btnZeroSteps.addEventListener('click', handleZeroSteps);

  document.addEventListener('keydown', (event) => {
    if (event.code === 'Space' && app.state !== STATE.DISCONNECTED) {
      event.preventDefault();
      handleEstop();
    }
  });

  ui.btnSaveSettings.addEventListener('click', async () => {
    const rpmMin = Number(ui.settingRpmMin.value);
    const rpmMax = Number(ui.settingRpmMax.value);
    if (rpmMin >= rpmMax) {
      showTransientMessage('RPM min must be less than RPM max - settings not saved.');
      return;
    }
    if (rpmMax > app.settings.hardMaxRpm) {
      showTransientMessage(`RPM max cannot exceed ${app.settings.hardMaxRpm} - settings not saved.`);
      return;
    }
    app.settings.rpmMin = rpmMin;
    app.settings.rpmMax = rpmMax;
    const newStepsPerRev = Number(ui.settingStepsPerRev.value);
    const newInvert = ui.settingInvert.value === 'true';
    const sprOrInvertChanged = newStepsPerRev !== app.settings.stepsPerRev || newInvert !== app.settings.invert;
    app.settings.stepsPerRev = newStepsPerRev;
    app.settings.invert = newInvert;
    app.settings.linkLostTimeoutMs = Number(ui.settingLinkLostTimeout.value);
    app.settings.commandQuietMs = Number(ui.settingCommandQuiet.value);
    app.settings.commandTimeoutMs = Number(ui.settingCommandTimeout.value);

    saveSettings(app.settings);
    applySettingsToControlLimits();

    if (app.drive && app.state === STATE.CONNECTED_IDLE) {
      try {
        await app.drive.setMaxRpm(app.settings.rpmMax);
        if (sprOrInvertChanged) {
          await app.drive.setStepsPerRev(app.settings.stepsPerRev);
          await app.drive.setInvert(app.settings.invert);
        }
        showTransientMessage('Settings saved and sent to the device.');
      } catch (err) {
        handleError(err, 'save-settings');
      }
    } else if (app.drive && sprOrInvertChanged) {
      showTransientMessage('Settings saved locally - stop the motor to push steps/rev and invert to the device.');
    } else {
      showTransientMessage('Settings saved.');
    }
  });

  ui.btnResetSettings.addEventListener('click', () => {
    app.settings = resetSettings();
    applySettingsToForm();
    applySettingsToControlLimits();
  });

  ui.btnRawSend.addEventListener('click', async () => {
    const command = ui.rawCommand.value.trim();
    if (!command || !app.lineSerial) return;
    try {
      await app.lineSerial.send(command);
    } catch (err) {
      appendRawLog(`error: ${err.message}`);
    }
  });
  ui.rawCommand.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') ui.btnRawSend.click();
  });
}

main();
