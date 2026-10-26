// localStorage-backed settings store. One versioned JSON blob so future schema changes are a
// migration function, not a scattering of individually-versioned keys.

const STORAGE_KEY = 'aim-servo-tools.settings.v1';

// Simple convenience presets from the vendor's model->max-speed table (YZ-AIM_Manual.pdf /
// the Python reference's max_rpm_by_model). Only pre-fills rpmMax when picked - rpmMax itself
// stays directly editable regardless.
const MODEL_PRESETS = {
  '42AIM15': 1500,
  '42AIM10': 1500,
  '57AIM15': 1500,
  '57AIM15H': 3000,
  '57AIM30': 1500,
  '57AIM30H': 3000,
  '60AIM25': 1500,
  '60AIM25H': 3000,
  'Custom': null,
};

const DEFAULT_SETTINGS = {
  slaveId: 1,
  model: '42AIM15',
  rpmMin: 0,
  rpmMax: 1500,
  accel: 5000,
  // Must be tuned per real hardware during commissioning - this sits between the two different
  // defaults used by the Python reference (0.8A in rock_servo.py, 1.0A in aim_shell.py).
  currentCutoffA: 0.9,
  // Confirmed on real hardware: signed target speed correctly reverses direction in speed mode.
  // 'dir-polarity' (register 0x09) was tried first based on the manual's documented unsigned
  // target-speed range, but confirmed NOT to affect Modbus speed-mode direction - the manual's
  // wording for that register turned out to describe the *external* hardware DIR pin only. Kept
  // implemented as a fallback (see aim-servo.js) in case a different unit behaves differently.
  directionMethod: 'signed',
  positiveDirPolarity: 0,
  pulsesPerTurn: 32768,
  pollIntervalMs: 250,
  requestTimeoutMs: 400,
  interWriteDelayMs: 20,
  linkLostThreshold: 3,
};

function loadSettings() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return { ...DEFAULT_SETTINGS };
    const parsed = JSON.parse(raw);
    return { ...DEFAULT_SETTINGS, ...parsed };
  } catch (err) {
    console.warn('Failed to load settings, using defaults', err);
    return { ...DEFAULT_SETTINGS };
  }
}

function saveSettings(settings) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
  } catch (err) {
    console.warn('Failed to save settings', err);
  }
}

function resetSettings() {
  try {
    localStorage.removeItem(STORAGE_KEY);
  } catch (err) {
    // ignore
  }
  return { ...DEFAULT_SETTINGS };
}

export {
  DEFAULT_SETTINGS,
  MODEL_PRESETS,
  loadSettings,
  saveSettings,
  resetSettings,
};
