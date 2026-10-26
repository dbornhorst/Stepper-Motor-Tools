// localStorage-backed settings store. One versioned JSON blob so future schema changes are a
// migration function, not a scattering of individually-versioned keys.

const STORAGE_KEY = 'icl-stepper-tools.settings.v1';

const DEFAULT_SETTINGS = {
  // Matches the sketch's own compiled-in defaults (kDefaultMaxRpm, kDefaultAccelRpmPerSec,
  // kDefaultDecelRpmPerSec, kDefaultStepsPerRev) - the sketch resets to these on every power-up,
  // since it has no persistent storage, so these are what you'll see on a fresh connection.
  rpmMin: 0,
  rpmMax: 600,
  hardMaxRpm: 3000, // mirrors the sketch's kHardMaxRpm - UI should never let rpmMax exceed this
  accelRpmPerSec: 300,
  decelRpmPerSec: 300,
  stepsPerRev: 400,
  invert: false,
  // Link-lost detection here is activity-based, not poll-based (this firmware pushes telemetry
  // rather than being polled) - if no "S,..." line arrives for this long, flag the link as lost.
  linkLostTimeoutMs: 1000,
  commandQuietMs: 150,
  commandTimeoutMs: 800,
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

export { DEFAULT_SETTINGS, loadSettings, saveSettings, resetSettings };
