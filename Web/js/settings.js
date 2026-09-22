// localStorage-backed settings store. One versioned JSON blob so future schema changes are a
// migration function, not a scattering of individually-versioned keys.

const STORAGE_KEY = 'dm556rs-tools.settings.v1';

// Pr5.24 data-format code -> Web Serial port options.
const DATA_FORMAT_OPTIONS = {
  '8E2': { code: 0, parity: 'even', stopBits: 2, label: '8E2 (8 data bits, even parity, 2 stop bits)' },
  '8O2': { code: 1, parity: 'odd', stopBits: 2, label: '8O2 (8 data bits, odd parity, 2 stop bits)' },
  '8E1': { code: 2, parity: 'even', stopBits: 1, label: '8E1 (8 data bits, even parity, 1 stop bit)' },
  '8O1': { code: 3, parity: 'odd', stopBits: 1, label: '8O1 (8 data bits, odd parity, 1 stop bit)' },
  '8N1': { code: 4, parity: 'none', stopBits: 1, label: '8N1 (8 data bits, no parity, 1 stop bit) - factory default' },
  '8N2': { code: 5, parity: 'none', stopBits: 2, label: '8N2 (8 data bits, no parity, 2 stop bits)' },
};

const BAUD_RATE_OPTIONS = [2400, 4800, 9600, 19200, 38400, 57600, 115200];
// Pr5.22 code for each baud rate above, same index order.
const BAUD_RATE_CODES = { 2400: 0, 4800: 1, 9600: 2, 19200: 3, 38400: 4, 57600: 5, 115200: 6 };

const DEFAULT_SETTINGS = {
  slaveId: 1,
  baudRate: 38400,
  dataFormat: '8N1',
  rpmMin: 1,
  rpmMax: 300,
  easeInMs: 150,
  easeOutMs: 150,
  pollIntervalMs: 250,
  requestTimeoutMs: 400,
  interWriteDelayMs: 5,
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
  DATA_FORMAT_OPTIONS,
  BAUD_RATE_OPTIONS,
  BAUD_RATE_CODES,
  loadSettings,
  saveSettings,
  resetSettings,
};
