// Web Serial transport + a newline-delimited line protocol, for talking to the iCL_Motor_Test
// Teensy sketch directly over USB serial (NOT Modbus - this firmware is our own, driving the
// StepperOnline iCL57/60 drive's PUL/DIR inputs itself, and speaks a plain-text line protocol).
// No dependencies. Targets Chrome/Edge (navigator.serial).
//
// SerialTransport here is the same generic Web Serial wrapper used by the other tools in this
// repo (forked, not shared - see iCL/README.md "Design notes" for why). LineSerial replaces
// those tools' Modbus framing/queue with something matching this firmware's actual protocol:
// newline-terminated ASCII commands, with no strict request/response framing - the sketch just
// prints whatever "# ..." lines a command produces, and separately pushes "S,..." telemetry
// lines at a fixed ~20Hz when streaming is enabled (see the sketch's printStreamLine()).

// Wraps a single navigator.serial SerialPort: open/close, write, and a byte-stream read loop
// that hands raw chunks to a listener. Knows nothing about the line protocol.
class SerialTransport {
  constructor() {
    this._port = null;
    this._reader = null;
    this._readLoopPromise = null;
    this._dataListeners = new Set();
    this._disconnectListeners = new Set();
    this._boundNativeDisconnect = this._onNativeDisconnect.bind(this);
  }

  static isSupported() {
    return 'serial' in navigator;
  }

  get isOpen() {
    return this._port !== null;
  }

  onData(callback) { this._dataListeners.add(callback); return () => this._dataListeners.delete(callback); }
  onDisconnect(callback) { this._disconnectListeners.add(callback); return () => this._disconnectListeners.delete(callback); }

  // Must be called synchronously from within a user-gesture handler (e.g. a button click).
  async requestAndOpen(portOptions) {
    const port = await navigator.serial.requestPort();
    await this._openPort(port, portOptions);
  }

  // Reuses a previously-granted port (Chrome remembers the grant per origin+device) without
  // requiring a fresh user gesture. Returns false if no previously-granted port is available.
  async openKnownPort(portOptions) {
    const ports = await navigator.serial.getPorts();
    if (ports.length === 0) return false;
    await this._openPort(ports[0], portOptions);
    return true;
  }

  // Revokes all previously-granted permissions for this origin so the next connect attempt is
  // forced to show the native port picker again, letting the user pick a different device.
  static async forgetAllKnownPorts() {
    const ports = await navigator.serial.getPorts();
    for (const port of ports) {
      if (typeof port.forget === 'function') {
        await port.forget().catch(() => {});
      }
    }
  }

  async _openPort(port, portOptions) {
    await port.open({
      baudRate: portOptions.baudRate,
      dataBits: 8,
      parity: portOptions.parity,
      stopBits: portOptions.stopBits,
      bufferSize: 4096,
    });
    this._port = port;
    navigator.serial.addEventListener('disconnect', this._boundNativeDisconnect);
    this._startReadLoop();
  }

  _onNativeDisconnect(event) {
    if (event.target !== this._port) return;
    this._teardown();
    for (const cb of this._disconnectListeners) cb({ native: true });
  }

  async _startReadLoop() {
    this._readLoopPromise = (async () => {
      try {
        while (this._port && this._port.readable) {
          this._reader = this._port.readable.getReader();
          try {
            while (true) {
              const { value, done } = await this._reader.read();
              if (done) break;
              if (value && value.length > 0) {
                for (const cb of this._dataListeners) cb(value);
              }
            }
          } finally {
            this._reader.releaseLock();
            this._reader = null;
          }
        }
      } catch (err) {
        // Port likely closed/disconnected out from under the read loop; native 'disconnect'
        // handler (if this was a real unplug) will have already fired or will fire shortly.
      }
    })();
  }

  async write(bytes) {
    if (!this._port || !this._port.writable) {
      throw new Error('Serial port is not open');
    }
    const writer = this._port.writable.getWriter();
    try {
      await writer.write(bytes);
    } finally {
      writer.releaseLock();
    }
  }

  async close() {
    navigator.serial.removeEventListener('disconnect', this._boundNativeDisconnect);
    await this._teardown();
  }

  async _teardown() {
    const port = this._port;
    this._port = null;
    try {
      if (this._reader) {
        await this._reader.cancel().catch(() => {});
      }
      if (this._readLoopPromise) {
        await this._readLoopPromise.catch(() => {});
      }
      if (port) {
        await port.close().catch(() => {});
      }
    } catch (err) {
      // best-effort teardown
    }
  }
}

const textEncoder = new TextEncoder();
const textDecoder = new TextDecoder();

// Splits the incoming byte stream into '\n'-terminated lines (stripping a trailing '\r') and
// provides a best-effort "send and collect whatever comes back" helper, since this firmware has
// no strict request/response framing - unlike the other tools' Modbus transports, there's no
// frame length or CRC to key off of.
class LineSerial {
  constructor(transport) {
    this._transport = transport;
    this._buffer = '';
    this._lineListeners = new Set();
    this._unsubscribe = transport.onData(this._onData.bind(this));
  }

  dispose() {
    this._unsubscribe();
  }

  onLine(callback) { this._lineListeners.add(callback); return () => this._lineListeners.delete(callback); }

  _onData(chunk) {
    this._buffer += textDecoder.decode(chunk, { stream: true });
    let idx;
    while ((idx = this._buffer.indexOf('\n')) >= 0) {
      let line = this._buffer.slice(0, idx);
      this._buffer = this._buffer.slice(idx + 1);
      if (line.endsWith('\r')) line = line.slice(0, -1);
      for (const cb of this._lineListeners) cb(line);
    }
  }

  async send(command) {
    await this._transport.write(textEncoder.encode(command + '\n'));
  }

  // Sends a command, then collects every line seen until the link has been quiet for `quietMs`
  // (reset by ANY incoming line, including unrelated stream traffic) or `timeoutMs` elapses
  // overall, whichever comes first. This is a best-effort correlation, not a guarantee the
  // collected lines are only this command's response - fine for this firmware's actual usage
  // (short, bursty "# ..." replies), not appropriate for anything safety-critical.
  async sendAndCollect(command, { quietMs = 150, timeoutMs = 800 } = {}) {
    const lines = [];
    const unsubscribeCollect = this.onLine((line) => lines.push(line));
    try {
      await this.send(command);
      await new Promise((resolve) => {
        let quietTimer;
        const overallTimer = setTimeout(done, timeoutMs);
        const unsubscribeReset = this.onLine(bump);
        function bump() {
          clearTimeout(quietTimer);
          quietTimer = setTimeout(done, quietMs);
        }
        function done() {
          clearTimeout(quietTimer);
          clearTimeout(overallTimer);
          unsubscribeReset();
          resolve();
        }
        bump();
      });
    } finally {
      unsubscribeCollect();
    }
    return lines;
  }
}

export { SerialTransport, LineSerial };
