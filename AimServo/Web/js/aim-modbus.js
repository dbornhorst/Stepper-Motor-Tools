// Web Serial transport + Modbus RTU (FC03/06/10) framing, CRC, and a single-flight request queue.
// No dependencies. Targets Chrome/Edge (navigator.serial).
//
// This is a fork of DM556RS/Web/js/modbus-serial.js, not a shared module - see AimServo/README.md
// "Design notes" for why. If you fix a generic bug here (e.g. in SerialTransport), port the same
// fix to the DM556RS copy.
//
// Extended beyond the DM556RS original: the YZ-AIM servo drive uses two vendor-specific function
// codes beyond standard Modbus - 0x78 (relative move) and 0x7B (absolute move, unreliable on
// bench per the Python reference this was ported from - not used by aim-servo.js's own moveAbsolute()
// method, which is software-only). Both have the same fixed-8-byte reply shape as FC06/FC10, so
// _tryResolvePending() recognizes them the same way. Word-swap decoding of their ack payload is
// drive-specific interpretation and intentionally lives in aim-servo.js, not here.

const CRC_TABLE = (() => {
  // Standard Modbus CRC-16 (poly 0xA001, init 0xFFFF), precomputed per-byte table.
  const table = new Uint16Array(256);
  for (let byte = 0; byte < 256; byte++) {
    let crc = byte;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc & 0x0001) ? ((crc >>> 1) ^ 0xA001) : (crc >>> 1);
    }
    table[byte] = crc;
  }
  return table;
})();

function modbusCrc16(bytes) {
  let crc = 0xFFFF;
  for (let i = 0; i < bytes.length; i++) {
    const idx = (crc ^ bytes[i]) & 0xFF;
    crc = (crc >>> 8) ^ CRC_TABLE[idx];
  }
  return crc; // low byte = crc & 0xFF, high byte = (crc >> 8) & 0xFF
}

function appendCrc(bytes) {
  // Verified equivalent to the Python reference's struct.pack("<H", crc) - little-endian 16-bit
  // means low byte first, which is exactly what's appended here; same convention, not an assumption.
  const crc = modbusCrc16(bytes);
  const out = new Uint8Array(bytes.length + 2);
  out.set(bytes, 0);
  out[bytes.length] = crc & 0xFF;        // low byte first on the wire
  out[bytes.length + 1] = (crc >> 8) & 0xFF;
  return out;
}

function crcOk(frame) {
  const body = frame.subarray(0, frame.length - 2);
  const crc = modbusCrc16(body);
  return (crc & 0xFF) === frame[frame.length - 2] &&
         ((crc >> 8) & 0xFF) === frame[frame.length - 1];
}

class ModbusTimeoutError extends Error {
  constructor(message) { super(message); this.name = 'ModbusTimeoutError'; }
}
class ModbusCrcError extends Error {
  constructor(message) { super(message); this.name = 'ModbusCrcError'; }
}
class ModbusFramingError extends Error {
  constructor(message) { super(message); this.name = 'ModbusFramingError'; }
}
class ModbusExceptionError extends Error {
  constructor(functionCode, exceptionCode) {
    super(`Modbus exception 0x${exceptionCode.toString(16).padStart(2, '0')} on function 0x${functionCode.toString(16).padStart(2, '0')}`);
    this.name = 'ModbusExceptionError';
    this.functionCode = functionCode;
    this.exceptionCode = exceptionCode;
  }
}

// Wraps a single navigator.serial SerialPort: open/close, write, and a byte-stream read loop
// that hands raw chunks to a listener. Knows nothing about Modbus framing.
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
  // forced to show the native port picker again, letting the user pick a different device
  // instead of silently reusing whatever was picked before (openKnownPort()'s behavior).
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

// Modbus RTU framing + a FIFO request queue on top of a SerialTransport.
class ModbusRTU {
  constructor(transport, options = {}) {
    this._transport = transport;
    this.requestTimeoutMs = options.requestTimeoutMs ?? 400;
    this.interByteTimeoutMs = options.interByteTimeoutMs ?? 50;
    this._chain = Promise.resolve();
    this._rxBuffer = [];
    this._pendingResolution = null; // { expectedLength, resolve, reject, byteTimer, overallTimer }
    this._unsubscribe = transport.onData(this._onData.bind(this));
  }

  dispose() {
    this._unsubscribe();
  }

  _onData(chunk) {
    for (let i = 0; i < chunk.length; i++) this._rxBuffer.push(chunk[i]);
    this._tryResolvePending();
  }

  _tryResolvePending() {
    const pending = this._pendingResolution;
    if (!pending) {
      // No one is waiting; keep buffer from growing unbounded on stray bytes.
      if (this._rxBuffer.length > 512) this._rxBuffer.length = 0;
      return;
    }

    // Need at least 2 bytes to know function code / exception bit.
    if (this._rxBuffer.length < 2) {
      this._resetByteTimer();
      return;
    }
    const fnByte = this._rxBuffer[1];
    let expectedLength;
    if (fnByte & 0x80) {
      expectedLength = 5; // error frame
    } else if (fnByte === 0x03) {
      if (this._rxBuffer.length < 3) { this._resetByteTimer(); return; }
      expectedLength = 3 + this._rxBuffer[2] + 2;
    } else if (fnByte === 0x06 || fnByte === 0x10 || fnByte === 0x78 || fnByte === 0x7B) {
      // 0x78/0x7B (relative/absolute move, vendor-specific) echo a fixed 8-byte reply, same
      // shape as FC06/FC10: [addr, fc, 4 payload bytes, crc-lo, crc-hi].
      expectedLength = 8;
    } else {
      // Unknown function code echoed back; bail with what we have once the byte timer lapses.
      this._resetByteTimer();
      return;
    }

    if (this._rxBuffer.length >= expectedLength) {
      const frame = Uint8Array.from(this._rxBuffer.slice(0, expectedLength));
      this._rxBuffer.splice(0, expectedLength);
      this._settlePending(frame);
    } else {
      this._resetByteTimer();
    }
  }

  _resetByteTimer() {
    const pending = this._pendingResolution;
    if (!pending) return;
    clearTimeout(pending.byteTimer);
    pending.byteTimer = setTimeout(() => {
      // Inter-byte silence with an incomplete frame in hand: treat as framing failure so a
      // stuck/partial reply doesn't linger and corrupt the next transaction.
      this._rxBuffer.length = 0;
      this._failPending(new ModbusFramingError('Incomplete Modbus frame (inter-byte timeout)'));
    }, pending.interByteTimeoutMs);
  }

  _settlePending(frame) {
    const pending = this._pendingResolution;
    if (!pending) return;
    clearTimeout(pending.byteTimer);
    clearTimeout(pending.overallTimer);
    this._pendingResolution = null;

    if (!crcOk(frame)) {
      pending.reject(new ModbusCrcError('CRC mismatch in response'));
      return;
    }
    if (frame[0] !== pending.slaveId) {
      pending.reject(new ModbusFramingError(`Unexpected slave id in response: ${frame[0]}`));
      return;
    }
    if (frame[1] & 0x80) {
      pending.reject(new ModbusExceptionError(frame[1] & 0x7F, frame[2]));
      return;
    }
    pending.resolve(frame);
  }

  _failPending(err) {
    const pending = this._pendingResolution;
    if (!pending) return;
    clearTimeout(pending.byteTimer);
    clearTimeout(pending.overallTimer);
    this._pendingResolution = null;
    pending.reject(err);
  }

  async _transact(slaveId, requestBytes) {
    if (this._pendingResolution) {
      throw new Error('Internal error: overlapping Modbus transaction');
    }
    this._rxBuffer.length = 0;
    await this._transport.write(requestBytes);

    return new Promise((resolve, reject) => {
      const pending = {
        slaveId,
        resolve,
        reject,
        interByteTimeoutMs: this.interByteTimeoutMs,
        byteTimer: null,
        overallTimer: setTimeout(() => {
          this._pendingResolution = null;
          reject(new ModbusTimeoutError('No response from driver (timeout)'));
        }, this.requestTimeoutMs),
      };
      this._pendingResolution = pending;
    });
  }

  _enqueue(op) {
    const result = this._chain.then(op, op);
    // Swallow rejections in the chain itself so one failed request doesn't wedge the queue;
    // the caller's awaited promise still sees the real rejection.
    this._chain = result.then(() => {}, () => {});
    return result;
  }

  readHoldingRegisters(slaveId, startAddr, quantity) {
    return this._enqueue(async () => {
      const req = appendCrc(new Uint8Array([
        slaveId, 0x03,
        (startAddr >> 8) & 0xFF, startAddr & 0xFF,
        (quantity >> 8) & 0xFF, quantity & 0xFF,
      ]));
      const resp = await this._transact(slaveId, req);
      const byteCount = resp[2];
      const values = [];
      for (let i = 0; i < byteCount / 2; i++) {
        values.push((resp[3 + i * 2] << 8) | resp[3 + i * 2 + 1]);
      }
      return values;
    });
  }

  writeSingleRegister(slaveId, addr, value) {
    return this._enqueue(async () => {
      const req = appendCrc(new Uint8Array([
        slaveId, 0x06,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF,
      ]));
      await this._transact(slaveId, req);
    });
  }

  writeMultipleRegisters(slaveId, startAddr, values) {
    return this._enqueue(async () => {
      const byteCount = values.length * 2;
      const header = [
        slaveId, 0x10,
        (startAddr >> 8) & 0xFF, startAddr & 0xFF,
        (values.length >> 8) & 0xFF, values.length & 0xFF,
        byteCount,
      ];
      const data = [];
      for (const v of values) { data.push((v >> 8) & 0xFF, v & 0xFF); }
      const req = appendCrc(new Uint8Array([...header, ...data]));
      await this._transact(slaveId, req);
    });
  }

  // Vendor-specific move commands (0x78 relative, 0x7B absolute). value32 is a signed 32-bit
  // integer (pulses), sent big-endian. Resolves with the raw 8-byte reply frame - decoding the
  // ack payload (which has its two 16-bit words swapped relative to a normal big-endian read) is
  // drive-specific interpretation done by the caller (aim-servo.js's decodeVendorMoveAck), not here.
  sendVendorMoveCommand(slaveId, functionCode, value32) {
    return this._enqueue(async () => {
      const unsigned = value32 < 0 ? (value32 + 0x100000000) >>> 0 : value32 >>> 0;
      const req = appendCrc(new Uint8Array([
        slaveId, functionCode,
        (unsigned >>> 24) & 0xFF, (unsigned >>> 16) & 0xFF,
        (unsigned >>> 8) & 0xFF, unsigned & 0xFF,
      ]));
      return this._transact(slaveId, req);
    });
  }

  // Used only by the current-cutoff safety trip (and any other "act now" control): discards
  // whatever is queued, forcibly abandons any in-flight transaction (so it can't collide with
  // this one), and jumps this write to the front - mirrors the DM556RS tool's E-Stop pattern, so
  // a safety-critical output-disable is never stuck behind a debounced RPM update or a slow poll.
  jumpQueueWriteSingleRegister(slaveId, addr, value) {
    this._chain = Promise.resolve();
    if (this._pendingResolution) {
      this._rxBuffer.length = 0;
      this._failPending(new ModbusFramingError('Transaction superseded by safety cutoff'));
    }
    return this._enqueue(async () => {
      const req = appendCrc(new Uint8Array([
        slaveId, 0x06,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF,
      ]));
      await this._transact(slaveId, req);
    });
  }
}

export {
  SerialTransport,
  ModbusRTU,
  ModbusTimeoutError,
  ModbusCrcError,
  ModbusFramingError,
  ModbusExceptionError,
  modbusCrc16,
};
