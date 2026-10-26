/*
 * DM556RS RS-485 Modbus RTU Controller for Teensy
 * 
 * Hardware:
 *   - Teensy (any model with Serial1 on pins 0/1)
 *   - RS-485 transceiver connected to Serial1 (pins 0=TX, 1=RX)
 *   - DE/RE pin of transceiver connected to pin defined by DE_RE_PIN
 * 
 * DM556RS Default settings (check your DIP switches):
 *   - Baud rate: 38400 (SW6=OFF, SW7=ON) — "factory"
 *               OR 115200 (SW6=ON, SW7=ON) — "default"
 *   - Slave ID: 1 (SW1-SW5 all ON)
 *   - Data: 8-bit, no parity, 1 stop bit (Pr5.24 = 4)
 * 
 * Usage: Open Serial Monitor at 115200 baud and type commands:
 *   help         - show all commands
 *   enable       - software-enable the drive (Pr0.07=1)
 *   disable      - software-disable the drive
 *   jog+         - JOG clockwise (holds if sent repeatedly < 50ms)
 *   jog-         - JOG counter-clockwise
 *   stop         - emergency stop
 *   abs <pos> <rpm> <acc> <dec>  - absolute position move (PR0)
 *   rel <pos> <rpm> <acc> <dec>  - relative position move (PR0)
 *   vel <rpm>    - velocity mode (PR0)
 *   status       - read motion status register
 *   current <dA> - set peak current (e.g. "current 32" = 3.2A)
 *   read <hex_addr>              - read a register (e.g. "read 0191")
 *   write <hex_addr> <hex_val>   - write a register
 *   save         - save all parameters to EEPROM
 *   alarm        - read current alarm code
 *   clearalarm   - clear current alarm
 */

// ─── Pin & Port Config ─────────────────────────────────────────────────────
#define RS485_SERIAL   Serial1      // Uses Teensy pins 0 (TX1) and 1 (RX1)
#define DEBUG_SERIAL   Serial       // USB serial for user interaction
#define DE_RE_PIN      4            // Direction control pin for RS-485 transceiver

// ─── Modbus Config ─────────────────────────────────────────────────────────
#define SLAVE_ID       1            // Match your DIP switch setting
#define RS485_BAUD     38400        // Match SW6/SW7: 38400=factory, 115200=default
// Data format: 8N1 — matches Pr5.24=4 factory default
// If you changed parity, use SERIAL_8E1 or SERIAL_8O1 below

// ─── Modbus Register Addresses (from datasheet) ────────────────────────────
#define REG_PULSE_PER_REV     0x0001  // Pr0.00 microstep (pulses/rev)
#define REG_SW_ENABLE         0x000F  // Pr0.07 force enable via RS485 (0=IO, 1=enabled)
#define REG_PEAK_CURRENT      0x0191  // Pr5.00 peak current (unit: 0.1A)
#define REG_JOG_VEL           0x01E1  // Pr6.00 JOG velocity via RS485 (rpm)
#define REG_JOG_ACCDEC        0x01E7  // Pr6.03 JOG acc/dec time (ms)
#define REG_MOTION_STATUS     0x1003  // Motion status (read-only bitfield)
#define REG_CONTROL_WORD      0x1801  // Control word (JOG, save, alarm clear, etc.)
#define REG_CURRENT_ALARM     0x2203  // Current alarm code (read-only)
// PR0 path registers
#define REG_PR0_MODE          0x6200  // PR0 operation mode
#define REG_PR0_POS_H         0x6201  // PR0 position high 16 bits
#define REG_PR0_POS_L         0x6202  // PR0 position low 16 bits
#define REG_PR0_VEL           0x6203  // PR0 velocity (rpm)
#define REG_PR0_ACC           0x6204  // PR0 acceleration (ms/1000rpm)
#define REG_PR0_DEC           0x6205  // PR0 deceleration (ms/1000rpm)
#define REG_PR_TRIGGER        0x6002  // PR trigger register

// Control word values
#define CW_JOG_CW            0x4001
#define CW_JOG_CCW           0x4002
#define CW_RESET_ALARM        0x1111
#define CW_SAVE_EEPROM        0x2211

// PR mode bits
#define PR_MODE_ABS_POS       0x0001  // Absolute position
#define PR_MODE_REL_POS       0x0041  // Relative position (bit6=1)
#define PR_MODE_VELOCITY      0x0002  // Velocity mode

// PR trigger values
#define PR_TRIGGER_PR0        0x0010  // Run PR0
#define PR_ESTOP              0x0040  // Emergency stop

// ─── Timing ────────────────────────────────────────────────────────────────
#define RS485_TX_DELAY_US     1000     // Hold DE high after last byte (us)
#define MODBUS_RESPONSE_TIMEOUT_MS 500

// ─── CRC-16 (Modbus) ───────────────────────────────────────────────────────
uint16_t modbuscrc16(uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else               { crc >>= 1; }
    }
  }
  return crc;
}

// ─── RS-485 Helpers ────────────────────────────────────────────────────────
void rs485_tx_enable()  { digitalWrite(DE_RE_PIN, HIGH); }
void rs485_rx_enable()  { digitalWrite(DE_RE_PIN, LOW);  }

// Send raw bytes over RS-485
void rs485_send(uint8_t *buf, uint8_t len) {
  rs485_tx_enable();
  delayMicroseconds(500);
  RS485_SERIAL.write(buf, len);
  RS485_SERIAL.flush();                // Wait for TX to complete
  delayMicroseconds(RS485_TX_DELAY_US);
  rs485_rx_enable();
}

// Receive up to maxLen bytes within timeout_ms; returns actual byte count
uint8_t rs485_recv(uint8_t *buf, uint8_t maxLen, uint32_t timeout_ms = MODBUS_RESPONSE_TIMEOUT_MS) {
  uint8_t count = 0;
  uint32_t t = millis();
  while ((millis() - t) < timeout_ms && count < maxLen) {
    if (RS485_SERIAL.available()) {
      buf[count++] = RS485_SERIAL.read();
      t = millis();   // Reset timeout on each byte received
    }
  }
  return count;
}

// ─── Modbus RTU Primitives ─────────────────────────────────────────────────

// FC03: Read single holding register. Returns value or -1 on error.
int32_t modbusReadReg(uint8_t slaveId, uint16_t regAddr) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x03;
  req[2] = regAddr >> 8;
  req[3] = regAddr & 0xFF;
  req[4] = 0x00;   // Read 1 register
  req[5] = 0x01;
  uint16_t crc = modbuscrc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (RS485_SERIAL.available()) RS485_SERIAL.read(); // Flush RX
  rs485_send(req, 8);

  uint8_t resp[7];
  uint8_t n = rs485_recv(resp, 7);
  if (n < 7) return -1;

  uint16_t rcrc = modbuscrc16(resp, 5);
  if ((rcrc & 0xFF) != resp[5] || (rcrc >> 8) != resp[6]) return -1;
  if (resp[0] != slaveId || resp[1] != 0x03 || resp[2] != 2)  return -1;

  return (int32_t)((uint16_t)(resp[3] << 8) | resp[4]);
}

// FC06: Write single holding register. Returns true on success.
bool modbusWriteReg(uint8_t slaveId, uint16_t regAddr, uint16_t value) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x06;
  req[2] = regAddr >> 8;
  req[3] = regAddr & 0xFF;
  req[4] = value >> 8;
  req[5] = value & 0xFF;
  uint16_t crc = modbuscrc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (RS485_SERIAL.available()) RS485_SERIAL.read();
  rs485_send(req, 8);

  uint8_t resp[8];
  uint8_t n = rs485_recv(resp, 8);
  if (n < 8) return false;

  uint16_t rcrc = modbuscrc16(resp, 6);
  if ((rcrc & 0xFF) != resp[6] || (rcrc >> 8) != resp[7]) return false;
  // Echo should match request
  return (resp[0] == slaveId && resp[1] == 0x06 &&
          resp[2] == req[2]  && resp[3] == req[3] &&
          resp[4] == req[4]  && resp[5] == req[5]);
}

// FC10: Write multiple consecutive registers.
// data[] is array of uint16_t values, count = number of registers.
bool modbusWriteMultiReg(uint8_t slaveId, uint16_t startAddr, uint16_t *data, uint8_t count) {
  uint8_t byteCount = count * 2;
  uint8_t reqLen = 9 + byteCount;
  uint8_t req[64];
  req[0] = slaveId;
  req[1] = 0x10;
  req[2] = startAddr >> 8;
  req[3] = startAddr & 0xFF;
  req[4] = 0x00;
  req[5] = count;
  req[6] = byteCount;
  for (uint8_t i = 0; i < count; i++) {
    req[7 + i*2] = data[i] >> 8;
    req[8 + i*2] = data[i] & 0xFF;
  }
  uint16_t crc = modbuscrc16(req, 7 + byteCount);
  req[7 + byteCount]     = crc & 0xFF;
  req[7 + byteCount + 1] = crc >> 8;

  while (RS485_SERIAL.available()) RS485_SERIAL.read();
  rs485_send(req, reqLen);

  uint8_t resp[8];
  uint8_t n = rs485_recv(resp, 8);
  if (n < 8) return false;

  uint16_t rcrc = modbuscrc16(resp, 6);
  return ((rcrc & 0xFF) == resp[6] && (rcrc >> 8) == resp[7] &&
          resp[0] == slaveId && resp[1] == 0x10);
}

// ─── Drive Commands ────────────────────────────────────────────────────────

bool driveEnable(bool en) {
  return modbusWriteReg(SLAVE_ID, REG_SW_ENABLE, en ? 1 : 0);
}

bool driveJogCW() {
  return modbusWriteReg(SLAVE_ID, REG_CONTROL_WORD, CW_JOG_CW);
}

bool driveJogCCW() {
  return modbusWriteReg(SLAVE_ID, REG_CONTROL_WORD, CW_JOG_CCW);
}

bool driveStop() {
  return modbusWriteReg(SLAVE_ID, REG_PR_TRIGGER, PR_ESTOP);
}

bool driveSave() {
  return modbusWriteReg(SLAVE_ID, REG_CONTROL_WORD, CW_SAVE_EEPROM);
}

bool driveClearAlarm() {
  return modbusWriteReg(SLAVE_ID, REG_CONTROL_WORD, CW_RESET_ALARM);
}

// Move PR0 in absolute or relative position mode
// pos: steps (positive or negative), vel: rpm, acc/dec: ms/1000rpm
bool driveMovePosition(bool relative, int32_t pos, uint16_t vel, uint16_t acc, uint16_t dec) {
  uint16_t mode = relative ? PR_MODE_REL_POS : PR_MODE_ABS_POS;
  uint16_t posH = (uint16_t)((pos >> 16) & 0xFFFF);
  uint16_t posL = (uint16_t)(pos & 0xFFFF);

  bool ok = true;
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_MODE,  mode);  delay(2);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_POS_H, posH);  delay(2);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_POS_L, posL);  delay(2);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_VEL,   vel);   delay(2);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_ACC,   acc);   delay(2);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_DEC,   dec);   delay(2);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR_TRIGGER, PR_TRIGGER_PR0);
  return ok;
}

// Run PR0 in velocity mode
bool driveMoveVelocity(uint16_t rpm) {
  bool ok = true;
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_MODE, PR_MODE_VELOCITY);  delay(5);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR0_VEL,  rpm);               delay(5);
  ok &= modbusWriteReg(SLAVE_ID, REG_PR_TRIGGER, PR_TRIGGER_PR0);
  return ok;
}

// Print motion status bits to DEBUG_SERIAL
void printMotionStatus(uint16_t status) {
  DEBUG_SERIAL.print(F("Motion status: 0x"));
  DEBUG_SERIAL.println(status, HEX);
  DEBUG_SERIAL.print(F("  Fault:              ")); DEBUG_SERIAL.println(status & 0x01 ? "YES" : "no");
  DEBUG_SERIAL.print(F("  Enabled:            ")); DEBUG_SERIAL.println(status & 0x02 ? "yes" : "NO");
  DEBUG_SERIAL.print(F("  Running:            ")); DEBUG_SERIAL.println(status & 0x04 ? "yes" : "no");
  DEBUG_SERIAL.print(F("  Command completed:  ")); DEBUG_SERIAL.println(status & 0x10 ? "yes" : "no");
  DEBUG_SERIAL.print(F("  Path completed:     ")); DEBUG_SERIAL.println(status & 0x20 ? "yes" : "no");
  DEBUG_SERIAL.print(F("  Homing completed:   ")); DEBUG_SERIAL.println(status & 0x40 ? "yes" : "no");
}

void printAlarmCode(uint16_t code) {
  DEBUG_SERIAL.print(F("Alarm code: 0x"));
  DEBUG_SERIAL.println(code, HEX);
  if (code == 0)       DEBUG_SERIAL.println(F("  No alarm"));
  if (code & 0x001)    DEBUG_SERIAL.println(F("  0x001: Over-current"));
  if (code & 0x002)    DEBUG_SERIAL.println(F("  0x002: Over-voltage"));
  if (code & 0x040)    DEBUG_SERIAL.println(F("  0x040: Current sampling fault"));
  if (code & 0x080)    DEBUG_SERIAL.println(F("  0x080: Failed to lock shaft (check motor wiring)"));
  if (code & 0x100)    DEBUG_SERIAL.println(F("  0x100: Auto-tuning fault"));
  if (code & 0x200)    DEBUG_SERIAL.println(F("  0x200: EEPROM fault"));
}

void printHelp() {
  DEBUG_SERIAL.println(F("\n─── DM556RS RS-485 Commander ────────────────────"));
  DEBUG_SERIAL.println(F("  help                    - This menu"));
  DEBUG_SERIAL.println(F("  enable                  - SW-enable the drive"));
  DEBUG_SERIAL.println(F("  disable                 - SW-disable the drive"));
  DEBUG_SERIAL.println(F("  jog+                    - JOG CW (send repeatedly <50ms to hold)"));
  DEBUG_SERIAL.println(F("  jog-                    - JOG CCW"));
  DEBUG_SERIAL.println(F("  stop                    - Emergency stop"));
  DEBUG_SERIAL.println(F("  abs <pos> <rpm> <acc> <dec>  - Absolute move (PR0)"));
  DEBUG_SERIAL.println(F("  rel <pos> <rpm> <acc> <dec>  - Relative move (PR0)"));
  DEBUG_SERIAL.println(F("  vel <rpm>               - Velocity mode (PR0)"));
  DEBUG_SERIAL.println(F("  status                  - Read motion status"));
  DEBUG_SERIAL.println(F("  current <dA>            - Set peak current (e.g. 32 = 3.2A)"));
  DEBUG_SERIAL.println(F("  read <hex_addr>         - Read register (e.g. read 0191)"));
  DEBUG_SERIAL.println(F("  write <hex_addr> <hex>  - Write register (e.g. write 6200 0041)"));
  DEBUG_SERIAL.println(F("  save                    - Save params to EEPROM"));
  DEBUG_SERIAL.println(F("  alarm                   - Read alarm code"));
  DEBUG_SERIAL.println(F("  clearalarm              - Clear current alarm"));
  DEBUG_SERIAL.println(F("──────────────────────────────────────────────────\n"));
}

// ─── Command Parser ────────────────────────────────────────────────────────
String inputBuffer = "";

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  DEBUG_SERIAL.print(F("> "));
  DEBUG_SERIAL.println(cmd);

  if (cmd == "help") {
    printHelp();

  } else if (cmd == "enable") {
    DEBUG_SERIAL.println(driveEnable(true) ? F("Enabled.") : F("ERROR: no response"));

  } else if (cmd == "disable") {
    DEBUG_SERIAL.println(driveEnable(false) ? F("Disabled.") : F("ERROR: no response"));

  } else if (cmd == "jog+") {
    DEBUG_SERIAL.println(driveJogCW() ? F("JOG CW sent.") : F("ERROR"));

  } else if (cmd == "jog-") {
    DEBUG_SERIAL.println(driveJogCCW() ? F("JOG CCW sent.") : F("ERROR"));

  } else if (cmd == "stop") {
    DEBUG_SERIAL.println(driveStop() ? F("Stopped.") : F("ERROR"));

  } else if (cmd == "save") {
    DEBUG_SERIAL.println(driveSave() ? F("Saved.") : F("ERROR"));

  } else if (cmd == "alarm") {
    int32_t v = modbusReadReg(SLAVE_ID, REG_CURRENT_ALARM);
    if (v < 0) DEBUG_SERIAL.println(F("ERROR: no response"));
    else printAlarmCode((uint16_t)v);

  } else if (cmd == "clearalarm") {
    DEBUG_SERIAL.println(driveClearAlarm() ? F("Alarm cleared.") : F("ERROR"));

  } else if (cmd == "status") {
    int32_t v = modbusReadReg(SLAVE_ID, REG_MOTION_STATUS);
    if (v < 0) DEBUG_SERIAL.println(F("ERROR: no response"));
    else printMotionStatus((uint16_t)v);

  } else if (cmd.startsWith("current ")) {
    uint16_t dA = (uint16_t)cmd.substring(8).toInt();
    DEBUG_SERIAL.println(modbusWriteReg(SLAVE_ID, REG_PEAK_CURRENT, dA) ?
                         F("Current set.") : F("ERROR"));

  } else if (cmd.startsWith("vel ")) {
    uint16_t rpm = (uint16_t)cmd.substring(4).toInt();
    DEBUG_SERIAL.println(driveMoveVelocity(rpm) ? F("Velocity mode started.") : F("ERROR"));

  } else if (cmd.startsWith("abs ") || cmd.startsWith("rel ")) {
    bool rel = cmd.startsWith("rel ");
    String args = cmd.substring(4);
    // Parse: pos rpm acc dec
    int s1 = args.indexOf(' ');
    int s2 = args.indexOf(' ', s1+1);
    int s3 = args.indexOf(' ', s2+1);
    if (s1<0 || s2<0 || s3<0) {
      DEBUG_SERIAL.println(F("Usage: abs <pos> <rpm> <acc> <dec>"));
      return;
    }
    int32_t  pos = args.substring(0,s1).toInt();
    uint16_t vel = args.substring(s1+1,s2).toInt();
    uint16_t acc = args.substring(s2+1,s3).toInt();
    uint16_t dec = args.substring(s3+1).toInt();
    DEBUG_SERIAL.print(rel ? F("Relative move: ") : F("Absolute move: "));
    DEBUG_SERIAL.print(pos); DEBUG_SERIAL.print(F(" steps @ "));
    DEBUG_SERIAL.print(vel); DEBUG_SERIAL.println(F(" rpm"));
    DEBUG_SERIAL.println(driveMovePosition(rel, pos, vel, acc, dec) ?
                         F("Move triggered.") : F("ERROR"));

  } else if (cmd.startsWith("read ")) {
    uint16_t addr = (uint16_t)strtol(cmd.substring(5).c_str(), nullptr, 16);
    int32_t v = modbusReadReg(SLAVE_ID, addr);
    if (v < 0) {
      DEBUG_SERIAL.println(F("ERROR: no response or CRC fail"));
    } else {
      DEBUG_SERIAL.print(F("Reg 0x")); DEBUG_SERIAL.print(addr, HEX);
      DEBUG_SERIAL.print(F(" = 0x")); DEBUG_SERIAL.print((uint16_t)v, HEX);
      DEBUG_SERIAL.print(F(" (dec ")); DEBUG_SERIAL.print(v); DEBUG_SERIAL.println(F(")"));
    }

  } else if (cmd.startsWith("write ")) {
    String args = cmd.substring(6);
    int sp = args.indexOf(' ');
    if (sp < 0) { DEBUG_SERIAL.println(F("Usage: write <hex_addr> <hex_val>")); return; }
    uint16_t addr = (uint16_t)strtol(args.substring(0,sp).c_str(), nullptr, 16);
    uint16_t val  = (uint16_t)strtol(args.substring(sp+1).c_str(), nullptr, 16);
    DEBUG_SERIAL.println(modbusWriteReg(SLAVE_ID, addr, val) ?
                         F("Written.") : F("ERROR"));

  } else {
    DEBUG_SERIAL.print(F("Unknown command: ")); DEBUG_SERIAL.println(cmd);
    DEBUG_SERIAL.println(F("Type 'help' for commands."));
  }
}

// ─── setup() / loop() ──────────────────────────────────────────────────────
void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 3000) {}  // Wait for USB serial

  pinMode(DE_RE_PIN, OUTPUT);
  rs485_rx_enable();

  // 8N1 to match Pr5.24=4 (factory default). Change if you altered parity.
  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1);

  DEBUG_SERIAL.println(F("\nDM556RS RS-485 Controller ready."));
  DEBUG_SERIAL.print(F("RS-485 baud: ")); DEBUG_SERIAL.println(RS485_BAUD);
  DEBUG_SERIAL.print(F("Slave ID:    ")); DEBUG_SERIAL.println(SLAVE_ID);
  printHelp();
}

void loop() {
  // Read chars from USB serial, process on newline
  while (DEBUG_SERIAL.available()) {
    char c = DEBUG_SERIAL.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

/*
```

---

## Key things to know before you power up

**Baud rate — check your DIP switches first.** The datasheet has two confusing "defaults":
- SW6=OFF, SW7=ON → **38400** ("factory" setting, what ships from the factory)
- SW6=ON, SW7=ON → **115200** ("default" in software)

The sketch uses 38400 — change `RS485_BAUD` if yours differs.

**Slave ID** is set by SW1–SW5 (all ON = ID 1). Change `SLAVE_ID` if yours is different.

**The drive is already enabled by default via DI1** (normally-closed hardware enable). The `enable`/`disable` commands use the *software override* (Pr0.07). You only need `enable` if someone has wired DI1 open or you've reconfigured it.

**Wiring your RS-485 transceiver:**
- Teensy TX1 (pin 0) → transceiver DI
- Teensy RX1 (pin 1) → transceiver RO
- Teensy pin 2 → transceiver DE and RE (tie them together)
- Transceiver A/B → drive's RS485+/RS485- (CN5 pins 1 & 2)
- Add a 120Ω termination resistor across A/B at the drive end — set DM556RS SW8=ON if it's the last device on the bus

**Example session in Serial Monitor (set to 115200, NL+CR):**
```
status              ← check if comms work; should see bit1=enabled
rel 10000 300 50 50 ← move 10000 steps relative, 300 rpm, acc/dec 50ms
stop
vel 200             ← spin at 200 rpm
stop
current 32          ← set peak current to 3.2A
save
*/