# DM556RS Stepper Control

A simple browser-based control panel for a STEPPERONLINE DM556RS Modbus RTU stepper motor
driver, connected via a USB-to-RS485 adapter. No install, no backend server — the page talks
directly to the adapter using the [Web Serial API](https://developer.chrome.com/docs/capabilities/serial).

> **Safety first.** Read the "Before you rely on this tool" section below before running a motor
> unattended. This tool's connection-loss warning is informational only — it **cannot** stop the
> motor if the serial link goes down. Only the drive's own E-Stop wiring, enable input, or power
> switch can do that.

## Requirements

- **Google Chrome or Microsoft Edge** on a desktop or laptop. Web Serial is not available in
  Safari, Firefox, or on most mobile browsers.
- A USB-to-RS485 adapter wired to the DM556RS driver's RS485 A/B terminals (most adapters handle
  the RS485 transmit/receive direction switching automatically in hardware — no special driver
  logic is needed on the computer side).
- The driver's DIP switches (or its current register settings) for slave ID and baud
  rate/parity/stop bits — you'll enter matching values in the app's Settings panel.

## Using it

1. Open the page (hosted on GitHub Pages, or run locally — see below).
2. Open the **Settings** panel and confirm the Slave ID, Baud rate, and Data format match your
   drive's DIP switches (factory default: slave ID 1, 38400 baud, 8N1).
3. Click **Connect** and choose your USB-RS485 adapter from the browser's port picker.
4. Set a speed and direction, click **Start**. Use **Stop** for a normal ramp-down, or the big
   red **EMERGENCY STOP** button (or the Space bar) for an immediate stop.

Settings are remembered in your browser between visits (not synced anywhere).

### Running it locally (for development/testing)

Web Serial requires a "secure context" — `https://` (which GitHub Pages provides) or
`http://localhost`. Opening `index.html` directly as a `file://` URL will not work. From the
`Web/` folder:

```
python3 -m http.server
```

then open `http://localhost:8000` in Chrome.

## Project layout

- `Web/` — the app itself (deployed to GitHub Pages via `.github/workflows/pages.yml`).
  - `js/modbus-serial.js` — Web Serial transport + Modbus RTU (FC03/06/10) framing and a
    single-flight request queue.
  - `js/dm556rs.js` — DM556RS register map and command sequences (run/stop/e-stop/status/alarm).
  - `js/settings.js` — browser-local settings storage.
  - `js/app.js` — UI wiring, connection/run state machine, status polling.
- `Firmware/rs485_motor_driver_coms_test.ino` — a Teensy 4.1 reference sketch this project's
  Modbus logic was ported from. Not used by the web app; kept as a manual bench-testing fallback
  (e.g. probing arbitrary registers from a serial console) if you ever need to debug the driver
  independent of a browser.
- `DM556RS_Manual.pdf` — the driver's manual.

## Design notes

There's no simple "speed register + run bit" on this driver — continuous-speed control is done
via one of its 16 motion "path" slots (Path 0) configured in velocity mode, which is what this
app uses so that ease-in and ease-out (acceleration/deceleration) can be set independently.

**Direction is set by sending a signed velocity value** (negative = reverse) — confirmed against
real hardware: positive rpm spins CW, negative spins CCW, no fallback needed. `Web/js/dm556rs.js`
still has two documented-but-unused fallback mechanisms (`setDirectionSense()`,
`jogCW()`/`jogCCW()`) left in place in case a different unit/firmware revision behaves
differently, but the shipped `startRun()` uses the signed-velocity approach.

**Two of the driver's status registers proved unreliable on tested hardware, so the app avoids
depending on them:**
- The **"running" status bit** (`0x1003` bit 2) does not mean "the shaft is turning." It stays
  set indefinitely after commanding a velocity-mode run at 0 rpm (how graceful Stop works), and
  can read as stale/true right after reconnecting even with the motor stationary.
- The **"feedback (actual) velocity" register** (`0x1046`/`0x1047`) read `0x0000`/`0x0000` while
  the motor was visibly spinning - it does not appear to be populated with real data on the
  tested unit (likely no functioning encoder feedback loop, despite being documented generically
  for the product family).

What the app uses instead:
- The **live speed readout** ("Speed reported by drive") reads the **profile/commanded velocity**
  register (`0x1044`/`0x1045`) instead, which was confirmed to track real commanded values
  (correctly read a signed value matching the commanded RPM and direction). This reflects what
  the drive's trajectory generator is currently outputting, not confirmed encoder feedback - it's
  labeled accordingly rather than as "actual speed."
- **Graceful Stop completion** is computed from the drive's own documented ramp spec (ease-out is
  ms per 1000 rpm) rather than polled from either status register: the app waits
  `easeOutMs * (rpm / 1000)` plus a margin, then considers the motor stopped. This avoids
  depending on any status signal that's proven unreliable on this hardware.
- **On-connect state reconciliation** additionally checks that the profile velocity is near zero
  before trusting the "running" bit, since that bit alone is known to be stale.

**If Connect fails with "Failed to open serial port"** after a browser reload, crash, or closing
the tab without clicking Disconnect: the OS-level port handle can be left locked (Web Serial does
not reliably release it on navigation). Unplug and replug the USB-RS485 adapter to clear it. The
app also makes a best-effort attempt to close the port on page unload to reduce how often this
happens, but it isn't guaranteed (browsers don't guarantee async work completes during unload).

## Before you rely on this tool: commissioning checklist

Do this once, on the bench, before using the tool for real work:

1. **Bench safety setup.** Motor mechanically free to spin (not coupled to anything that an
   unexpected direction or speed could damage). Physical E-stop/power switch within reach.
   Record the drive's DIP switch state before touching the software.
2. **Read-only link check.** Connect. In the Advanced panel, read register `1003` (status) and
   `2203` (alarm) and confirm clean responses with no pre-existing alarm.
3. ✅ **E-Stop check while stationary.** Press E-Stop with nothing running; confirm no error/fault.
4. ✅ **Lowest-risk motion test.** Start at 5–10 RPM briefly. Confirm the shaft turns, and note
   which physical direction corresponds to "CW" in the app. Stop; confirm it decelerates.
5. ✅ **Direction verification.** Confirmed on hardware — see Design notes above.
6. **RPM range tuning.** Check there's no stall near your intended minimum RPM and nothing unsafe
   at your intended maximum; update the Settings panel's RPM min/max to match.
7. **Ease-in/ease-out tuning.** Adjust the sliders at a mid-range RPM and confirm the ramps look
   right. Also check what happens if you change RPM *while already running* — note whether it
   ramps smoothly to the new speed or restarts from zero, and adjust expectations accordingly.
8. **Fault and link-loss test.** Trigger a safe, recoverable fault to confirm the alarm message
   and Clear Fault flow work. Then unplug the USB adapter mid-run to confirm the connection-lost
   banner appears — and confirm for yourself that (as documented) the motor keeps running until
   you intervene physically.
9. **Soak test.** Run continuously for 30–60+ minutes while watching for drift, faults, or
   comms errors before trusting it for unattended daily use.
