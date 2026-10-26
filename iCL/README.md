# iCL Stepper Control

> Part of the [Stepper-Motor-Tools](../README.md) catch-all repo.

A browser-based control panel for a Teensy 4.1 running the `iCL_Motor_Test` sketch, which drives
a StepperOnline iCL57/60 integrated closed-loop stepper motor through its PUL/DIR inputs. No
install, no backend server — the page talks directly to the Teensy over USB using the
[Web Serial API](https://developer.chrome.com/docs/capabilities/serial).

Unlike the other two tools in this repo, there's no Modbus and no third-party drive protocol to
reverse-engineer here — the Teensy sketch **is** the protocol. It was written with this exact web
UI in mind (see its `stream on|off` command and the comment above `printStreamLine()`).

> **Safety first.** Read "Before you rely on this tool" below before running this unattended.
> The Teensy's step generator is a free-running hardware timer (`IntervalTimer`) that keeps
> producing pulses regardless of whether the USB/serial link is alive — the connection-loss
> warning in the app is purely informational. It **cannot** stop the motor; only the drive's
> power or a physical E-Stop can guarantee that.

## Requirements

- **Google Chrome or Microsoft Edge** on a desktop or laptop. Web Serial is not available in
  Safari, Firefox, or on most mobile browsers.
- A Teensy 4.1 flashed with `Firmware/iCL_Motor_Test/iCL_Motor_Test.ino`, wired to the drive's
  PUL/DIR inputs per the sketch's header comment (NPN sinking, common-anode; Teensy pin HIGH
  turns the drive's optocoupler on).
- The drive's **SW6 DIP switch set to PUL&DIR mode** (iCL57-xx/iCL60-xx — `off` position; see
  `iCL_Series_User_Manual.pdf` section 4.2). This sketch always drives PUL/DIR-style signals, not
  CW&CCW, so the wrong DIP setting here means nothing will move no matter what the app sends.
- The drive's **SW1–SW4 microstep DIP switches matching the "Steps per revolution" setting** in
  this app's Settings panel (sketch/app default: 400, all switches on — see the manual's
  microstep table for other values).

## Using it

1. Flash the Teensy with the sketch, wire it up, and set the drive's DIP switches as above.
2. Open the page (hosted on GitHub Pages, or run locally — see below).
3. Click **Connect** and choose the Teensy's USB-serial port from the browser's port picker.
4. Set a speed and direction, click **Start**. Use **Stop** for a ramped stop (uses the configured
   decel rate), or the big red **EMERGENCY STOP** button (or the Space bar) to cut pulses
   immediately, bypassing the ramp.

Settings are remembered in your browser between visits, per site origin — but see "Design notes"
below: the firmware itself has no persistent storage, so the device's *actual* current
accel/decel/max-rpm/steps-per-rev/invert reset to the sketch's compiled-in defaults every time the
Teensy powers up, independent of whatever this app remembered from last time.

### Running it locally (for development/testing)

Web Serial requires a "secure context" — `https://` (which GitHub Pages provides) or
`http://localhost`. From the `Web/` folder:

```
python3 -m http.server
```

then open `http://localhost:8000` in Chrome.

## Project layout

- `Web/` — the app itself (deployed to GitHub Pages at the `/icl/` subpath via the repo root's
  `.github/workflows/pages.yml`).
  - `js/icl-serial.js` — Web Serial transport (a **fork** of the same generic `SerialTransport`
    used by the other tools — see "Design notes" for why) plus `LineSerial`, a newline-delimited
    line reader/writer replacing those tools' Modbus framing, since this firmware's protocol is
    plain ASCII commands with no strict request/response structure.
  - `js/icl-stepper.js` — command wrapper (`ICLStepperDrive`) and parsing for the sketch's
    `S,<target>,<current>,<steps>,<state>` stream lines and its `status` command's text output.
  - `js/settings.js` — browser-local settings storage.
  - `js/app.js` — UI wiring and a small connected/idle/running state machine, driven by the
    incoming telemetry stream rather than a polling loop (see "Design notes").
- `Firmware/iCL_Motor_Test/iCL_Motor_Test.ino` — the Teensy sketch itself. This **is** the
  protocol spec for this tool; there's no separate protocol document (the vendor manual below
  only covers the drive's physical PUL/DIR/DIP-switch side).
- `iCL_Series_User_Manual.pdf` — the drive's manual (wiring, DIP switches, electrical specs).

## Design notes

**Why `icl-serial.js` forks `SerialTransport` instead of sharing it with the other tools:** same
reasoning as the AIM servo tool gives for its own fork — each tool here is a standalone static
page served from its own `Web/` folder (`python3 -m http.server`, as above), and a module shared
across folders would break that for all of them. `SerialTransport` itself (the actual Web Serial
port-management code) is identical to the other tools' copies; only the protocol layered on top
differs.

**No Modbus here — this firmware speaks plain text.** The other two tools in this repo talk
Modbus RTU to a drive's own onboard controller. This one is different: the Teensy itself *is* the
step generator (driving PUL/DIR directly via a hardware `IntervalTimer`), and the "protocol" is
just newline-terminated ASCII commands (`rpm 100`, `stop`, `accel 300`, …) with human-readable
`# ...` reply lines and no formal request/response framing. `LineSerial.sendAndCollect()` handles
this with a best-effort "collect lines until the link goes quiet" approach rather than Modbus-style
exact framing — appropriate here since nothing safety-critical depends on perfectly correlating a
command with its reply (unlike, say, a Modbus CRC check).

**Telemetry is push-based, not polled.** The sketch streams a status line at a fixed ~20Hz once
`stream on` is sent, instead of being polled for status like the Modbus-based tools. `app.js` has
no polling loop; it just reacts to each incoming stream line, and detects a lost connection via an
activity watchdog (no stream line for `linkLostTimeoutMs`, default 1000ms) rather than counting
missed polls.

**No fault/alarm detection.** The drive does have a real alarm output (`ALM-`/`ALM+`, flagging
over-voltage/over-current/position-following-error — see the manual section 3.1.3), but the
current sketch doesn't wire it to a Teensy input or report it over serial, so this app has no way
to know about a drive-side fault. If that's ever needed, it means adding an `ALM` digital input
and a corresponding serial command/stream field to the sketch first — this app can't detect what
the firmware doesn't tell it.

**No enable/disable or current-sensing safety cutoff**, unlike the AIM servo tool — this setup has
neither a software enable register nor any current feedback, so there's nothing equivalent to
build a safety trip on. E-Stop here means "stop sending step pulses immediately," which is already
the strongest stop action this firmware/wiring combination supports.

**Steps-per-revolution and invert are stopped-only**, matching the firmware's own rule (it refuses
these commands while moving, per the sketch's `motorStopped()` checks) — the Settings panel also
disables pushing these to the device while running, rather than relying solely on the firmware to
reject them.

## Before you rely on this tool: commissioning checklist

Do this once, on the bench, before using the tool for real work:

1. **Bench safety setup.** Motor mechanically free to spin. Physical power switch within reach.
   Confirm the drive's DIP switches: SW6 = PUL&DIR mode, SW1–SW4 matching the steps/rev you plan
   to use.
2. **Read-only link check.** Connect. Confirm status/target/actual/steps all show sensible values
   and update live (open the Advanced raw console and send `status` to see the full text dump).
3. **E-Stop check while stationary.** Press E-Stop with nothing running; confirm no error and the
   state stays idle.
4. **Lowest-risk motion test.** Start at a low RPM (e.g. 10–20) briefly. Confirm the shaft turns,
   note which physical direction is "CW" in the app (SW5 sets the drive's own default direction
   sense, which interacts with this), and confirm Stop ramps down smoothly.
5. **Direction verification.** Confirm CCW actually reverses the shaft. If it's backwards for
   your wiring, use the **Invert** setting (stopped-only) rather than rewiring, and re-verify.
6. **RPM range tuning.** Check there's no stall near your intended minimum RPM and nothing unsafe
   at your intended maximum; set RPM min/max in Settings to match.
7. **Accel/decel tuning.** Adjust at a mid-range RPM and confirm the ramps feel right; these apply
   live even while running, unlike steps/rev and invert.
8. **Link-loss test.** Unplug the Teensy's USB cable mid-run (power it from elsewhere if you want
   the motor to keep spinning for this test) to confirm the link-lost banner appears, and confirm
   for yourself that — as documented — the motor keeps running until you intervene physically.
9. **Soak test.** Run continuously for 30–60+ minutes while watching for drift or unexpected
   behavior before trusting it for unattended daily use.
