# AIM Servo Control

> Part of the [Stepper-Motor-Tools](../README.md) catch-all repo.

A browser-based control panel for a YZ-AIM integrated Modbus RTU servo drive, connected via a
USB-to-RS485 adapter. No install, no backend server — the page talks directly to the adapter
using the [Web Serial API](https://developer.chrome.com/docs/capabilities/serial).

This replaces an existing Python CLI/shell (kept in `PythonReference/` for reference) that worked
but didn't "transport well" — it needed Python, pip, a venv, and POSIX-only terminal APIs for its
live-control modes, none of which a non-technical user should have to deal with.

> **Safety first.** Read "Before you rely on this tool" below before running this unattended.
> The connection-loss warning in the app is informational only — it **cannot** stop the servo if
> the serial link goes down. Only the drive's enable input or power can do that.

## Requirements

- **Google Chrome or Microsoft Edge** on a desktop or laptop. Web Serial is not available in
  Safari, Firefox, or on most mobile browsers.
- A USB-to-RS485 adapter wired to the drive's 485A/485B terminals.
- The drive's Modbus slave address (default 1) — this drive has no configurable baud/parity
  register; it's fixed at **19200 baud, 8N1**, so the app always opens the port with those
  settings.

## Using it

1. Open the page (hosted on GitHub Pages, or run locally — see below).
2. Open **Settings** and confirm the Slave ID matches your drive (default 1). Pick your motor
   model from the dropdown to pre-fill a sensible RPM max, or leave it as "Custom" and set RPM
   max yourself.
3. Click **Connect** and choose your USB-RS485 adapter from the browser's port picker.
4. Use **Position control** for relative/absolute moves, or **Speed mode** to jog continuously.

Settings are remembered in your browser between visits, per site origin (a local test server and
the GitHub Pages URL don't share settings — you'll set them again the first time on each).

### Running it locally (for development/testing)

Web Serial requires a "secure context" — `https://` (which GitHub Pages provides) or
`http://localhost`. From the `Web/` folder:

```
python3 -m http.server
```

then open `http://localhost:8000` in Chrome.

## Project layout

- `Web/` — the app itself (deployed to GitHub Pages at the `/aim-servo/` subpath via the repo
  root's `.github/workflows/pages.yml`).
  - `js/aim-modbus.js` — Web Serial transport + Modbus RTU (FC03/06/10, plus this drive's two
    vendor-specific move commands 0x78/0x7B) framing and a single-flight request queue. This is a
    **fork** of `DM556RS/Web/js/modbus-serial.js`, not a shared module — see "Design notes" below
    for why — so a generic fix to one should be ported to the other.
  - `js/aim-servo.js` — register map and command sequences (`AIMServoDrive`).
  - `js/settings.js` — browser-local settings storage.
  - `js/app.js` — UI wiring, connection/motion state machine, tiered status polling.
- `PythonReference/` — the original Python CLI/shell this tool replaces (`aim_servo.py`,
  `aim_shell.py`, `rock_servo.py`, `configure_pulse_dir.py`). Not used by the web app; kept for
  reference and as a bench-testing fallback.
- `YZ-AIM_Manual.pdf` — the vendor's manual.

## Design notes

**V1 is core controls only.** Status/telemetry, enable/disable output, relative move, software
absolute move, zero position, speed-mode jog with start/stop, and current-based auto-release
safety. Deliberately **not** included (may come later): the Python reference's keyboard-live
"spin" control, auto-centering "oscillate" mode, sine-profile "rock" motion, the cosmetic
"resistance" heuristic, `configure_pulse_dir.py`-style pulse/direction-mode switching, the native
hardware absolute-move opcode as a wired UI feature, and multi-axis support.

**Why `aim-modbus.js` is a fork, not a shared module with the DM556RS tool:** both tools are
static pages meant to be served standalone from their own `Web/` folder (`python3 -m
http.server`, as above). A shared module living outside both `Web/` folders would break that
workflow for both tools. The file is small and the underlying protocol (Modbus RTU framing)
doesn't change, so a fork is simpler to maintain here than a real shared package.

**Absolute move is software-only**, exactly like the Python reference: read the current absolute
position, compute the delta to the target, and send a relative move (function code `0x78`). The
drive also has a native hardware absolute-move opcode (`0x7B`), documented in the manual, but the
Python reference found it "acknowledged but did not produce reliable physical motion" on the
bench — it's implemented in `aim-servo.js` for potential future use but not wired into any control.

**Direction-reversal method defaults to `signed` target speed — confirmed on real hardware.**
The vendor manual documents the target-speed register's range as unsigned (0–3000 r/min), which
initially looked like reason to doubt a signed/negative write would reverse direction, so this
tool shipped with a `dir-polarity` (register `0x09`) fallback as the default instead. Commissioning
against real hardware showed the opposite of what the manual's wording suggested: `dir-polarity`
has **no effect at all** on Modbus speed-mode direction (that register turned out to govern only
the drive's *external* hardware DIR pin, used in a completely different control mode), while a
plain **signed** target-speed value reverses direction correctly despite the documented unsigned
range. `dir-polarity` is kept implemented only as a fallback in case a different unit/firmware
revision differs. Switch via Settings → Direction method if you ever need to.

**Device address (Slave ID) is read-only from this tool.** The vendor manual lists register
`0x15` as read-only, and separately mentions a function code `0x7A` for changing the device
address — but doesn't document its frame format with a worked example anywhere in the manual, so
it isn't implemented here. Changing the slave address requires the vendor's own configuration
tool or DIP switches, if this drive model has them.

**No "clear alarm" register was found** in the manual or the Python reference (unlike the
DM556RS tool, which has one). Alarms are treated as read-only/informational: motion is blocked
while an alarm is active, and Resume requires a fresh read confirming it's actually cleared.

**The manual describes writing directly to the absolute-position registers (`0x16`/`0x17`) via
function code `0x10` as an alternative way to command absolute motion** (see the manual's
"Modbus mode host control process" walkthrough), which would contradict treating those registers
as a pure read-only-plus-rezero counter. This tool follows the Python reference's approach
instead (bench-tested: relative motion works, `0x7B` doesn't) rather than the untested
manual-described alternative — flagged here in case this is worth investigating later.

## Before you rely on this tool: commissioning checklist

Do this once, on the bench, before using the tool for real work:

1. **Bench safety setup.** Shaft mechanically free to spin. Physical enable/power switch within
   reach. Record the drive's model and rated current.
2. **Read-only link check.** Connect. Confirm alarm/output-enable/speed/voltage/temperature/
   position all read cleanly with no pre-existing alarm.
3. **Enable/disable check while stationary.** Toggle output enable with the shaft free; confirm
   no unexpected motion.
4. ✅ **Lowest-risk motion test.** A small relative move (e.g. a few hundred pulses). Confirm the
   shaft turns the expected small amount and the move reports completion.
5. **Software absolute-move verification.** Read the current position, request a nearby absolute
   target, confirm it lands within tolerance.
6. **Zero-position verification.** At a safe reference point, zero; confirm position reads back 0.
7. ✅ **Direction-reversal verification.** Confirmed on hardware — see "Design notes" above
   (signed target speed reverses direction correctly; the `dir-polarity` fallback does not affect
   Modbus speed-mode direction at all).
8. **Current-cutoff threshold tuning.** Observe baseline holding current at rest, then typical
   current during small moves and low-RPM jogging. Set the cutoff comfortably above
   worst-case-normal but below a stall current; deliberately induce one safe, low-RPM stall to
   confirm output disables immediately and requires Resume.
9. **Alarm handling.** Since no clear-alarm register was found, confirm that alarms clear on
   their own once the underlying condition is resolved (per the drive's own alarm-processing
   behavior in the manual), and that this tool's Resume flow correctly detects that.
10. **Link-loss test.** Unplug the adapter mid-jog; confirm the link-lost banner appears and that
    the servo keeps spinning physically until you intervene — the app cannot stop it without a
    live connection.
11. **Soak test.** Run jogging and/or repeated relative moves for 30–60+ minutes, watching for
    drift, alarms, or comms errors, before trusting it for unattended use.
