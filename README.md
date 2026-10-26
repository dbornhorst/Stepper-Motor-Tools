# Stepper-Motor-Tools

A catch-all home for browser-based motor-control tools: static pages that talk directly to
hardware over a USB-to-serial adapter using the [Web Serial API](https://developer.chrome.com/docs/capabilities/serial) —
no install, no backend server, just open a link in Chrome or Edge. Each tool is hosted on
GitHub Pages from this same repo.

## Tools

| Tool | Live page | Source / docs |
|---|---|---|
| DM556RS Stepper Control | `/dm556rs/` | [`DM556RS/README.md`](DM556RS/README.md) |
| AIM Servo Control | `/aim-servo/` | [`AimServo/README.md`](AimServo/README.md) |
| iCL Stepper Control | `/icl/` | [`iCL/README.md`](iCL/README.md) |

## Adding a new tool

1. Create a new top-level folder for it (e.g. `MyDriver/`) with its own `Web/` subfolder holding
   `index.html`, `css/`, `js/` — a static, no-build-step page, following the pattern of the
   existing tools.
2. Add a link to it from [`Landing/index.html`](Landing/index.html).
3. Add a `cp -r MyDriver/Web/. dist/my-driver/` line to the "Assemble site" step in
   [`.github/workflows/pages.yml`](.github/workflows/pages.yml).
4. Give it its own `README.md` inside its folder (requirements, usage, design notes, a
   commissioning checklist for real hardware) — see the existing tools' READMEs for the shape.

Each tool keeps its own settings (`localStorage`), its own Modbus/serial transport code, and its
own register map/driver module — there's no shared runtime code between tools by design, since
this is a small collection of static pages, not a published library. See either tool's README for
why (briefly: keeping each tool servable standalone from its own folder, with `python3 -m
http.server`, matters more here than de-duplicating a few hundred lines of protocol framing).
