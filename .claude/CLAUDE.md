# CLAUDE.md — PUP EN24C Radar Linux Port

## What this project is

A port of the **PUP EN24C T2R4** 24 GHz FMCW radar control software from Windows/MATLAB to a **standalone Linux ARM64 executable** for Raspberry Pi 4/5. The target binary runs **without any MATLAB runtime** on the Pi.

Hardware: 2Tx / 4Rx FMCW radar over USB 3.x, driven by a **Cypress FX3** controller. Firmware (`SDR_USB_FW.hex`) is downloaded into FX3 RAM at every plug-in via the Cypress `0xA0` boot protocol.

## Repository layout

| Path | Status | Purpose |
|---|---|---|
| [docs/](docs/) | reference | Datasheets and manual for the radar (PDF) |
| [docs/Matlab src/](Matlab%20src/) | **read-only reference** | Existing Windows GUI ([PUPradarGUI.m](Matlab%20src/PUPradarGUI.m)) + pre-built `.mexw64`. Source of truth for the radar control surface. Do **not** modify or rebuild. |
| [docs/Mex src/](Mex%20src/) | **read-only reference** | C++ MEX sources wrapping Cypress CyAPI (Windows-only) + protocol docs ([PupRadar_Protocol.docx](Mex%20src/PupRadar_Protocol.docx), [PupRadar_DataFormat.docx](Mex%20src/PupRadar_DataFormat.docx)). Reference for porting, not a build input. |
| `linux/` | **NEW — all new code lives here** | Cross-compiled C++17 executable, libusb-1.0 based. See [the plan](../../../.claude/plans/frolicking-strolling-dijkstra.md). |

## Architecture (capture-only MVP)

```
pupradar_capture (single aarch64 ELF)
 ├─ CLI                → argparse-style flags (--fc, --bw, --fs, --duration, --tx, --rx, --out)
 ├─ RadarSession       → ports the 6 MATLAB API funcs (init, configureFMCW, setActive, captureIQ, shutdown)
 ├─ UsbDevice          → libusb-1.0 wrapper (replaces all CyAPI calls)
 └─ IqWriter           → interleaved int16 I/Q .bin + JSON sidecar
```

The MATLAB → C++ mapping:

| MATLAB / MEX (Windows) | C++ on Linux |
|---|---|
| `PUPradar_initiating` | `RadarSession::init()` |
| `Send_Basic_Parameter` + `Send_PLL_Sawtooth` | `RadarSession::configureFMCW()` |
| `SetActiveParameters` | `RadarSession::setActive()` |
| `GetComplexData` (looped) | `RadarSession::captureIQ(duration)` |
| `usbcheckchip` (CyAPI) | `UsbDevice::openByVidPid()` (libusb) |
| `usbdownload` (CyAPI 0xA0) | `UsbDevice::downloadFirmware()` (libusb control xfer) |
| `usbsetinterface{0,1}` | `UsbDevice::claimInterface()` |
| `miniradargetdata` (bulk) | `UsbDevice::bulkRead()` |
| `miniradarputdata` (bulk) | `UsbDevice::controlTransfer()` |
| `Send_PLL_CW` | **deferred** — sawtooth only in MVP |
| `PUPradarGUI.fig` + GUIDE callbacks | **discarded** — CLI only |

## Core decisions (locked)

- **Manual C++ rewrite**, not MATLAB Coder, not MATLAB Compiler. The control surface is small (~6 functions, mostly USB packet packing); codegen would add friction without payoff.
- **Target:** Raspberry Pi 4 / Pi 5, **Raspberry Pi OS 64-bit (aarch64)**. USB 3.0 required for sustained IQ throughput.
- **Build host:** WSL on the Windows dev machine, cross-compiling with `aarch64-linux-gnu-g++`.
- **Output:** raw interleaved `int16` I/Q in `<out>.bin` plus `<out>.json` sidecar with capture parameters. **No on-device DSP** in the MVP — all FFT/range-Doppler/CFAR work stays on the host.
- **Dependencies:** libusb-1.0 (system), nlohmann/json (header-only, vendored). No MATLAB runtime, no FFTW, no HDF5.

## Working rules for Claude in this repo

1. **Never modify `docs/Matlab src/` or `docs/Mex src/`.** They are the canonical reference for the radar's wire protocol; treat them as read-only.
2. **All new code goes under `linux/`.** Do not scatter Linux-port files at the project root.
3. **Wire-protocol authority is the .docx files first**, MATLAB + MEX source second. If the docs and code disagree, ask the user — don't guess.
4. **No DSP in the MVP.** If a request implies on-device FFT, range-Doppler, CFAR, or detection, stop and confirm — that is explicitly deferred.
5. **No GUI.** No live plotting, no Qt, no web UI. CLI only.
6. **Firmware download is the highest-risk step.** After the FX3 receives the `0xA0` boot, it re-enumerates — the code must wait and re-open. Don't assume libusb handles this transparently (it doesn't; CyAPI did).
7. **Throughput first, async second.** Start with synchronous `libusb_bulk_transfer`. Move to async (`libusb_submit_transfer` with an in-flight ring) only if measured throughput on the Pi is insufficient.
8. **udev rules ship with the binary.** The deploy script must install `/etc/udev/rules.d/99-pupradar.rules` so the executable doesn't require root.

## Key reference files (read these before making protocol decisions)

- [PupRadar_Protocol.docx](Mex%20src/PupRadar_Protocol.docx) — authoritative on-the-wire protocol
- [PupRadar_DataFormat.docx](Mex%20src/PupRadar_DataFormat.docx) — IQ packet layout
- [PUPradarGUI.m](Matlab%20src/PUPradarGUI.m) — search for `PUPradar_initiating`, `GetComplexData`, `Send_Basic_Parameter`, `Send_PLL_Sawtooth`, `SetActiveParameters`
- [usbdownload.cpp](Mex%20src/usbdownload.cpp) — firmware download reference (highest-risk port)
- [miniradargetdata.cpp](Mex%20src/miniradargetdata.cpp) — IQ ingest reference (endpoint, packet size, timeout)
- [usbcheckchip.cpp](Mex%20src/usbcheckchip.cpp) — VID/PID + enumeration
- [PUP_EN24C_T2R4_datasheet.pdf](docs/PUP_EN24C_T2R4_datasheet.pdf), [PUP_EN24C_T2R4_Manual.pdf](docs/PUP_EN24C_T2R4_Manual.pdf) — RF parameter ranges

## MCP tools available

A MATLAB MCP server is configured in [.mcp.json](.mcp.json) — Claude can run MATLAB code, run `.m` files, and inspect installed toolboxes on the Windows dev machine. Useful during porting to verify what a MATLAB function actually emits over USB before transcribing it to C++.


