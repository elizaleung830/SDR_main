# PUP EN24C Radar — Linux/Raspberry Pi Port (Capture-Only MVP)

## Context

The PUP EN24C T2R4 24 GHz FMCW radar is currently usable only from a Windows MATLAB GUI ([PUPradarGUI.m](Matlab src/PUPradarGUI.m)) that wraps Cypress CyAPI through 7 Windows-only `.mexw64` MEX files. The goal is a **standalone Linux ARM64 executable** that runs on a Raspberry Pi 4/5 (Raspberry Pi OS 64-bit) **without any MATLAB runtime** on the target.

The first executable is intentionally minimal: initialize the radar, configure FMCW sawtooth mode from CLI parameters, capture raw IQ for a user-specified duration, and write the IQ stream to a binary file with a JSON sidecar describing the capture. No on-device DSP — downstream analysis stays in MATLAB/Python on a host. This MVP proves the toolchain end-to-end (firmware download → USB control plane → bulk IQ ingest → file output) before any signal-processing work is layered on.

The hardware path on Linux is not exotic: the Cypress FX3 enumerates as a standard USB device, so **libusb-1.0** replaces CyAPI cleanly. The MATLAB control surface is tiny (~6 functions, all parameter packing into USB control transfers), so a manual C++ port is faster and cleaner than MATLAB Coder for this domain.

## Decisions (confirmed)

| Question | Choice |
|---|---|
| Port approach | Manual C++ rewrite (no MATLAB Coder, no MCR) |
| Scope | Capture-only MVP — raw IQ out, no FFT/DSP |
| Target | Raspberry Pi 4/5, Raspberry Pi OS 64-bit (aarch64) |
| Build host | Cross-compile from WSL on the Windows dev machine |
| Output | Interleaved int16 IQ `.bin` + JSON sidecar |

## Architecture

```
+----------------------------------------------------+
|  pupradar_capture (single C++17 ARM64 executable)  |
+----------------------------------------------------+
|  CLI (argparse-style)                              |
|  └─ mode, duration, fc, BW, Fs, Tx/Rx, out path    |
|                                                    |
|  RadarSession  (high-level facade)                 |
|  ├─ init()          ← was PUPradar_initiating      |
|  ├─ configureFMCW() ← was Send_Basic_Parameter +   |
|  │                       Send_PLL_Sawtooth         |
|  ├─ setActive()     ← was SetActiveParameters      |
|  ├─ captureIQ(dur)  ← was GetComplexData (looped)  |
|  └─ shutdown()                                     |
|                                                    |
|  UsbDevice  (libusb-1.0 wrapper)                   |
|  ├─ openByVidPid()    ← was usbcheckchip           |
|  ├─ downloadFirmware()← was usbdownload (ReqCode   |
|  │                       0xA0, FX3 RAM boot)       |
|  ├─ claimInterface()  ← was usbsetinterface{0,1}   |
|  ├─ controlTransfer()                              |
|  └─ bulkRead()        ← was miniradargetdata       |
|                                                    |
|  IqWriter (raw int16 + JSON sidecar)               |
+----------------------------------------------------+
        │ libusb-1.0           │ nlohmann/json (header-only)
        ▼                      ▼
+----------------------------------------------------+
|  Raspberry Pi OS 64-bit (aarch64) — USB 3.0 host   |
|         ↓ USB                                      |
|  PUP EN24C radar (Cypress FX3)                     |
+----------------------------------------------------+
```

**Key design points**

- **One static-ish binary**, dynamically linked only against libusb-1.0 and libc/libstdc++ — easy to deploy with `scp`.
- **Firmware loaded at startup** using the Cypress 8051 RAM-boot protocol (control transfer, `bmRequestType=0x40`, `bRequest=0xA0`, address in `wValue`, payload from [SDR_USB_FW.hex](Matlab src/SDR_USB_FW.hex) parsed as Intel HEX). Replaces [usbdownload.cpp](Mex src/usbdownload.cpp).
- **Capture loop** uses `libusb_bulk_transfer` (synchronous in MVP; async/zero-copy is a later optimization if throughput is a problem). The current MEX [miniradargetdata.cpp](Mex src/miniradargetdata.cpp) is the reference for endpoint, packet size, and timeout.
- **CLI-only** — no GUI, no live plotting. The PUPradarGUI.fig and ~138 KB of GUIDE callbacks are discarded.

## Repository layout (proposed)

```
SDR_main/
├── CLAUDE.md                       ← project context for Claude (NEW)
├── docs/                           ← existing PDFs (keep)
├── Matlab src/                     ← existing reference (read-only from now on)
├── Mex src/                        ← existing reference (read-only from now on)
└── linux/                          ← NEW — all new work lives here
    ├── CMakeLists.txt
    ├── toolchains/
    │   └── aarch64-linux-gnu.cmake ← cross-compile toolchain file
    ├── third_party/
    │   └── nlohmann_json.hpp       ← header-only
    ├── firmware/
    │   └── SDR_USB_FW.hex          ← copy of Matlab src/SDR_USB_FW.hex
    ├── include/pupradar/
    │   ├── UsbDevice.hpp
    │   ├── RadarSession.hpp
    │   ├── IqWriter.hpp
    │   └── Protocol.hpp            ← VID/PID, endpoints, opcodes, packet structs
    ├── src/
    │   ├── main.cpp                ← CLI + orchestration
    │   ├── UsbDevice.cpp           ← libusb-1.0 wrapper
    │   ├── RadarSession.cpp        ← ports of the 6 MATLAB API funcs
    │   ├── IqWriter.cpp
    │   ├── IntelHex.cpp            ← .hex parser for firmware download
    │   └── Protocol.cpp            ← byte-packing for control transfers
    ├── scripts/
    │   ├── build_wsl.sh            ← one-shot cross-compile
    │   └── deploy_pi.sh            ← scp + ssh chmod
    └── README.md
```

## Implementation phases

### Phase 0 — Knowledge capture (no code yet)
1. Write [CLAUDE.md](CLAUDE.md) at project root: hardware summary, why MATLAB exists, why the Linux port, where the canonical references live, and a note that `Matlab src/` and `Mex src/` are read-only references for porting (not build inputs).
2. Read [PupRadar_Protocol.docx](Mex src/PupRadar_Protocol.docx) and [PupRadar_DataFormat.docx](Mex src/PupRadar_DataFormat.docx) — these likely document the on-the-wire control opcodes and IQ packet format. **If these are accurate, the C++ port can be derived directly from them; the MATLAB code becomes secondary.** If they're missing details, fall back to reverse-engineering from MATLAB + MEX.
3. Record VID/PID, endpoint addresses, and the firmware download sequence by reading [usbcheckchip.cpp](Mex src/usbcheckchip.cpp), [usbdownload.cpp](Mex src/usbdownload.cpp), [usbfindendpoint.cpp](Mex src/usbfindendpoint.cpp), [miniradargetdata.cpp](Mex src/miniradargetdata.cpp), [miniradarputdata.cpp](Mex src/miniradarputdata.cpp), [usbsetinterface1.cpp](Mex src/usbsetinterface1.cpp), and the corresponding call sites in [PUPradarGUI.m](Matlab src/PUPradarGUI.m) (`PUPradar_initiating`, `Send_Basic_Parameter`, `Send_PLL_Sawtooth`, `SetActiveParameters`, `GetComplexData`).

### Phase 1 — Build skeleton
4. Create the `linux/` tree with CMake + aarch64 toolchain file. Verify "hello world" cross-compiles in WSL and runs on the Pi (`uname -m` → `aarch64`).
5. Install `libusb-1.0` headers/libs in the WSL sysroot and link a stub program that calls `libusb_init` / `libusb_get_device_list` and runs on the Pi.

### Phase 2 — USB layer (`UsbDevice`)
6. Port `usbcheckchip` → `UsbDevice::openByVidPid()` using `libusb_get_device_list` + `libusb_get_device_descriptor`. VID/PID come from Phase 0.
7. Port `usbdownload` → `UsbDevice::downloadFirmware()`. Implement Intel HEX parser, then issue `libusb_control_transfer(0x40, 0xA0, address, 0, payload, len, timeout)` per record.
8. Port `usbsetinterface{0,1}` → `claimInterface(idx)` via `libusb_claim_interface` / `libusb_set_interface_alt_setting`.
9. Port `usbfindendpoint` → endpoint discovery via `libusb_get_active_config_descriptor`.
10. Port `miniradarputdata` / `miniradargetdata` → `controlTransfer()` and `bulkRead()`.

### Phase 3 — Radar control (`RadarSession`)
11. Port `PUPradar_initiating` — full bring-up: enumerate → firmware download → re-enumerate (FX3 typically re-enumerates after RAM boot) → claim interface → discover endpoints.
12. Port `Send_Basic_Parameter`, `Send_PLL_Sawtooth`, `SetActiveParameters` — these are byte-packing into control transfers; transcribe the MATLAB byte arrays into C++ literals/structs in `Protocol.cpp`. CW path (`Send_PLL_CW`) deferred — capture-only MVP uses sawtooth.
13. Port `GetComplexData` into `captureIQ(duration_seconds)` — bulk-read loop accumulating samples until duration elapses.

### Phase 4 — I/O and CLI
14. `IqWriter`: writes interleaved `int16` I/Q to `<out>.bin`, writes `<out>.json` with `{fc_hz, bw_hz, fs_hz, n_chirps, n_samples_per_chirp, tx_mask, rx_mask, duration_s, timestamp_utc, firmware_sha256}`.
15. `main.cpp`: CLI flags (`--fc`, `--bw`, `--fs`, `--duration`, `--tx`, `--rx`, `--out`), with sane defaults from the datasheet.

### Phase 5 — Verification on Pi
16. Deploy to Pi via `scripts/deploy_pi.sh`. Add `udev` rule so the binary doesn't need root.
17. Functional test: 5-second sawtooth capture against a known target (corner reflector at fixed range), then load `.bin` in MATLAB on the Windows host using the existing GUI's data path and confirm range bin matches expectation.

## Critical files to read before any porting (Phase 0)

| File | Why |
|---|---|
| [PupRadar_Protocol.docx](Mex src/PupRadar_Protocol.docx) | Authoritative wire protocol — read first |
| [PupRadar_DataFormat.docx](Mex src/PupRadar_DataFormat.docx) | IQ packet layout |
| [PUPradarGUI.m](Matlab src/PUPradarGUI.m) | The 6 API functions listed above are the porting targets |
| [usbdownload.cpp](Mex src/usbdownload.cpp) | Firmware download = highest-risk porting step |
| [miniradargetdata.cpp](Mex src/miniradargetdata.cpp) | IQ ingest path — endpoint, packet size, timeout |
| [usbcheckchip.cpp](Mex src/usbcheckchip.cpp) | VID/PID + enumeration logic |
| [PUP_EN24C_T2R4_Manual.pdf](docs/PUP_EN24C_T2R4_Manual.pdf), [PUP_EN24C_T2R4_datasheet.pdf](docs/PUP_EN24C_T2R4_datasheet.pdf) | RF parameter ranges, sane defaults |

## Open risks (flagged early)

1. **Firmware re-enumeration timing.** After `0xA0` RAM boot, the FX3 typically detaches and re-enumerates with a different (or same) VID/PID. The Linux code needs to wait + re-open. Windows CyAPI hides this; libusb does not.
2. **udev permissions.** Out of the box, non-root users can't claim a USB device. A udev rule (`/etc/udev/rules.d/99-pupradar.rules`) granting `MODE="0666"` for the radar's VID/PID is required — included in deploy script.
3. **Bulk-transfer throughput on Pi USB stack.** Pi 4/5 expose USB 3.0, but the kernel USB stack has historically been finicky under sustained bulk loads. If we hit overruns at the radar's max sample rate, we move from sync to async (`libusb_submit_transfer` with a ring of in-flight transfers). MVP starts sync.
4. **The `.docx` protocol files may be incomplete.** If they don't fully document opcodes, Phase 0 expands to careful read of MATLAB + MEX call sites.
5. **MATLAB R2010b-era MEX assumptions.** The MEX code may rely on quirks (column-major arrays, `mxArray` lifetime) that don't survive translation; we're rewriting the API surface, not transliterating MEX, so this is mostly avoidable.

## Verification plan

End-to-end success criteria for the MVP:

- [ ] Cross-compile in WSL produces an `aarch64` ELF (`file pupradar_capture` reports ARM aarch64).
- [ ] Binary runs on Pi 4/5 64-bit and prints a valid `--help`.
- [ ] With radar plugged in, `pupradar_capture --duration 5 --out /tmp/cap` exits 0, produces `/tmp/cap.bin` and `/tmp/cap.json`.
- [ ] `cap.bin` size matches expectation: `Fs × duration × 2 (I,Q) × n_rx × 2 bytes` ± framing overhead.
- [ ] Loading `cap.bin` in MATLAB on the Windows host (using the existing GUI's deinterleave path) shows the expected IQ envelope from a known target (e.g., corner reflector).
- [ ] Re-running 10× back-to-back succeeds (no zombie USB handles, no kernel `dmesg` errors).

## Out of scope for this plan (explicitly deferred)

- Range FFT, range-Doppler, CFAR, any DSP.
- CW mode (`Send_PLL_CW`).
- A GUI of any kind.
- Real-time streaming over network.
- MATLAB Coder integration (revisit only if/when we add nontrivial DSP from `.m`).
