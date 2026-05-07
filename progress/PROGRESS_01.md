# PROGRESS_01 — Phase 0: Protocol Knowledge Capture

**Date:** 2026-05-06
**Phase:** 0 (Knowledge capture, no code yet)

## What I read

| File | Purpose |
|---|---|
| [usbcheckchip.cpp](../Mex src/usbcheckchip.cpp) | Device enumeration (VID/PID/config) |
| [usbdownload.cpp](../Mex src/usbdownload.cpp) | FX2LP 8051 firmware loader (0xA0) |
| [usbsetinterface1.cpp](../Mex src/usbsetinterface1.cpp) | SetAltIntfc(1) on interface 0 |
| [usbfindendpoint.cpp](../Mex src/usbfindendpoint.cpp) | Endpoint discovery by address |
| [miniradargetdata.cpp](../Mex src/miniradargetdata.cpp) | Bulk-IN read |
| [miniradarputdata.cpp](../Mex src/miniradarputdata.cpp) | Bulk-OUT write (fixed 512 bytes!) |
| [PUPradarGUI.m](../Matlab src/PUPradarGUI.m) | API surface + protocol opcodes |
| [SDR_USB_FW.hex](../Matlab src/SDR_USB_FW.hex) | 15-line Intel HEX firmware |

## Critical findings

### 1. Chip is FX2LP, not FX3 (correction to PLAN.md)

CPUCS register at address `0xE600` is the **Cypress FX2LP** signature. FX3 uses different addresses entirely. This means:
- USB 2.0 high-speed (480 Mbps), not USB 3.0 SuperSpeed.
- 8051 microcontroller (small RAM, downloaded firmware).
- All Pi models with USB 2.0+ are sufficient (USB 3.0 not strictly required).

### 2. Pre-firmware VID/PID

Default unprogrammed FX2LP: **VID = 0x04B4 (decimal 1204), PID = 0x8613 (decimal 34323)**. Confirmed in `PUPradar_initiating` (line 2615 of GUI):
```matlab
if (vID~=1204) || (pID~=34323) error
```

### 3. ⚠ Post-firmware VID/PID is UNKNOWN

CyAPI silently re-binds after the chip re-enumerates. The MATLAB code does not list the post-firmware VID/PID anywhere — CyAPI's `CCyUSBDevice(NULL)` just opens "any Cypress device" each call. With libusb we MUST know the post-firmware VID/PID.

**Mitigation:** expose post-firmware VID/PID as CLI flags with placeholder defaults; the user runs `lsusb` after the first firmware load to read the actual values, then passes them in. Document this clearly in README. Alternatively, we can add a "scan all USB devices and find one whose VID matches 0x04B4 and PID changed" heuristic.

### 4. Firmware download = standard Cypress 0xA0 protocol

```
Step 1: control read  bmRequestType=0xC0, bRequest=0xA0, wValue=0xE600, wIndex=0, len=1
        → reads CPUCS register
Step 2: cpucs |= 0x01 (set 8051 reset)
Step 3: control write bmRequestType=0x40, bRequest=0xA0, wValue=0xE600, wIndex=0, data=[cpucs]
Step 4: For each Intel HEX data record (type 00):
        control write bmRequestType=0x40, bRequest=0xA0,
                       wValue=record_address, wIndex=0,
                       data=record_bytes, len=record_len
Step 5: cpucs &= 0xFE (clear 8051 reset → start firmware execution)
Step 6: control write same as step 3 with new cpucs
        → chip now re-enumerates with firmware-defined VID/PID
```

### 5. Wire protocol — outbound commands (host → MCU)

**All commands** are sent as **fixed 512-byte bulk writes** to **EP2 OUT (address 0x02)**. The MEX hardcodes `outlength=512` regardless of input buffer size. The MCU command parser reads the **first uint16 little-endian** and discards the rest.

Each command word = `(opcode_byte << 8) | parameter_byte`:

| Opcode | Meaning | Parameter |
|---|---|---|
| 0xFA | Request board info | 0x00 (no param; response on EP6 IN) |
| 0xE1 | Set modulation | 0=Sawtooth, 3=CW |
| 0xE2 | Set sweep-time index | 1..5 = 0.5/1/2/4/8 ms |
| 0xE3 | Set sampling-number index | 1..4 (table indexed by Rx + sweep time) |
| 0xE4 | Set Tx mask | 1=Tx1, 2=Tx2, 3=Tx1+Tx2 |
| 0xE5 | Set Rx mask | 1=Rx1, 2=Rx2, 4=Rx3, 8=Rx4, 3=Rx12, 12=Rx34, 15=All |
| 0xC1/C2/C3 | PLL Reg 0x03 (start_N integer) H/M/L bytes | 0..0xFF |
| 0xC4/C5/C6 | PLL Reg 0x04 (start_N fraction) H/M/L bytes | 0..0xFF |
| 0xC7/C8/C9 | PLL Reg 0x0A (step size) H/M/L bytes | 0..0xFF |
| 0xCA/CB/CC | PLL Reg 0x0C (stop integer) H/M/L bytes | 0..0xFF |
| 0xCD/CE/CF | PLL Reg 0x0D (stop fraction) H/M/L bytes | 0..0xFF |
| 0xD1/D2 | PLL sweep-stop counter H/L bytes | 0..0xFF |

**PLL math (sawtooth):**
- `T_ref = 1/50 MHz = 20 ns`
- `F_PLLinput = F_Tx / 16` (BGT24 mixer ratio)
- `Start_N = (F_low / 16) / 50e6`, integer + fractional×2^24
- `Stop_N` analogous from F_high
- Step counter, stop registers derived (see [PUPradarGUI.m:2438..2602](../Matlab src/PUPradarGUI.m#L2438))

### 6. Wire protocol — inbound (MCU → host)

**EP6 IN (address 0x86)** bulk endpoint. MCU continuously streams once configured. Two read patterns:

**Board-info read** (after sending 0xFA00):
- Read 2560 bytes (= 512 hdr + 2048 payload).
- First 2048 bytes appear to be padding/echo; useful info starts at byte 2048.
- Magic 0xFA05 at offset 2048..2049 (little-endian uint16).
- Next bytes encode FrequencyBand, Num_Tx, Num_Rx, AntennaType, Version → modelcode.

**IQ data read** (continuous after configuration):
- `DataLength = ceil((NumSweeps+40) × LASN × 2 × LANR × LANT / 512) × 512 + 4096` bytes per read
- NumSweeps = 64 (in GUI's run loop)
- LASN = active sampling number per sweep
- LANR = number of active Rx
- LANT = number of active Tx
- After read, MATLAB discards the leading 2048 bytes (header) and trailing padding.
- IQ samples are interleaved `int16` (likely signed offset-binary; needs hardware confirmation).

**Important:** there is **no separate "start capture" command.** The radar streams continuously once configured. The host triggers acquisition simply by issuing the bulk-IN read.

### 7. Initialization sequence (`PUPradar_initiating`)

```
1. Enumerate, expect 1 device with VID=0x04B4 PID=0x8613
2. Claim interface 0, alt setting 1
3. Verify endpoints: EP2 OUT (0x02), EP6 IN (0x86)
4. Parse SDR_USB_FW.hex (Intel HEX format)
5. Issue 0xA0 firmware download (steps 1-6 from §4)
6. (Chip re-enumerates with firmware VID/PID — UNKNOWN, see §3)
7. Re-open device with new VID/PID
8. Re-claim interface 0 alt 1
9. Send 0xFA00 (request board info)
10. Read 2560 bytes from EP6 IN
11. Parse board info to confirm model
```

### 8. Endpoint addressing nuance

In MATLAB: `usbfindendpoint(2)` and `usbfindendpoint(134)`. CyAPI's `EndPoints[i]->Address` returns the **USB endpoint address byte** (with direction bit). 134 = 0x86 = EP6 IN. The MEX returns the **array index** within `CCyUSBDevice::EndPoints[]`, not the address — but for libusb, we only need the address byte directly: `0x02` for EP2 OUT, `0x86` for EP6 IN. No discovery loop required if we hardcode (the firmware always exposes these).

### 9. Sample rate and capture sizing

The user-visible sample rate per channel = `LASN / LAST` (samples per sweep / sweep time). For the largest standard config (8 ms sweep, 16384 samples, Rx=1): 2.048 MS/s per channel. With 4 Rx + 2 Tx, aggregate IQ bytes ≈ 32 MB/s — well within USB 2.0 high-speed (480 Mbps ≈ 60 MB/s practical).

For a 5-second capture at default 1 ms / 1024-sample / 4-Rx / 1-Tx: ≈ 5 × 1024 × 4 × 1 × 2 (I,Q) × 2 bytes / 1ms = 80 MB total.

## Decisions locked from this knowledge

1. **Implement Intel HEX parser in C++** (not in MATLAB-style cell-array approach). Validate record type and checksum. Parse records of type 00 (data), 01 (EOF), ignore others.
2. **Pad all OUT bulk writes to 512 bytes** with the command word repeated, matching the original MEX behavior. The MCU only inspects the first uint16, but the FX2LP bulk endpoint expects full max-packet writes for proper framing.
3. **Hardcode endpoints** EP2_OUT = 0x02, EP6_IN = 0x86. No discovery.
4. **Pre-firmware VID/PID** = 0x04B4 / 0x8613 (constant).
5. **Post-firmware VID/PID** = CLI flag, default `0x04B4/0x1004` placeholder, with documented `lsusb` workaround. We also implement a "find any new Cypress device" fallback that scans VID 0x04B4 with any PID ≠ 0x8613 after a short re-enumeration delay.
6. **No board-info parse in the MVP runtime.** Just send 0xFA00, read 2560 bytes, log the modelcode bytes verbatim into the JSON sidecar for the user to verify. Saves complexity.
7. **No CW mode.** Sawtooth only.
8. **No DSP, no live plot, no continuous-record file rollover.** One file per run.

## Risks updated

| Risk | Severity | Mitigation |
|---|---|---|
| Post-firmware VID/PID unknown | **High** | CLI flag + scan fallback; document lsusb step |
| Re-enumeration timing varies | Medium | Poll for new device with 5 s timeout, 100 ms poll |
| Endianness of int16 IQ on Pi (ARM) | Low | x86 and aarch64 are both little-endian; FX2LP is little-endian; no swap needed |
| udev permission for non-root | Low | Ship udev rule in deploy script |
| Overrun under sustained bulk read | Medium | Start sync; switch to async ring if needed; provide `--bulk-timeout-ms` flag |
| Intel HEX checksum bug in upstream firmware file | Low | We validate checksums; if any record fails, we abort firmware download with a clear error |

## Next phase

Phase 1 — build skeleton: `linux/` tree, CMake with both native (host) and cross-compile (aarch64) targets, libusb-1.0 dependency, simple "hello USB" smoke test that runs on the host.
