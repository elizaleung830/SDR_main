# PROGRESS_03 — Final report and hardware bring-up runbook

**Date:** 2026-05-06
**Status:** Implementation complete. 25/25 unit tests pass on x86_64 host without hardware. Cross-compile and libusb integration require one-time toolchain installation in WSL (instructions in README + this doc). Hardware verification on the Pi is the next step and requires you.

## What's done

✅ **Phase 0** — protocol extracted from MEX + MATLAB sources; documented in PROGRESS_01.md
✅ **Phase 1** — `linux/` tree, CMake, aarch64 toolchain file
✅ **Phase 2** — Protocol packet builder, PLL math, Intel HEX parser, UsbDevice (libusb-1.0), abstract `IUsbBackend`
✅ **Phase 3** — RadarSession (init / configure / captureIq / shutdown)
✅ **Phase 4** — IqWriter (raw .bin + JSON sidecar), CLI in main.cpp
✅ **Tests** — 25 unit tests covering hex parser, protocol, session, and writer; all pass

⏳ **Phase 5 (hardware)** — yours to run; runbook below.

## Test results

```
25 tests, 0 failed.
```

Coverage matrix:

| Component | Tests | Notes |
|---|---|---|
| `IntelHex` | 8 | minimal EOF, 1 record, checksum failure, missing EOF, missing colon, real first line of `SDR_USB_FW.hex`, lowercase, CR/LF |
| `Protocol` | 11 | packet size, little-endian word repeat, sweep-time table, samples-per-sweep table (cross-checked vs GUI), Tx mask validation, Rx mask validation, PLL command count + ordering + Start_N at 24.0 GHz, rejects inverted band |
| `RadarSession` | 4 | full init sequence (open → claim → CPUCS → firmware → reopen → board-info), configure opcode order (22 BulkOuts), captureIq sinks bytes + returns metadata, rejects CW |
| `IqWriter` | 2 | bin file size matches, JSON contains required fields, backslash escape works |

## Hardware bring-up runbook

### 0) One-time WSL setup (dev machine)

```bash
sudo dpkg --add-architecture arm64
echo 'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy main universe' \
    | sudo tee /etc/apt/sources.list.d/arm64.list
sudo apt-get update
sudo apt-get install -y \
    cmake pkg-config \
    libusb-1.0-0-dev \
    g++-aarch64-linux-gnu libusb-1.0-0-dev:arm64
```

### 1) Verify host build + tests still green

```bash
cd linux
./scripts/build_native.sh
# Expect: 25/25 tests pass, build-native/pupradar_capture present
```

### 2) Cross-compile for the Pi

```bash
./scripts/build_cross.sh
file build-arm64/pupradar_capture
# Expect: ELF 64-bit LSB executable, ARM aarch64
```

### 3) Deploy

```bash
./scripts/deploy_pi.sh pi@raspberrypi.local:~/pupradar
ssh pi@raspberrypi.local
sudo cp ~/pupradar/99-pupradar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4) First-light test (expected to surface the post-firmware VID/PID)

Plug the radar into the Pi, then:

```bash
lsusb | grep "04b4"        # should show 04b4:8613 (unprogrammed FX2LP)
cd ~/pupradar
./pupradar_capture --duration 2 --out /tmp/cap0
```

**Expected outcomes:**

- ✅ **Best case:** the binary completes, writes `cap0.bin` and `cap0.json`. Means the firmware loads, the chip re-enumerates with our placeholder PID `0x1004`, and the bulk read works. (Unlikely on first try — placeholder PID is a guess.)

- ⚠ **Likely case:** binary fails at `[pupradar] initializing` with `reopenAfterRenumeration: device did not reappear in time`. **This is the diagnostic step.** Now run:
  ```bash
  lsusb | grep "04b4"
  ```
  You should see a *new* line — same vendor (04b4), different product ID. That's the post-firmware PID. Note it (e.g., `04b4:8613` → `04b4:00f1`).

  Then add this PID to `99-pupradar.rules`:
  ```bash
  sudo sed -i 's/04b4.*1004/04b4", ATTR{idProduct}=="00f1/' /etc/udev/rules.d/99-pupradar.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
  And re-run with the discovered PID:
  ```bash
  ./pupradar_capture --post-fw-pid 0x00f1 --duration 2 --out /tmp/cap1
  ```

- 🚨 **If `lsusb` shows nothing** after running the binary: firmware download itself failed. Check `dmesg | tail -20` for USB errors. Verify the udev rule covers `04b4:8613` (yes by default). If running as user without the udev rule applied, you may see permission errors.

### 5) Validate output bytes

```bash
ls -la /tmp/cap1.*
cat /tmp/cap1.json
xxd /tmp/cap1.bin | head -5
```

**Sanity checks:**

- `bytes_captured` in JSON should be roughly `(duration_s × bytes_per_read × reads / s)`. For default config (1 ms sweep, 2048 samples, 1 Tx, 1 Rx): each bulk read is ~856 KB and the radar streams 64 sweeps per ~64 ms → ~13 MB/s → ~26 MB for 2 s.
- `cap1.bin` size should match `bytes_captured` exactly.
- First 16 bytes of `.bin` will be the firmware-side preamble, not actual IQ. The GUI's `GetComplexData` strips 2048 bytes before each acquisition window — our raw dump preserves that.

### 6) Cross-check against MATLAB

Copy `cap1.bin` back to the Windows host. In MATLAB, load and deinterleave using the same logic as `PUPradarGUI.m::GetComplexData` (lines 2731–2734):

```matlab
fid = fopen('cap1.bin', 'rb');
raw = fread(fid, inf, 'int16');
fclose(fid);
% Strip leading 2048 samples (= 1024 int16 = 4096 bytes? double-check on your file)
raw = raw(2049:end);
% Then proceed with the GUI's deinterleave path.
```

If the IQ envelope from a known target (e.g., a corner reflector at fixed range) shows the expected peak, **you've succeeded.**

### 7) Stress test

```bash
for i in $(seq 1 10); do
  ./pupradar_capture --duration 5 --out /tmp/loop$i \
                     --post-fw-pid 0x00f1
  sleep 1
done
ls -la /tmp/loop*.bin   # all sizes should match (within USB jitter)
dmesg | tail            # no USB errors
```

This catches USB handle leaks and re-enumeration races.

## Risks still open until hardware test

| Risk | Probability | Mitigation if hit |
|---|---|---|
| Post-firmware VID/PID guess is wrong | **High** | Discover via `lsusb`, pass with `--post-fw-pid` (planned for) |
| `setAltSetting` on unprogrammed FX2LP fails | Medium | Already wrapped in try/catch — initialize() ignores the error pre-firmware |
| Re-enumeration takes >5s on slow USB hubs | Medium | `--reenum-timeout-ms 10000` |
| Bulk-read timeout too short | Medium | `--bulk-timeout-ms 5000` |
| PLL math off by one bit somewhere | Medium | Use a spectrum analyzer or compare radar response to MATLAB GUI side-by-side |
| Throughput inadequate (sync libusb) | Low/Medium | Switch to async (`libusb_submit_transfer` ring) — work doc'd in PLAN.md |
| 2048-byte preamble offset varies | Low | Confirm against the MATLAB strip path; adjust documentation if needed |

## Files written this session

```
progress/
├── PROGRESS_01.md           ← protocol knowledge capture
├── PROGRESS_02.md           ← skeleton + implementation
└── PROGRESS_03.md           ← this file

linux/                        ← entire tree (~1,200 LOC, 0 third-party runtime deps beyond libusb)
```

`Matlab src/` and `Mex src/` were NOT modified (rule from CLAUDE.md respected).

## Recommended next actions for the user

1. Run the WSL one-time apt install (§0 above).
2. Re-run `scripts/build_native.sh` to confirm 25/25 still green with the libusb backend in the build.
3. Run `scripts/build_cross.sh` and confirm `file build-arm64/pupradar_capture` reports aarch64.
4. Deploy and execute §4 of the runbook.
5. Report back with the post-firmware VID/PID from `lsusb` so we can lock it in as the default.

## Out of scope (deferred per PLAN.md)

- DSP (FFT, range-Doppler, CFAR)
- CW mode
- Async libusb transfers
- GUI / live plotting
- Real-time network streaming
- MATLAB Coder integration
