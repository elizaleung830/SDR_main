# pupradar_capture — Linux ARM64 capture-only MVP

Single-binary tool for the **PUP EN24C T2R4** 24 GHz FMCW radar. Initializes the device, downloads firmware to the Cypress FX2LP, configures sawtooth FMCW, captures raw IQ for N seconds, and writes `<out>.bin` + `<out>.json`.

No MATLAB runtime on the target. Tested without hardware via the FakeUsbBackend test suite.

## Layout

```
linux/
├── CMakeLists.txt
├── toolchains/aarch64-linux-gnu.cmake     # cross-compile toolchain file
├── firmware/SDR_USB_FW.hex                # FX2LP firmware (copied from Matlab src/)
├── include/pupradar/                      # public headers
│   ├── Protocol.hpp                       # opcodes, command-packet builder, PLL math
│   ├── IntelHex.hpp                       # firmware HEX parser
│   ├── IUsbBackend.hpp                    # abstract USB interface (testable)
│   ├── UsbDevice.hpp                      # libusb-1.0 implementation
│   ├── RadarSession.hpp                   # init / configure / captureIq
│   └── IqWriter.hpp                       # raw .bin + JSON sidecar
├── src/                                   # implementations
├── tests/                                 # FakeUsbBackend + ~25 unit tests
└── scripts/
    ├── build_native.sh                    # host build + ctest
    ├── build_cross.sh                     # aarch64 cross build
    ├── deploy_pi.sh                       # scp to Pi
    └── 99-pupradar.rules                  # udev rule for non-root access
```

## Build & test on dev host (WSL / Linux)

Prereqs: `cmake >= 3.16`, `g++` with C++17, `libusb-1.0-0-dev`, `pkg-config`.

```bash
cd linux
./scripts/build_native.sh
```

This builds the executable for the host arch and runs the full test suite (`ctest --output-on-failure`). The Windows-side build path (CMake on MSVC) is not supported — use WSL.

## Cross-compile for Raspberry Pi 4/5 (Raspberry Pi OS 64-bit)

Prereqs in WSL/Debian:

```bash
sudo dpkg --add-architecture arm64
echo 'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy main universe' \
    | sudo tee /etc/apt/sources.list.d/arm64.list
sudo apt-get update
sudo apt-get install -y g++-aarch64-linux-gnu pkg-config libusb-1.0-0-dev:arm64
```

Then:

```bash
./scripts/build_cross.sh
file build-arm64/pupradar_capture   # → ELF 64-bit LSB executable, ARM aarch64
```

## Deploy and run on the Pi

```bash
./scripts/deploy_pi.sh pi@raspberrypi.local:~/pupradar
ssh pi@raspberrypi.local
sudo cp ~/pupradar/99-pupradar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ~/pupradar
./pupradar_capture --out /tmp/cap
```

## Discovering the post-firmware VID/PID (one-time)

The Cypress FX2LP boots with a default VID/PID of `04B4:8613`, then re-enumerates after firmware load with VID/PID defined inside `SDR_USB_FW.hex`. The MATLAB driver hides this behind a fresh per-call device open; libusb holds one persistent handle across the firmware download, so it has to wait for the real re-enumeration explicitly (see `RadarSession::initialize`).

The post-firmware VID/PID is a fixed constant in `main.cpp` (`post_fw`), matching every other radar parameter the MATLAB path doesn't expose. If a capture fails at the re-enumeration step, inspect:

```bash
lsusb
# Look for the new Cypress (vendor 04b4) entry that wasn't there before firmware load.
```

Then update the `post_fw` VID/PID constant in `main.cpp` to match, and edit `99-pupradar.rules` to match too, before rebuilding and reinstalling.

## CLI reference

Every option below is exactly a `makeRadarHandles()` parameter from the MATLAB
path — nothing else is adjustable. Operational details MATLAB doesn't expose
either (post-firmware VID/PID, re-enumeration timeout, command delay) are
fixed constants in `main.cpp`.

```
pupradar_capture --out <basename> [options]
  --sweep-time-index <1-5>  1=0.5ms 2=1ms 3=2ms 4=4ms 5=8ms (default 1)
  --transmitter <name>      Tx1 | Tx2 | Tx1.Tx2 (default Tx1)
  --receiver <name>         Rx1 | Rx2 | Rx3 | Rx4 | Rx1.Rx2 | Rx3.Rx4 |
                             Rx1.Rx2.Rx3.Rx4 (default Rx1)
  --sampling-index <1-4>    1=BASN, 2=BASN/2, 3=BASN/4, 4=BASN/8 (default 1).
                             Samples per sweep follow from the sweep time and
                             receiver count; they are not set directly.
  --num-sweeps <n>          Sweeps to acquire (default 128)
  --fc-low <Hz>              Chirp start frequency (default 24.0e9)
  --fc-high <Hz>             Chirp stop frequency (default 24.25e9)
  --firmware <path>          Path to SDR_USB_FW.hex (default ./firmware/SDR_USB_FW.hex)
  --cmd-delay-us <us>        Settling delay after each command packet (default 15000)
  -h, --help                 Show this help
```

`--cmd-delay-us` is the one knob that is not a `makeRadarHandles()` parameter — see
[USB model](#usb-model-one-claimed-interface-paced-commands) below. If a capture comes back with 0
sweep markers found, `pupradar_capture` prints a warning and the `.json` sidecar's
`sweep_markers_found` field reads `0`; raise `--cmd-delay-us` and re-capture.

## USB model: one claimed interface, paced commands

The board has **two** processors, and this matters:

- the **FX2LP USB chip**, which runs `SDR_USB_FW.hex`. The Cypress `0xA0` loader holds its
  8051 in reset while writing, so it starts clean on every run.
- a **separate radar MCU**, which holds the Tx/Rx/modulation/PLL state, tags sweeps, and
  streams samples. The firmware download does **not** reset it — it free-runs with
  whatever configuration it was last given until the board is power-cycled.

Every MATLAB MEX entry point (`miniradarputdata`, `miniradargetdata`, `usbdownload`,
`usbcheckchip`, `usbsetinterface1`, `usbfindendpoint`) constructs a fresh
`CCyUSBDevice(NULL)`, performs **exactly one** operation, and destroys it. There is no
persistent handle anywhere in the MATLAB stack. `Send_Basic_Parameter.m` and
`Send_PLL_Sawtooth.m` contain no `pause()` — all the spacing between the 22 configuration
words comes from that per-call open/enumerate/close cycle, which costs milliseconds to
tens of milliseconds each.

**That spacing is the only thing the per-call open/close actually buys**, and it is
load-bearing. An earlier version of this tool pushed all 22 words back-to-back in a few
hundred microseconds; the later words were silently dropped — **`E5` (Rx select) most
reliably of all**, being the last basic-parameter word and immediately followed by 17 PLL
writes. The symptom was that `--receiver Rx1` and `--receiver Rx4` produced identical
captures: the front end kept its previous channel while the EP6 data path, unaffected,
went on streaming live samples.

So `--cmd-delay-us` (default 15000) reproduces the gap explicitly, on **one** persistently
claimed interface:

| Operation | MATLAB equivalent | Here |
|---|---|---|
| claim + select alternate setting 1 | `usbsetinterface1` | **once**, during bring-up |
| each of the 22 config words | `miniradarputdata` | one 512 B bulk-OUT + `--cmd-delay-us` |
| board-info request / read | `miniradarputdata(FA00)` + `miniradargetdata` | bulk-OUT then bulk-IN |
| capture read | `miniradargetdata` | one logical transfer, split into ≤1 MiB chunks |
| firmware download | `usbdownload` | one handle for all the `0xA0` control transfers |

> **Do not** be tempted to open and close the device around each command to mimic the MEX
> more literally. `libusb_release_interface` issues a `SET_INTERFACE` request — *"resetting
> interface state to the first alternate setting"* — which flushes the FX2's EP2 FIFO and
> destroys the command word before the 8051 consumes it. CyAPI's `~CCyUSBDevice` does no
> such thing, which is why the same structure is safe in MATLAB and harmful here. The
> `Session_configure_does_not_disturb_the_interface` test guards this.

The capture read is chunked because libusb's synchronous API pins the whole request at
once and usbfs limits that (`/sys/module/usbcore/parameters/usbfs_memory_mb`, 16 MB by
default) — a ~50 MB capture asked for in one call returns `LIBUSB_ERROR_NO_MEM`.

## Output format

- `<out>.bin` — raw bulk-IN stream from EP6 (interleaved `int16` I/Q, little-endian).
  Includes the firmware-side framing/header bytes (≈2048-byte preamble per
  acquisition window per the GUI's deinterleave path). Parse with the same
  logic as `PUPradarGUI.m::GetComplexData` if you need calibrated samples.
- `<out>.json` — sidecar with frequencies, sweep time, sample rate per channel,
  channel masks, board info uint16 dump, ISO-8601 timestamp.

## Test approach (no hardware required)

`tests/FakeUsbBackend.hpp` records every USB call and replays scripted
responses for `controlTransfer` IN and `bulkRead`. Unit tests cover:

- Intel HEX parsing (valid, checksum failure, missing EOF, real first line)
- 512-byte command packet packing
- PLL register math (count, ordering, Start_N at 24.0 GHz)
- Sweep-time / sampling-number tables (verified against GUI `SetActiveParameters`)
- Tx/Rx mask validation
- Full session initialize sequence (open → claim → CPUCS → firmware → reopen → board info)
- Configure: opcodes in expected order, all 22 BulkOut commands
- captureIq: sinks bytes, returns coherent metadata
- IqWriter: bin size matches, JSON contains expected fields, escape characters

Run `ctest --output-on-failure` after `build_native.sh`.

## Known limitations / deferred

- CW mode (`--modulation cw`) — not implemented; `Send_PLL_CW` lives in MATLAB only.
- On-device DSP (FFT, range-Doppler) — explicitly out of scope per `PLAN.md`.
- Async libusb transfers — sync-only in MVP. Move to `libusb_submit_transfer` ring if sustained throughput proves insufficient.
- Windows host build — not supported. Use WSL.

See `progress/PROGRESS_*.md` for the implementation log.
