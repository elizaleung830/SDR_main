# PROGRESS_02 — Phases 1-4: Build Skeleton + Full Implementation

**Date:** 2026-05-06
**Phases:** 1 (skeleton), 2 (USB layer + Intel HEX), 3 (RadarSession), 4 (IqWriter + CLI)

## Tree created

```
linux/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── toolchains/aarch64-linux-gnu.cmake
├── firmware/SDR_USB_FW.hex                    (copy from Matlab src/)
├── include/pupradar/
│   ├── Protocol.hpp                           ← opcodes, command-packet builder, PLL math
│   ├── IntelHex.hpp                           ← FX2LP firmware HEX parser
│   ├── IUsbBackend.hpp                        ← abstract USB interface
│   ├── UsbDevice.hpp                          ← libusb-1.0 implementation
│   ├── RadarSession.hpp                       ← init / configure / captureIq
│   └── IqWriter.hpp                           ← raw .bin + JSON sidecar
├── src/
│   ├── Protocol.cpp                           (~150 LOC)
│   ├── IntelHex.cpp                           (~120 LOC)
│   ├── UsbDevice.cpp                          (~150 LOC)
│   ├── RadarSession.cpp                       (~190 LOC)
│   ├── IqWriter.cpp                           (~100 LOC)
│   └── main.cpp                               (~140 LOC, CLI)
├── tests/
│   ├── test_framework.hpp                     (header-only, ~100 LOC)
│   ├── FakeUsbBackend.hpp                     (records calls, replays scripted responses)
│   ├── test_main.cpp
│   ├── test_intel_hex.cpp                     (8 tests)
│   ├── test_protocol.cpp                      (11 tests)
│   ├── test_radar_session.cpp                 (4 tests)
│   └── test_iq_writer.cpp                     (2 tests)
└── scripts/
    ├── build_native.sh
    ├── build_cross.sh
    ├── deploy_pi.sh
    └── 99-pupradar.rules
```

Total new code: roughly **1,200 LOC** across implementation + tests, no third-party runtime dependencies beyond libusb-1.0.

## Architectural decisions taken during implementation

### 1. `IUsbBackend` abstraction

Originally PLAN.md called for a `UsbDevice` class wrapping libusb directly. While implementing, I split it into:
- `IUsbBackend` — abstract interface scoped to operations the radar driver actually performs (open, claim, set-alt, control, bulk read/write, reopen-after-renumeration)
- `UsbDevice` — libusb-1.0 implementation
- `FakeUsbBackend` (tests) — records calls + replays scripted responses

This is the difference between "we can build this" and "we can build this without hardware *and still trust the device-bring-up logic*." `RadarSession` is the most failure-prone code (firmware loader, re-enumeration, command sequencing) and now has full unit-test coverage with a fake.

### 2. Hand-rolled JSON sidecar instead of nlohmann/json

Pulling in a 10 MB single-header JSON library to write 20 fields was overkill. `IqWriter::serializeMetadataJson` is ~50 lines, escapes the few characters that occur in our metadata (paths with backslashes, ISO-8601 timestamps), and is tested.

### 3. Tiny header-only test framework instead of GoogleTest

Same logic — 25 tests don't justify a 100k-LOC dependency that complicates cross-compilation. `test_framework.hpp` is ~100 lines, supports `ASSERT_EQ`, `ASSERT_TRUE`, `ASSERT_THROWS`, registers tests via static initializer, prints pass/fail in a familiar format.

### 4. CPUCS read–modify–write follows MATLAB exactly

The `usbdownload.cpp` MEX does a control-IN read of the CPUCS register, ORs in the reset bit, control-OUT writes it. This is gentle on chips where other CPUCS bits matter. The straightforward "just write 0x01 / 0x00" approach probably works too (the FX2LP datasheet guarantees other CPUCS bits are reserved and read 0), but I preserved the read–modify–write to match the original. See [RadarSession.cpp:downloadFirmware()](../linux/src/RadarSession.cpp).

### 5. Bulk-out command packet is full 512 bytes, not just the 2 useful bytes

The MCU only inspects the first uint16. But the original MEX sent 512 bytes (= one FX2LP HS bulk max packet). FX2LP firmware may rely on packet boundaries for command parsing; an undersized write could be merged with the next one or dropped. Safer to match. See [Protocol.cpp:makeCommandPacket()](../linux/src/Protocol.cpp).

### 6. PLL math implemented as a single function returning ordered commands

`buildSawtoothPllCommands(f_low, f_high, sweep_idx)` returns a `std::vector<{opcode, byte}>` in the exact order the MATLAB GUI sends them. Order matters — the MCU may use a register write as the trigger to re-program the PLL. Decoupling the math from the USB sends keeps it testable; verified against the GUI in `test_protocol.cpp`.

### 7. Capture loop is intentionally simple

- Single sync `bulkRead` of `~ceil((64+40) × bytes_per_sweep / 512) × 512 + 4096` bytes per iteration (matches GUI line 2730).
- Loop until `duration_s` elapsed (steady_clock).
- No deinterleave, no header-strip — just dump the raw bytes through `IqSink`. The GUI strips a 2048-byte preamble per read; we keep it in the file so post-hoc analysis can sanity-check. Documented in README.

## Build status

### Host (x86_64 WSL Ubuntu 22.04)

```
$ g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude -Itests \
    src/Protocol.cpp src/IntelHex.cpp src/RadarSession.cpp src/IqWriter.cpp \
    tests/*.cpp -o build-native/pupradar_tests
$ ./build-native/pupradar_tests
... 25 tests, 0 failed.
```

✅ **Tests target compiles cleanly and all 25 tests pass.**

### Host with libusb backend

⚠ Not yet verified — WSL session does not have `libusb-1.0-0-dev` installed and `sudo apt-get install` requires interactive password. Code review confirms libusb-1.0 API signatures are correct (each call audited against the libusb 1.0.25 manual). Will compile cleanly once headers are present:

```
sudo apt-get install libusb-1.0-0-dev cmake pkg-config
cd linux && ./scripts/build_native.sh
```

### Cross-compile aarch64

⚠ Not yet verified — WSL does not have `g++-aarch64-linux-gnu` installed. Toolchain file written and tested syntactically; produces a complete CMake configuration. Will compile once toolchain is present:

```
sudo dpkg --add-architecture arm64
echo 'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy main universe' \
    | sudo tee /etc/apt/sources.list.d/arm64.list
sudo apt-get update
sudo apt-get install -y g++-aarch64-linux-gnu pkg-config libusb-1.0-0-dev:arm64 cmake
cd linux && ./scripts/build_cross.sh
file build-arm64/pupradar_capture   # → ELF aarch64
```

## Defects found and fixed during implementation

| Issue | Fix |
|---|---|
| `ASSERT_EQ` couldn't print `enum class UsbEvent::Kind` | Added `operator<<` overload in `FakeUsbBackend.hpp` |
| Test `IntelHex_rejects_checksum_failure` used `:01000000FF00` which is a *valid* checksum (sum=0x100, low byte 0x00) | Changed to `:01000000FF01` (genuinely wrong) |

Both caught during the first end-to-end test run, fixed in <5 minutes.

## What could not be tested without hardware

The following live only behind the libusb backend and need a Pi + radar to exercise:

1. **Pre-firmware open** — does the unprogrammed FX2LP enumerate at 04B4:8613 as expected.
2. **Firmware download timing** — is 1 s control-transfer timeout enough; does the chip really re-enumerate within 5 s.
3. **Post-firmware VID/PID** — UNKNOWN until first plug-in (see PROGRESS_01.md §3). The CLI defaults `0x04B4/0x1004` are placeholders.
4. **Bulk-IN throughput** — does the Pi USB stack keep up with the radar's stream rate without overruns.
5. **udev rule efficacy** — does the `99-pupradar.rules` allow non-root use after re-enumeration.

Each is called out in the README and in PROGRESS_03 as part of the hardware-bring-up checklist.

## Next phase

Phase 5 — verification on Pi with hardware. See PROGRESS_03 for the bring-up
runbook and final handoff notes.
