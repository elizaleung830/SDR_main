#include "pupradar/RadarSession.hpp"

#include "pupradar/IntelHex.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace pupradar {

namespace {

constexpr std::uint8_t kCtrlOut   = 0x40;  // vendor, host→device, recipient=device
constexpr std::uint8_t kCtrlIn    = 0xC0;  // vendor, device→host, recipient=device
constexpr unsigned int kCtrlTimeoutMs = 1000;
constexpr unsigned int kBulkOutTimeoutMs = 1000;

// GUI line 2672: DataLength = 512 + 2048
constexpr std::size_t kBoardInfoReadBytes = 512 + 2048;
// GUI line 2674: PUPradarBoardInfo(1025:1100) — word 1025 (1-based) = byte 2048
constexpr std::size_t kBoardInfoUsefulOffsetBytes = 2048;
// GUI reads 76 words (1025:1100) but only words 0-4 have meaning:
//   word 0 — FA05 signature (checked at GUI line 2675)
//   word 1 — FrequencyBand  (GUI line 2677)
//   word 2 — Num_Tx, Num_Rx, AntennaType (GUI line 2679-2681)
//   word 3 — Version        (GUI line 2682-2683)
// 32 bytes = 16 words gives comfortable headroom over those 4-5 semantic words.
// Hardware note: the device may return fewer bytes than kBoardInfoReadBytes;
// the decode in requestBoardInfo() is therefore bounded by what actually arrived.
constexpr std::size_t kBoardInfoExposedBytes = 32;

// FX2LP high-speed bulk max packet. Every bulk IN request length must be a
// whole multiple of this: ask for less than a packet and the device's next full
// packet overflows the buffer, which libusb reports as LIBUSB_ERROR_OVERFLOW
// and the kernel logs as a buffer overflow against the USB device.
constexpr std::size_t kBulkPacketBytes = 512;

// Largest single bulk-IN request. libusb's synchronous API submits the whole
// request to the kernel at once rather than splitting it, and usbfs caps the
// memory a process may pin for in-flight transfers — see
// /sys/module/usbcore/parameters/usbfs_memory_mb, 16 MB by default. Asking for
// a whole multi-megabyte capture in one call returns LIBUSB_ERROR_NO_MEM
// immediately (hit at --num-sweeps 12800, a ~50 MB transfer). 1 MiB leaves
// generous headroom under that cap while keeping the read loop cheap; it is a
// whole multiple of kBulkPacketBytes, as every request must be.
constexpr std::size_t kMaxBulkChunkBytes = 1024u * 1024u;
static_assert(kMaxBulkChunkBytes % kBulkPacketBytes == 0,
              "bulk chunk must be a whole number of max packets");

/**
 * @brief Returns the current UTC wall-clock time as an ISO-8601 string.
 * @return Timestamp in the form "YYYY-MM-DDTHH:MM:SSZ" (second resolution).
 */
std::string isoTimestampUtcNow() {
    auto t  = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(t);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &tt);
#else
    gmtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

/**
 * @brief Counts Tx1/Tx2 sweep-header marker words in a captured byte region.
 *
 * Mirrors the marker check in process_pi.m: interprets `data` as little-endian
 * uint16 words and counts how many fall in [kTx1HeaderTag, kTx1HeaderTag+kAdcMax]
 * or [kTx2HeaderTag, kTx2HeaderTag+kAdcMax]. Zero means the board never tagged a
 * single sweep — a strong signal that a configuration command was dropped.
 * @param data   Captured bytes (as handed to the IqSink).
 * @param length Number of bytes in `data`; a trailing odd byte is ignored.
 */
std::size_t countSweepMarkers(const std::uint8_t* data, std::size_t length) {
    std::size_t count = 0;
    for (std::size_t i = 0; i + 1 < length; i += 2) {
        const auto word = static_cast<std::uint16_t>(data[i] | (data[i + 1] << 8));
        const bool is_tx1 = word >= kTx1HeaderTag && word <= kTx1HeaderTag + kAdcMax;
        const bool is_tx2 = word >= kTx2HeaderTag && word <= kTx2HeaderTag + kAdcMax;
        if (is_tx1 || is_tx2) ++count;
    }
    return count;
}

}  // namespace

/**
 * @brief Constructs a RadarSession bound to a USB backend and a firmware image.
 * @param usb               Reference to the USB backend (lifetime must exceed this object).
 * @param firmware_hex_path Path to the FX3 firmware Intel HEX file (e.g. "SDR_USB_FW.hex").
 */
RadarSession::RadarSession(IUsbBackend& usb, std::string firmware_hex_path)
    : usb_(usb), fw_path_(std::move(firmware_hex_path)) {}

/**
 * @brief Sends one command word to EP2, then waits for the MCU to act on it.
 *
 * Wire format is a direct port of miniradarputdata.cpp: one 512-byte bulk-OUT
 * packet holding 256 little-endian repeats of `opcode << 8 | param`
 * (miniradarputdata.cpp:21 hard-codes the 512-byte length).
 *
 * The USB *state* model is a port too, and a deliberate one. Every MEX call
 * constructs and destroys its own CCyUSBDevice, but CyAPI does not touch the
 * interface on either — `usbsetinterface1` selects alternate setting 1 exactly
 * once during bring-up and the device stays there for the whole session. All the
 * per-call open/close actually buys MATLAB is *time* between command words, so
 * that is what cmd_delay_us_ reproduces, on one persistently claimed interface.
 *
 * Do NOT be tempted to open and close the device around each command to mimic
 * the MEX more literally: libusb_release_interface sends a SET_INTERFACE request
 * ("resetting interface state to the first alternate setting"), which flushes the
 * FX2's EP2 FIFO. Bracketing each write that way discards the command word before
 * the 8051 has consumed it — the front end then keeps its previous Tx/Rx setting
 * while the EP6 data path goes on streaming live samples, which is exactly the
 * "--receiver Rx1 and Rx4 produce identical captures" failure.
 *
 * @param opcode    Command opcode byte (see pupradar::opcode namespace in Protocol.hpp).
 * @param parameter Command parameter byte (semantics depend on opcode).
 * @throws std::runtime_error if fewer bytes than the full 512-byte packet are transferred.
 */
void RadarSession::sendCommand(std::uint8_t opcode, std::uint8_t parameter) {
    auto pkt = makeCommandPacket(opcode, parameter);
    std::size_t n = usb_.bulkWrite(kEpOutBulk, pkt.data(), pkt.size(),
                                    kBulkOutTimeoutMs);
    if (n != pkt.size()) {
        std::ostringstream oss;
        oss << "sendCommand(0x" << std::hex << +opcode << ",0x" << +parameter
            << "): short write " << std::dec << n << "/" << pkt.size();
        throw std::runtime_error(oss.str());
    }
    // The gap MATLAB gets for free from its per-call CCyUSBDevice construction.
    // See RadarSession::setCommandDelayUs.
    if (cmd_delay_us_ > 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(cmd_delay_us_));
    }
}

/**
 * @brief Downloads the Intel HEX firmware image into FX3 RAM via the Cypress 0xA0 loader protocol.
 *
 * Sequence mirrors usbdownload.cpp:
 *   1. Read CPUCS register (0xE600), set bit 0 to hold the 8051 in reset.
 *   2. Write each HEX data record to its target address via control transfers.
 *   3. Clear CPUCS bit 0 to release reset — the 8051 begins executing firmware.
 *
 * After this returns the chip re-enumerates under a new VID/PID. Callers must invoke
 * IUsbBackend::reopenAfterRenumeration() before issuing any further transfers.
 * @throws std::runtime_error if the HEX file contains no data records or if any
 *         control transfer completes with fewer bytes than expected.
 */
void RadarSession::downloadFirmware() {
    auto records = parseIntelHexFile(fw_path_);
    if (records.empty()) {
        throw std::runtime_error("Firmware HEX has no data records");
    }

    // Step 1: read CPUCS, set bit 0 (hold 8051 in reset), write back.
    std::uint8_t cpucs = 0;
    usb_.controlTransfer(kCtrlIn,  kFwLoaderRequest, kCpucsRegister, 0,
                         &cpucs, 1, kCtrlTimeoutMs);
    cpucs = static_cast<std::uint8_t>(cpucs | kCpucsReset);
    usb_.controlTransfer(kCtrlOut, kFwLoaderRequest, kCpucsRegister, 0,
                         &cpucs, 1, kCtrlTimeoutMs);

    // Step 2: write each data record into 8051 RAM.
    for (const auto& rec : records) {
        if (rec.data.empty()) continue;
        std::size_t n = usb_.controlTransfer(
            kCtrlOut, kFwLoaderRequest, rec.address, 0,
            const_cast<std::uint8_t*>(rec.data.data()),
            rec.data.size(), kCtrlTimeoutMs);
        if (n != rec.data.size()) {
            std::ostringstream oss;
            oss << "Firmware download: short write at addr 0x"
                << std::hex << rec.address << " (" << std::dec
                << n << "/" << rec.data.size() << ")";
            throw std::runtime_error(oss.str());
        }
    }

    // Step 3: clear CPUCS bit 0 → 8051 starts running firmware.
    cpucs = static_cast<std::uint8_t>(cpucs & ~kCpucsReset);
    usb_.controlTransfer(kCtrlOut, kFwLoaderRequest, kCpucsRegister, 0,
                         &cpucs, 1, kCtrlTimeoutMs);
}

/**
 * @brief Sends the 0xFA board-info request and parses the response into last_board_info_.
 *
 * Mirrors the GUI at line 2674: sends opcode kBoardInfoRequest, bulk-reads
 * kBoardInfoReadBytes (512 + 2048) bytes from EP6, then extracts up to
 * kBoardInfoExposedBytes starting at kBoardInfoUsefulOffsetBytes (2048),
 * decoded as little-endian uint16 words.
 *
 * Hardware note: the device may return fewer than kBoardInfoReadBytes bytes.
 * The observed minimum is ~2058 (useful_offset + 10), which covers all 5
 * semantically meaningful words from the MATLAB GUI (lines 2675-2683).
 * The decode is therefore bounded by however many bytes actually arrived,
 * not by the nominal kBoardInfoExposedBytes.
 *
 * @throws std::runtime_error if the bulk read returns fewer than
 *         kBoardInfoUsefulOffsetBytes + 2 bytes (less than 1 uint16 word).
 */
void RadarSession::requestBoardInfo() {
    // PUPradar_initiating.m:131-134 — miniradarputdata(FA00) then miniradargetdata.
    sendCommand(opcode::kBoardInfoRequest, 0x00);
    std::vector<std::uint8_t> buf(kBoardInfoReadBytes, 0);
    std::size_t n = usb_.bulkRead(kEpInBulk, buf.data(), buf.size(),
                                   /*timeout_ms*/ 2000);
    if (n < kBoardInfoUsefulOffsetBytes + 2) {
        std::ostringstream oss;
        oss << "Board info: short read " << n
            << " bytes (need at least " << (kBoardInfoUsefulOffsetBytes + 2) << ")";
        throw std::runtime_error(oss.str());
    }
    // Decode however many complete uint16 words arrived, up to kBoardInfoExposedBytes.
    const std::size_t avail_bytes = n - kBoardInfoUsefulOffsetBytes;
    const std::size_t decode_bytes = (std::min(kBoardInfoExposedBytes, avail_bytes) / 2) * 2;
    last_board_info_.clear();
    last_board_info_.reserve(decode_bytes / 2);
    for (std::size_t i = 0; i < decode_bytes; i += 2) {
        std::uint8_t lo = buf[kBoardInfoUsefulOffsetBytes + i];
        std::uint8_t hi = buf[kBoardInfoUsefulOffsetBytes + i + 1];
        last_board_info_.push_back(
            static_cast<std::uint16_t>((hi << 8) | lo));
    }
}

/**
 * @brief Full device bring-up, retried until the board info carries FA05.
 *
 * Each attempt (attemptBringUp()) performs the sequence PUPradar_initiating()
 * does in the MATLAB GUI:
 *   1. Open the unprogrammed FX2LP (VID 0x04B4 / PID 0x8613).
 *   2. Claim interface 0, set alternate setting 1 (benign if the descriptor lacks it).
 *   3. Download firmware via the Cypress 0xA0 loader (downloadFirmware()).
 *   4. Wait for the chip to detach and re-enumerate under one of post_fw_candidates.
 *      (PUPradar_initiating.m has no equivalent wait — MATLAB re-opens "the first
 *      Cypress device" fresh on every mex call — but the firmware download holds
 *      one handle throughout, like usbdownload.cpp, so it must wait for the real
 *      hardware re-enumeration that MATLAB's per-call opens happen to paper over.)
 *   5. Claim interface 0 and select alternate setting 1 once, as usbsetinterface1
 *      does; the interface is then left alone for the rest of the session.
 *   6. Request board info (requestBoardInfo()) — result available via lastBoardInfo().
 *
 * The retry itself mirrors PUPradarGUI.m:114-122, which calls
 * PUPradar_initiating() twice unconditionally and then up to 5 more times in
 * a while loop until the board info decodes — the full bring-up, firmware
 * download included, not just the board-info request. (The standalone
 * PUPradar_initiating.m script this file's comments otherwise track drops
 * that retry and raises on the first failure; the actual GUI it was
 * extracted from does not — this class follows the GUI here.) A board-info
 * reply that still lacks FA05 after all attempts is only a warning, per
 * PUPradar_initiating.m: "the endpoints are still valid, so acquisition may
 * still work."
 *
 * @param post_fw_candidates Non-empty list of (VID, PID) pairs the chip may present after
 *                           the firmware boots. Tried in order until one opens or the timeout
 *                           elapses.
 * @param reenum_timeout_ms  Maximum time in milliseconds to wait for re-enumeration
 *                           on each attempt (default: 5000 ms).
 * @throws std::invalid_argument if post_fw_candidates is empty.
 * @throws UsbError or std::runtime_error if the final attempt hits a USB failure.
 */
void RadarSession::initialize(
        const std::vector<IUsbBackend::VidPid>& post_fw_candidates,
        unsigned int reenum_timeout_ms) {
    if (post_fw_candidates.empty()) {
        throw std::invalid_argument("Need at least one post-firmware VID/PID candidate");
    }
    // Every operation after the firmware download reopens the device against
    // these, the way each MEX call constructs a fresh CCyUSBDevice.
    post_fw_ = post_fw_candidates;

    // GUI's 2 unconditional attempts + 5 retries (PUPradarGUI.m:114-122).
    constexpr int kMaxInitAttempts = 7;
    for (int attempt = 1; attempt <= kMaxInitAttempts; ++attempt) {
        std::fprintf(stderr, "[pupradar] init attempt %d/%d\n",
                     attempt, kMaxInitAttempts);
        try {
            attemptBringUp(post_fw_candidates, reenum_timeout_ms);
            if (decodeBoardInfo(last_board_info_).valid) {
                std::fprintf(stderr, "[pupradar] init attempt %d: FA05 OK\n", attempt);
                return;  // FA05 present — device is in a known-good state.
            }
            std::fprintf(stderr, "[pupradar] init attempt %d: no FA05 in board info\n",
                         attempt);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[pupradar] init attempt %d failed: %s\n",
                         attempt, e.what());
        }
        usb_.close();  // drop the handle so the next attempt opens cleanly
    }
    std::fprintf(stderr,
                 "[pupradar] WARNING: all %d init attempts failed to return a valid\n"
                 "[pupradar]          FA05 board info. The endpoints are still valid,\n"
                 "[pupradar]          so acquisition may still work.\n",
                 kMaxInitAttempts);
}

/**
 * @brief One pass of the bring-up sequence, as attempted by initialize().
 *
 * Split out so initialize() can retry it. Leaves the board info in
 * last_board_info_; the caller checks it for the FA05 signature.
 *
 * @param post_fw_candidates VID/PID pairs the chip may present after booting.
 * @param reenum_timeout_ms  Re-enumeration wait budget.
 * @throws UsbError or std::runtime_error on any USB failure.
 */
void RadarSession::attemptBringUp(
        const std::vector<IUsbBackend::VidPid>& post_fw_candidates,
        unsigned int reenum_timeout_ms) {
    usb_.open(kPreFirmwareVid, kPreFirmwarePid);
    std::fprintf(stderr, "[pupradar]   opened %04X:%04X\n",
                 kPreFirmwareVid, kPreFirmwarePid);

    usb_.claimInterface(kInterfaceNumber);

    // Set alt 1 — this matches PUPradar_initiating ordering. On the
    // unprogrammed FX2LP this is benign because the default descriptor
    // exposes a single interface 0 with one alt setting.
    try {
        usb_.setAltSetting(kInterfaceNumber, kAlternateSetting);
    } catch (const UsbError&) {
        // Some unprogrammed FX2LP descriptors do not have alt 1; ignore here.
    }

    std::fprintf(stderr, "[pupradar]   downloading firmware…\n");
    downloadFirmware();

    // Re-enumeration: chip detaches and re-appears with firmware-defined VID/PID.
    std::fprintf(stderr, "[pupradar]   waiting for re-enumeration…\n");
    usb_.reopenAfterRenumeration(post_fw_candidates.data(),
                                 post_fw_candidates.size(),
                                 reenum_timeout_ms);
    // Claim and select alternate setting 1 ONCE, as usbsetinterface1 does, and
    // then leave the interface alone for the rest of the session. Re-selecting
    // it later would flush the FX2 endpoint FIFOs mid-configuration.
    usb_.claimInterface(kInterfaceNumber);
    usb_.setAltSetting(kInterfaceNumber, kAlternateSetting);

    requestBoardInfo();
}

/**
 * @brief Pushes a full set of FMCW parameters to the radar over USB.
 *
 * Sends commands in the same order as the MATLAB GUI (SetActiveParameters →
 * Send_Basic_Parameter → Send_PLL_Sawtooth): modulation, sweep time index, sampling
 * number index, Tx mask, Rx mask, then all PLL register bytes for the sawtooth ramp.
 * @param cfg Capture configuration describing the desired waveform. The MVP requires
 *            cfg.modulation == Modulation::Sawtooth; CW is deferred.
 * @throws std::invalid_argument if cfg.modulation is not Modulation::Sawtooth.
 * @throws std::runtime_error on any USB write failure (propagated from sendCommand).
 */
void RadarSession::configure(const CaptureConfig& cfg) {
    if (cfg.modulation != Modulation::Sawtooth) {
        throw std::invalid_argument("MVP supports Sawtooth only (CW deferred)");
    }

    // Order mirrors GUI: SetActiveParameters → Send_Basic_Parameter → Send_PLL_Sawtooth.
    // Modulation
    sendCommand(opcode::kModulation, static_cast<std::uint8_t>(cfg.modulation));
    // Sweep time index
    sendCommand(opcode::kSweepTimeIndex,
                static_cast<std::uint8_t>(cfg.sweep_time_idx));
    // Sampling number index
    sendCommand(opcode::kSamplingNumIndex,
                static_cast<std::uint8_t>(cfg.samp_num_idx));
    // Tx mask
    sendCommand(opcode::kTxMask, cfg.tx_mask);
    // Rx mask
    sendCommand(opcode::kRxMask, cfg.rx_mask);

    // PLL register byte writes (sawtooth)
    auto pll = buildSawtoothPllCommands(cfg.f_low_hz, cfg.f_high_hz,
                                        cfg.sweep_time_idx);
    for (const auto& c : pll) sendCommand(c.opcode, c.param);
}

/**
 * @brief Records cfg.num_sweeps of IQ the way the GUI's Record button does.
 *
 * Follows PUPradarGUI.m:1831-1859 (Record), NOT the display loop at :203-248:
 *   1. configure() ONCE — basic parameters and PLL registers, paced by
 *      cmd_delay_us_ so the MCU latches each word (see sendCommand).
 *   2. One logical transfer sized for the whole recording:
 *      cfg.num_sweeps straight from the caller (MATLAB's
 *      handles.NumSweeps), then GUI line 2730's
 *      ceil((NumSweeps+40)*LASN*2*LANR*LANT/512)*512 + 4096 bytes. libusb may
 *      satisfy it in pieces; the config is never re-sent between them.
 *   3. Drop the leading 4096 stale-FIFO bytes (GUI line 2734) before the sink.
 *
 * The device is deliberately not reconfigured during the transfer. Doing so
 * re-triggers the sweep generator mid-stream, which yields one live sweep per
 * read followed by the mixer output decaying to DC.
 *
 * @param cfg  Capture configuration used for buffer sizing, run duration, and bulk timeout.
 *             Applied by this function; no prior configure() call is needed.
 * @param sink Called once with the de-preambled capture (pointer into an
 *             internal buffer, valid only for the duration of the call).
 * @return CaptureMetadata populated with echoed config values, derived sample rate (fs_hz),
 *         sweep time, actual capture duration, byte and read counts, last board info words,
 *         firmware path, and a UTC ISO-8601 timestamp.
 * @throws std::runtime_error on a USB error during bulk transfer (propagated from bulkRead).
 */
CaptureMetadata RadarSession::captureIq(const CaptureConfig& cfg,
                                         const IqSink& sink) {
    const int n_rx = rxCount(cfg.rx_mask);
    const int n_tx = txCount(cfg.tx_mask);
    const int samples_per_sweep_per_rx =
        samplesPerSweep(cfg.sweep_time_idx, cfg.samp_num_idx, n_rx);
    const double sweep_time_s = sweepTimeFromIndex(cfg.sweep_time_idx);
    const double fs_hz = static_cast<double>(samples_per_sweep_per_rx) / sweep_time_s;

    // GUI line 2728: LASNperSweep = LASN*LANR*2*LANT — that is a count of uint16
    // WORDS per sweep (the *2 is I and Q). Confirmed against real captures: at
    // LASN=128, 1 Rx, 1 Tx the measured marker-to-marker stride is 256 words.
    const std::size_t words_per_sweep =
        static_cast<std::size_t>(samples_per_sweep_per_rx) *
        2u /* I and Q */ *
        static_cast<std::size_t>(n_rx) * static_cast<std::size_t>(n_tx);

    // ...and two bytes per word. The GUI drops this factor: line 2730 builds
    // DataLength from the WORD count, but miniradargetdata hands it to CyAPI
    // XferData as a BYTE count (miniradargetdata.cpp:40), so the GUI actually
    // transfers half the sweeps it asked for — which is why it then has to clamp
    // NumSweeps down to ValidSweepNumber at lines 2744-2748. Mirroring that
    // exactly gave captures half the requested duration, so keep the factor.
    const std::size_t bytes_per_sweep = words_per_sweep * 2u;

    // ---- 1. Size the transfer, as GUI Record does --------------------------
    //
    // GUI Record (PUPradarGUI.m:1831-1859) sets NumSweeps = RecordTime /
    // SweepTime — the whole recording, not a batch — and makes a SINGLE
    // GetComplexData call for it.
    // MATLAB passes NumSweeps straight in (handles.NumSweeps); we do the same
    // rather than deriving it from a wall-clock duration.
    const auto num_sweeps = static_cast<std::size_t>(cfg.num_sweeps);
    const double expected_duration_s = static_cast<double>(num_sweeps) * sweep_time_s;
    const std::size_t raw_bytes = (num_sweeps + 40) * bytes_per_sweep;
    // Round UP to a 512-byte boundary, then add 4096 (GUI line 2730). The extra
    // 4096 exists because the first 4096 bytes are stale FIFO content; the GUI
    // discards exactly that many at line 2734.
    const std::size_t capture_bytes = ((raw_bytes + 511u) / 512u) * 512u + 4096u;

    // ---- 2. Buffer and the read helper both phases share -------------------
    //
    // One max-packet of slack past capture_bytes. libusb hands the whole request
    // to the kernel, which will write a complete packet or fail the transfer, so
    // the destination must always have room for one even when the tail of the
    // capture does not need it.
    std::vector<std::uint8_t> buf(capture_bytes + kBulkPacketBytes, 0);

    // Fills `buf` up to capture_bytes without reconfiguring, returning the bytes
    // received.
    //
    // Every request length is a whole multiple of the 512-byte endpoint max
    // packet. A bulk IN request that is not gets LIBUSB_ERROR_OVERFLOW the
    // moment the device sends a full packet into the short remainder — which
    // surfaces as a kernel "buffer overflow" against the USB device. Rounding
    // the remainder down is what keeps every request legal; the slack in `buf`
    // covers the case where the device answers with a partial packet and leaves
    // total_bytes off a 512-byte boundary.
    //
    // Requests are also capped at kMaxBulkChunkBytes. libusb does NOT split a
    // large synchronous request for us — it pins the whole thing at once — and
    // usbfs limits how much memory a process may pin for in-flight transfers
    // (/sys/module/usbcore/parameters/usbfs_memory_mb, 16 MB by default). A long
    // capture asked for in one call fails immediately with LIBUSB_ERROR_NO_MEM.
    //
    // Each chunk gets a timeout scaled to how long the board needs to produce
    // that many bytes, plus cfg.bulk_timeout_ms of slack, so a board that stops
    // streaming mid-capture fails fast instead of stalling once per chunk on a
    // whole-capture-sized timeout.
    const double bytes_per_second =
        static_cast<double>(bytes_per_sweep) / sweep_time_s;
    auto readCapture = [&](std::size_t& reads_out) -> std::size_t {
        std::size_t total = 0;
        while (total < capture_bytes) {
            const std::size_t remaining = capture_bytes - total;
            const std::size_t request =
                std::min((remaining / kBulkPacketBytes) * kBulkPacketBytes,
                         kMaxBulkChunkBytes);
            if (request == 0) break;  // less than one packet left to ask for
            const auto chunk_timeout_ms =
                static_cast<unsigned int>(
                    (static_cast<double>(request) / bytes_per_second) * 1000.0) +
                cfg.bulk_timeout_ms;
            std::size_t got = usb_.bulkRead(kEpInBulk, buf.data() + total,
                                             request, chunk_timeout_ms);
            ++reads_out;
            if (got == 0) break;  // device has nothing more to give
            total += got;
        }
        return total;
    };

    // ---- 3. Configure, then capture -----------------------------------------
    //
    // configure() is sent ONCE — run_GetComplexData.m does the same, and an
    // earlier implementation that re-sent all 22 packets before every 30 KB read
    // re-triggered the sweep generator mid-stream, producing one live sweep per
    // read followed by the mixer output decaying to DC. What makes the MCU
    // actually latch those 22 words is the pacing in sendCommand().
    using clock = std::chrono::steady_clock;

    configure(cfg);

    // ---- 4. The recording --------------------------------------------------
    //
    // One logical transfer, as GetComplexData.m makes a single miniradargetdata
    // call. libusb needs it split into bounded chunks (see kMaxBulkChunkBytes),
    // but the config is never re-sent between them.
    std::size_t reads_done  = 0;
    const auto t_start = clock::now();
    const std::size_t total_bytes = readCapture(reads_done);
    const auto t_done = clock::now();

    // Discard the stale FIFO preamble, as GUI line 2734
    // (RawData = double(RawData(2049:end))), then hand over the real samples.
    constexpr std::size_t kStalePreambleBytes = 2048 * 2;
    std::size_t markers_found = 0;
    if (total_bytes > kStalePreambleBytes) {
        const std::uint8_t* captured = buf.data() + kStalePreambleBytes;
        const std::size_t   captured_len = total_bytes - kStalePreambleBytes;
        markers_found = countSweepMarkers(captured, captured_len);
        sink(captured, captured_len);
    }

    CaptureMetadata md;
    md.f_low_hz                 = cfg.f_low_hz;
    md.f_high_hz                = cfg.f_high_hz;
    md.fs_hz                    = fs_hz;
    md.sweep_time_s             = sweep_time_s;
    md.samples_per_sweep_per_rx = samples_per_sweep_per_rx;
    md.num_tx                   = n_tx;
    md.num_rx                   = n_rx;
    md.tx_mask                  = cfg.tx_mask;
    md.rx_mask                  = cfg.rx_mask;
    md.num_sweeps               = cfg.num_sweeps;
    md.requested_duration_s     = expected_duration_s;
    md.actual_duration_s        = std::chrono::duration<double>(t_done - t_start).count();
    // Report what actually reached the sink (and therefore the .bin), not what
    // came off the wire, so the sidecar matches the file on disk.
    md.bytes_captured           = total_bytes > kStalePreambleBytes
                                      ? total_bytes - kStalePreambleBytes : 0;
    md.reads_completed          = reads_done;
    md.sweep_markers_found      = markers_found;
    md.board_info_words         = last_board_info_;
    md.firmware_path            = fw_path_;
    md.timestamp_utc            = isoTimestampUtcNow();
    md.cmd_delay_us             = cmd_delay_us_;
    return md;
}

/**
 * @brief Releases all USB interfaces and closes the device handle.
 *
 * After this call the RadarSession is not usable again without a fresh call to initialize().
 */
void RadarSession::shutdown() {
    usb_.close();
}

}  // namespace pupradar
