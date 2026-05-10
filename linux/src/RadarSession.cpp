#include "pupradar/RadarSession.hpp"

#include "pupradar/IntelHex.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
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

}  // namespace

/**
 * @brief Constructs a RadarSession bound to a USB backend and a firmware image.
 * @param usb               Reference to the USB backend (lifetime must exceed this object).
 * @param firmware_hex_path Path to the FX3 firmware Intel HEX file (e.g. "SDR_USB_FW.hex").
 */
RadarSession::RadarSession(IUsbBackend& usb, std::string firmware_hex_path)
    : usb_(usb), fw_path_(std::move(firmware_hex_path)) {}

/**
 * @brief Packs an opcode/parameter pair into a 512-byte bulk-OUT packet and sends it to EP2.
 *
 * The MCU interprets only the first 16-bit little-endian word (opcode | param << 8), but the
 * full 512-byte max-packet must be sent to avoid FX2LP framing issues observed on some hosts.
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
 * @brief Full device bring-up: open pre-firmware device, download firmware, wait for
 *        re-enumeration, reclaim the interface, and fetch board info.
 *
 * Mirrors PUPradar_initiating() in the MATLAB GUI. Steps:
 *   1. Open the unprogrammed FX2LP (VID 0x04B4 / PID 0x8613).
 *   2. Claim interface 0, set alternate setting 1 (benign if the descriptor lacks it).
 *   3. Download firmware via the Cypress 0xA0 loader (downloadFirmware()).
 *   4. Wait for the chip to re-enumerate under one of post_fw_candidates.
 *   5. Claim interface 0 and set alternate setting 1 again (now required for bulk EPs).
 *   6. Request board info (requestBoardInfo()) — result available via lastBoardInfo().
 *
 * @param post_fw_candidates Non-empty list of (VID, PID) pairs the chip may present after
 *                           the firmware boots. Tried in order until one opens or the timeout
 *                           elapses.
 * @param reenum_timeout_ms  Maximum time in milliseconds to wait for re-enumeration
 *                           (default: 5000 ms).
 * @throws std::invalid_argument if post_fw_candidates is empty.
 * @throws UsbError or std::runtime_error on any USB failure.
 */
void RadarSession::initialize(
        const std::vector<IUsbBackend::VidPid>& post_fw_candidates,
        unsigned int reenum_timeout_ms) {
    if (post_fw_candidates.empty()) {
        throw std::invalid_argument("Need at least one post-firmware VID/PID candidate");
    }

    // Pre-firmware open
    usb_.open(kPreFirmwareVid, kPreFirmwarePid);
    usb_.claimInterface(kInterfaceNumber);
    
    // Set alt 1 — this matches PUPradar_initiating ordering. On the
    // unprogrammed FX2LP this is benign because the default descriptor
    // exposes a single interface 0 with one alt setting.
    try {
        usb_.setAltSetting(kInterfaceNumber, kAlternateSetting);
    } catch (const UsbError&) {
        // Some unprogrammed FX2LP descriptors do not have alt 1; ignore here,
        // we'll set it again post-firmware where it matters.
    }

    downloadFirmware();

    // Re-enumeration: chip detaches and re-appears with firmware-defined VID/PID.
    usb_.reopenAfterRenumeration(post_fw_candidates.data(),
                                 post_fw_candidates.size(),
                                 reenum_timeout_ms);
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
 * @brief Bulk-reads raw IQ data from EP6 for cfg.duration_s seconds, forwarding each
 *        chunk to sink.
 *
 * Buffer sizing mirrors the GUI at line 2730: (kNumSweepsPerRead=64 + 40) sweeps worth of
 * raw bytes, rounded up to a 512-byte boundary, plus 4096 bytes of padding. The capture
 * loop runs until std::chrono::steady_clock exceeds the computed deadline; zero-length
 * reads are silently skipped. No DSP is performed — this is a capture-only MVP.
 *
 * @param cfg  Capture configuration used for buffer sizing, run duration, and bulk timeout.
 *             Must match the configuration previously sent via configure().
 * @param sink Callback invoked with each non-empty bulk read chunk (raw pointer into an
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

    // Read size mirrors GUI line 2730: round to 512-byte boundary, plus 4096
    // padding. NumSweeps batched to 64 to match the GUI's run loop, which is
    // what the firmware's USB FIFO is sized for.
    constexpr int kNumSweepsPerRead = 64;
    // samplesPerSweep() returns the BASN value, which is already a uint16 count
    // with I and Q interleaved (both channels). Multiplying by 2 for int16 byte
    // width is correct; multiplying again for I,Q would double-count and produce
    // a buffer 2× too large, causing libusb to wait beyond the FX3's burst size.
    const std::size_t bytes_per_sweep =
        static_cast<std::size_t>(samples_per_sweep_per_rx) *
        2u /* bytes per uint16 */ *
        static_cast<std::size_t>(n_rx) * static_cast<std::size_t>(n_tx);
    const std::size_t raw_bytes_per_read =
        static_cast<std::size_t>(kNumSweepsPerRead + 40) * bytes_per_sweep;
    // Round UP to 512-byte boundary, then add 4096
    const std::size_t bytes_per_read =
        ((raw_bytes_per_read + 511u) / 512u) * 512u + 4096u;

    std::vector<std::uint8_t> buf(bytes_per_read, 0);

    using clock = std::chrono::steady_clock;
    const auto t_start = clock::now();
    const auto t_end   = t_start +
        std::chrono::milliseconds(static_cast<long>(cfg.duration_s * 1000.0));

    std::size_t total_bytes = 0;
    std::size_t reads_done  = 0;

    while (clock::now() < t_end) {
        // Mirror MATLAB GUI: re-send full config before every read so the FX3
        // re-arms its capture buffer (it fills one batch per configure trigger,
        // then outputs 0x07FD filler until re-triggered).
        configure(cfg);
        std::size_t got = usb_.bulkRead(kEpInBulk, buf.data(), buf.size(),
                                         cfg.bulk_timeout_ms);
        if (got == 0) continue;
        sink(buf.data(), got);
        total_bytes += got;
        ++reads_done;
    }
    const auto t_done = clock::now();

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
    md.requested_duration_s     = cfg.duration_s;
    md.actual_duration_s        = std::chrono::duration<double>(t_done - t_start).count();
    md.bytes_captured           = total_bytes;
    md.reads_completed          = reads_done;
    md.board_info_words         = last_board_info_;
    md.firmware_path            = fw_path_;
    md.timestamp_utc            = isoTimestampUtcNow();
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
