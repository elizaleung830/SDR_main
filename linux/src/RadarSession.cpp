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

// 0xFA00 board-info read length, matching MATLAB DataLength = 512 + 2048
constexpr std::size_t kBoardInfoReadBytes = 512 + 2048;
// Offset (in bytes) of useful info inside the board-info read, per GUI line 2674
constexpr std::size_t kBoardInfoUsefulOffsetBytes = 2048;
// We expose 32 bytes (16 uint16 words) of useful info — the GUI reads 76 words
// but only the first ~5 carry semantic meaning. 32 bytes is enough headroom.
constexpr std::size_t kBoardInfoExposedBytes = 32;

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

RadarSession::RadarSession(IUsbBackend& usb, std::string firmware_hex_path)
    : usb_(usb), fw_path_(std::move(firmware_hex_path)) {}

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

void RadarSession::requestBoardInfo() {
    sendCommand(opcode::kBoardInfoRequest, 0x00);
    std::vector<std::uint8_t> buf(kBoardInfoReadBytes, 0);
    std::size_t n = usb_.bulkRead(kEpInBulk, buf.data(), buf.size(),
                                   /*timeout_ms*/ 2000);
    if (n < kBoardInfoUsefulOffsetBytes + kBoardInfoExposedBytes) {
        std::ostringstream oss;
        oss << "Board info: short read " << n << " < expected "
            << (kBoardInfoUsefulOffsetBytes + kBoardInfoExposedBytes);
        throw std::runtime_error(oss.str());
    }
    last_board_info_.clear();
    last_board_info_.reserve(kBoardInfoExposedBytes / 2);
    for (std::size_t i = 0; i < kBoardInfoExposedBytes; i += 2) {
        std::uint8_t lo = buf[kBoardInfoUsefulOffsetBytes + i];
        std::uint8_t hi = buf[kBoardInfoUsefulOffsetBytes + i + 1];
        last_board_info_.push_back(
            static_cast<std::uint16_t>((hi << 8) | lo));
    }
}

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
    const std::size_t bytes_per_sweep =
        static_cast<std::size_t>(samples_per_sweep_per_rx) *
        2u /* I,Q */ * 2u /* int16 */ *
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

void RadarSession::shutdown() {
    usb_.close();
}

}  // namespace pupradar
