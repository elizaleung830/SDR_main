#pragma once

#include "pupradar/IUsbBackend.hpp"
#include "pupradar/Protocol.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace pupradar {

struct CaptureConfig {
    double          f_low_hz       = 24.0e9;
    double          f_high_hz      = 24.25e9;
    int             sweep_time_idx = 2;        // 1ms
    int             samp_num_idx   = 1;        // BASN (max samples)
    std::uint8_t    tx_mask        = 0x01;     // Tx1 only
    std::uint8_t    rx_mask        = 0x01;     // Rx1 only
    Modulation      modulation     = Modulation::Sawtooth;
    double          duration_s     = 5.0;
    unsigned int    bulk_timeout_ms = 2000;
};

struct CaptureMetadata {
    // Echoed configuration
    double          f_low_hz;
    double          f_high_hz;
    double          fs_hz;             // samples / sweep / sweep_time
    double          sweep_time_s;
    int             samples_per_sweep_per_rx;
    int             num_tx;
    int             num_rx;
    std::uint8_t    tx_mask;
    std::uint8_t    rx_mask;
    double          requested_duration_s;
    double          actual_duration_s;
    // What we measured
    std::size_t     bytes_captured;
    std::size_t     reads_completed;
    // Board info from 0xFA00 response (raw, host-byte-order uint16 dump,
    // first 16 16-bit words from the info region — let user verify modelcode)
    std::vector<std::uint16_t> board_info_words;
    std::string     firmware_path;
    // ISO-8601 UTC timestamp
    std::string     timestamp_utc;
};

// Non-owning pointer to a sink that writes IQ bytes to disk (or wherever).
// We keep this a simple callback so the writer is fully decoupled from the
// session and easy to mock in tests.
using IqSink = std::function<void(const std::uint8_t* data, std::size_t length)>;

class RadarSession {
public:
    RadarSession(IUsbBackend& usb, std::string firmware_hex_path);

    // Full bring-up: enumerate (pre-firmware), claim, download firmware,
    // wait for re-enumeration, claim again, request board info.
    // post_fw_candidates: list of (vid,pid) the chip may take after firmware load.
    // Throws UsbError or std::runtime_error on failure.
    void initialize(const std::vector<IUsbBackend::VidPid>& post_fw_candidates,
                    unsigned int reenum_timeout_ms = 5000);

    // Apply a configuration: send modulation/sweep/sampling/Tx/Rx and PLL regs.
    void configure(const CaptureConfig& cfg);

    // Bulk-read IQ for `cfg.duration_s` seconds, sinking bytes to `sink`.
    // Fills out the metadata struct. Does NOT include parsing of the byte
    // stream — capture-only MVP per PLAN.md.
    CaptureMetadata captureIq(const CaptureConfig& cfg, const IqSink& sink);

    // Power-down convenience: release interfaces, close device.
    void shutdown();

    // For testing / observability.
    const std::vector<std::uint16_t>& lastBoardInfo() const { return last_board_info_; }

private:
    void sendCommand(std::uint8_t opcode, std::uint8_t parameter);
    void downloadFirmware();
    void requestBoardInfo();

    IUsbBackend&               usb_;
    std::string                fw_path_;
    std::vector<std::uint16_t> last_board_info_;
};

}  // namespace pupradar
