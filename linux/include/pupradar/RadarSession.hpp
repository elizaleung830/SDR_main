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
    /// Sweeps to acquire — the MATLAB handles.NumSweeps, not a duration.
    /// GetComplexData takes a sweep count and derives the transfer size from
    /// it (PUPradarGUI.m:2730); expressing this as seconds meant the two
    /// implementations sized their transfers from different quantities.
    int             num_sweeps      = 128;
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
    int             num_sweeps;
    double          requested_duration_s;   // num_sweeps * sweep_time_s
    double          actual_duration_s;
    // What we measured
    std::size_t     bytes_captured;
    std::size_t     reads_completed;
    // Count of Tx1/Tx2 sweep-header marker words found in the captured region
    // (see Protocol.hpp kTx1HeaderTag/kTx2HeaderTag). Zero means the board
    // never tagged a single sweep — almost always a dropped configuration
    // command rather than a hardware fault; see RadarSession::setCommandDelayUs.
    std::size_t     sweep_markers_found = 0;
    // Board info from 0xFA00 response (raw, host-byte-order uint16 dump,
    // first 16 16-bit words from the info region — let user verify modelcode)
    std::vector<std::uint16_t> board_info_words;
    std::string     firmware_path;
    // ISO-8601 UTC timestamp
    std::string     timestamp_utc;
    // Inter-command settling delay in effect during this capture (microseconds).
    unsigned int    cmd_delay_us = 0;
};

// Non-owning pointer to a sink that writes IQ bytes to disk (or wherever).
// We keep this a simple callback so the writer is fully decoupled from the
// session and easy to mock in tests.
using IqSink = std::function<void(const std::uint8_t* data, std::size_t length)>;

class RadarSession {
public:
    RadarSession(IUsbBackend& usb, std::string firmware_hex_path);

    /// Full bring-up: enumerate (pre-firmware), claim, download firmware,
    /// wait for re-enumeration, claim again, request board info. Retried up
    /// to 7 times if the board info doesn't carry the FA05 signature — this
    /// mirrors PUPradarGUI.m:114-122, which calls PUPradar_initiating() twice
    /// unconditionally and then up to 5 more times in a while loop before
    /// giving up. (The standalone PUPradar_initiating.m script drops this
    /// retry and just raises on the first failure; the GUI it was extracted
    /// from does not.)
    /// @param post_fw_candidates List of (vid,pid) the chip may take after firmware load.
    /// Throws UsbError or std::runtime_error on failure.
    void initialize(const std::vector<IUsbBackend::VidPid>& post_fw_candidates,
                    unsigned int reenum_timeout_ms = 5000);

    /// Apply a configuration: send modulation/sweep/sampling/Tx/Rx and PLL regs.
    void configure(const CaptureConfig& cfg);

    /// Bulk-read IQ for `cfg.duration_s` seconds, sinking bytes to `sink`.
    /// Fills out the metadata struct. Does NOT include parsing of the byte
    /// stream — capture-only MVP.
    CaptureMetadata captureIq(const CaptureConfig& cfg, const IqSink& sink);

    /// Power-down convenience: release interfaces, close device.
    void shutdown();

    /// Extra settling delay inserted after every command packet, in microseconds.
    ///
    /// Supplemental only. Each command already gets its own full USB device
    /// open/claim/transfer/close cycle (mirroring miniradarputdata.cpp), and
    /// that cycle is the primary thing giving the MCU time to latch a command
    /// before the next one arrives. Raise this if the board still ignores a
    /// parameter — i.e. if a capture comes back with sweep_markers_found == 0.
    void setCommandDelayUs(unsigned int us) { cmd_delay_us_ = us; }
    unsigned int commandDelayUs() const { return cmd_delay_us_; }

    /// For testing / observability.
    const std::vector<std::uint16_t>& lastBoardInfo() const { return last_board_info_; }

private:
    void sendCommand(std::uint8_t opcode, std::uint8_t parameter);
    void downloadFirmware();
    void requestBoardInfo();
    /// One pass of the bring-up sequence; initialize() retries this until the
    /// board info carries the FA05 signature.
    void attemptBringUp(const std::vector<IUsbBackend::VidPid>& post_fw_candidates,
                        unsigned int reenum_timeout_ms);

    IUsbBackend&               usb_;
    std::string                fw_path_;
    std::vector<std::uint16_t> last_board_info_;
    unsigned int               cmd_delay_us_ = 0;
    /// VID/PID the board presents once its firmware is running. Every
    /// post-bring-up operation reopens the device against this list, the way
    /// each MEX call constructs a fresh CCyUSBDevice. initialize() overwrites
    /// it with the candidates it was given; the default is what the board
    /// actually presents today, so callers that skip initialize() (tests, and
    /// any direct configure()/captureIq() use) still work.
    std::vector<IUsbBackend::VidPid> post_fw_{
        {kPreFirmwareVid, kPreFirmwarePid}};
};

}  // namespace pupradar
