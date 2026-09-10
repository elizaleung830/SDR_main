// pupradar_capture — capture-only MVP for the PUP EN24C T2R4 radar on Linux.
//
// Single-binary deployment: parses an Intel HEX firmware blob, downloads it
// to the FX2LP via the standard Cypress 0xA0 loader, configures sawtooth FMCW,
// captures raw IQ for N seconds, writes <out>.bin + <out>.json.
//
// Usage:
//   pupradar_capture --duration 5 --out /tmp/cap [--firmware /path/to.hex]
//                    [--post-fw-vid 0x04B4 --post-fw-pid 0x8613]
//                    [--fc-low 24.0e9 --fc-high 24.25e9]
//                    [--sweep-time 2 --samp-num 1 --tx 1 --rx 1]
//                    [--reenum-timeout-ms 5000 --bulk-timeout-ms 2000]
//
// Defaults: 24.0..24.25 GHz, 1 ms sweep, max samples, Tx1+Rx1.
// On first hardware test, run `lsusb` after firmware load to discover the
// post-firmware VID/PID and pass them via --post-fw-vid/--post-fw-pid.

#include "pupradar/IqWriter.hpp"
#include "pupradar/Protocol.hpp"
#include "pupradar/RadarSession.hpp"
#include "pupradar/UsbDevice.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void printUsage(const char* prog) {
    std::cout <<
        "Usage: " << prog << " --out <basename> [options]\n"
        "\n"
        "Captures raw IQ, following the same sequence as the MATLAB\n"
        "run_GetComplexData.m: PUPradar_initiating -> Send_Basic_Parameter ->\n"
        "Send_PLL_Sawtooth -> GetComplexData. The options below are exactly the\n"
        "makeRadarHandles() parameters; nothing else is adjustable.\n"
        "\n"
        "Options:\n"
        "  --out <basename>          Output basename (writes <basename>.bin and .json)\n"
        "  --sweep-time-index <1-5>  1=0.5ms 2=1ms 3=2ms 4=4ms 5=8ms (default 1)\n"
        "  --transmitter <name>      Tx1 | Tx2 | Tx1.Tx2 (default Tx1)\n"
        "  --receiver <name>         Rx1 | Rx2 | Rx3 | Rx4 | Rx1.Rx2 | Rx3.Rx4 |\n"
        "                            Rx1.Rx2.Rx3.Rx4 (default Rx1)\n"
        "  --sampling-index <1-4>    1=BASN, 2=BASN/2, 3=BASN/4, 4=BASN/8 (default 1).\n"
        "                            Samples per sweep follow from the sweep time and\n"
        "                            receiver count; they are not set directly.\n"
        "  --num-sweeps <n>          Sweeps to acquire (default 128)\n"
        "  --fc-low <Hz>             Chirp start frequency (default 24.0e9)\n"
        "  --fc-high <Hz>            Chirp stop frequency (default 24.25e9)\n"
        "  --firmware <path>         Path to SDR_USB_FW.hex (default: ./firmware/SDR_USB_FW.hex)\n"
        "  -h, --help                Show this help\n";
}

/// Transmitter name -> (command value, channel count).
/// Mirrors makeRadarHandles / PUPradarGUI.m:2000-2021.
bool parseTransmitter(const std::string& s, std::uint8_t& value, int& count) {
    if (s == "Tx1")     { value = 1; count = 1; return true; }
    if (s == "Tx2")     { value = 2; count = 1; return true; }
    if (s == "Tx1.Tx2") { value = 3; count = 2; return true; }
    return false;
}

/// Receiver name -> (command value, channel count).
/// Mirrors makeRadarHandles / PUPradarGUI.m:2026-2075.
bool parseReceiver(const std::string& s, std::uint8_t& value, int& count) {
    if (s == "Rx1")             { value =  1; count = 1; return true; }
    if (s == "Rx2")             { value =  2; count = 1; return true; }
    if (s == "Rx3")             { value =  4; count = 1; return true; }
    if (s == "Rx4")             { value =  8; count = 1; return true; }
    if (s == "Rx1.Rx2")         { value =  3; count = 2; return true; }
    if (s == "Rx3.Rx4")         { value = 12; count = 2; return true; }
    if (s == "Rx1.Rx2.Rx3.Rx4") { value = 15; count = 4; return true; }
    return false;
}

class Args {
public:
    explicit Args(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "-h" || a == "--help") { help_ = true; continue; }
            if (a.rfind("--", 0) != 0) {
                throw std::runtime_error("Unexpected positional argument: " + a);
            }
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + a);
            }
            kv_[a.substr(2)] = argv[++i];
        }
    }
    bool        help() const { return help_; }
    bool        has(const std::string& k) const { return kv_.count(k) != 0; }
    std::string get(const std::string& k, const std::string& def = "") const {
        auto it = kv_.find(k); return it == kv_.end() ? def : it->second;
    }
    double      getDouble(const std::string& k, double def) const {
        return has(k) ? std::stod(get(k)) : def;
    }
    int         getInt(const std::string& k, int def) const {
        return has(k) ? std::stoi(get(k), nullptr, 0) : def;
    }
    unsigned int getUint(const std::string& k, unsigned int def) const {
        return has(k) ? static_cast<unsigned int>(std::stoul(get(k), nullptr, 0)) : def;
    }
private:
    bool help_ = false;
    std::map<std::string, std::string> kv_;
};

}  // namespace

int main(int argc, char** argv) try {
    Args args(argc, argv);
    if (args.help()) { printUsage(argv[0]); return 0; }
    if (!args.has("out")) {
        std::cerr << "Error: --out is required.\n\n";
        printUsage(argv[0]);
        return 2;
    }

    // Every option below is a makeRadarHandles() parameter. Operational
    // details the MATLAB path does not expose either (USB IDs, timeouts,
    // re-enumeration budget) are fixed constants further down.
    const std::string transmitter = args.get("transmitter", "Tx1");
    const std::string receiver    = args.get("receiver",    "Rx1");

    pupradar::CaptureConfig cfg;
    cfg.f_low_hz       = args.getDouble("fc-low",  24.0e9);
    cfg.f_high_hz      = args.getDouble("fc-high", 24.25e9);
    cfg.sweep_time_idx = args.getInt("sweep-time-index", 1);
    cfg.samp_num_idx   = args.getInt("sampling-index",   1);
    cfg.num_sweeps     = args.getInt("num-sweeps",     128);

    int n_tx = 0, n_rx = 0;
    if (!parseTransmitter(transmitter, cfg.tx_mask, n_tx)) {
        std::cerr << "Error: --transmitter must be Tx1, Tx2 or Tx1.Tx2 (got '"
                  << transmitter << "').\n";
        return 2;
    }
    if (!parseReceiver(receiver, cfg.rx_mask, n_rx)) {
        std::cerr << "Error: --receiver must be Rx1..Rx4, Rx1.Rx2, Rx3.Rx4 or "
                     "Rx1.Rx2.Rx3.Rx4 (got '" << receiver << "').\n";
        return 2;
    }
    if (cfg.sweep_time_idx < 1 || cfg.sweep_time_idx > 5) {
        std::cerr << "Error: --sweep-time-index must be 1..5.\n";
        return 2;
    }
    if (cfg.samp_num_idx < 1 || cfg.samp_num_idx > 4) {
        std::cerr << "Error: --sampling-index must be 1..4.\n";
        return 2;
    }
    if (cfg.num_sweeps < 1) {
        std::cerr << "Error: --num-sweeps must be >= 1.\n";
        return 2;
    }
    if (cfg.f_high_hz <= cfg.f_low_hz) {
        std::cerr << "Error: --fc-high must be above --fc-low.\n";
        return 2;
    }

    const std::string out_base = args.get("out");
    const std::string firmware = args.get("firmware", "firmware/SDR_USB_FW.hex");

    // Fixed operational constants — the MATLAB path hard-codes these too.
    // usbcheckchip checks vID 1204 (0x04B4) / pID 34323 (0x8613); see
    // PUPradar_initiating.m.
    constexpr unsigned int kReenumTimeoutMs = 5000;
    std::vector<pupradar::IUsbBackend::VidPid> post_fw{{0x04B4, 0x8613}};

    // Settling delay after each command packet. MATLAB gets milliseconds of
    // this for free — miniradarputdata constructs and destroys a whole
    // CCyUSBDevice per command (miniradarputdata.cpp:35-41) — whereas libusb
    // writes all 22 back to back over a persistent handle. Raise this if the
    // board ever ignores a parameter.
    constexpr unsigned int kCommandDelayUs = 0;

    const int samples_per_sweep =
        pupradar::samplesPerSweep(cfg.sweep_time_idx, cfg.samp_num_idx, n_rx);
    const double sweep_time_s = pupradar::sweepTimeFromIndex(cfg.sweep_time_idx);

    std::cout << "[pupradar] firmware: " << firmware << "\n";
    std::cout << "[pupradar] out: "      << out_base << ".bin / .json\n";
    std::cout << "[pupradar] config: "   << n_tx << " Tx / " << n_rx << " Rx ("
              << transmitter << ", " << receiver << "), "
              << sweep_time_s * 1e3 << " ms sweep, "
              << samples_per_sweep  << " samples/sweep/Rx, "
              << cfg.num_sweeps     << " sweeps\n";
    std::cout << "[pupradar] chirp: "    << cfg.f_low_hz/1e9 << ".."
              << cfg.f_high_hz/1e9      << " GHz\n";

    pupradar::UsbDevice    usb;
    pupradar::RadarSession session(usb, firmware);
    session.setCommandDelayUs(kCommandDelayUs);

    // Always the full bring-up with a firmware download, matching
    // PUPradar_initiating.m — which run_GetComplexData.m calls on every run.
    // The Cypress loader holds the 8051 in reset while it writes, so the board
    // starts from a known state and cannot carry stale sweeps in its FIFO.
    std::cout << "[pupradar] initializing (open → claim → firmware → reopen)…\n";
    session.initialize(post_fw, kReenumTimeoutMs);

    std::printf("[pupradar] board info (uint16 words, hex): ");
    for (auto w : session.lastBoardInfo()) std::printf("%04X ", w);
    std::printf("\n");

    const pupradar::BoardInfo info = pupradar::decodeBoardInfo(session.lastBoardInfo());
    if (info.valid) {
        std::printf("[pupradar] model:        %s\n",  info.model_name);
        std::printf("[pupradar] freq_band:    %u  num_tx: %u  num_rx: %u\n",
                    info.freq_band, info.num_tx, info.num_rx);
        std::printf("[pupradar] antenna_type: %u  version: %u\n",
                    info.antenna_type, info.version);
    } else {
        std::printf("[pupradar] warning: FA05 signature not found in board info\n");
    }

    pupradar::IqWriter writer(out_base);
    auto sink = [&writer](const std::uint8_t* d, std::size_t n) {
        writer.write(d, n);
    };

    // captureIq configures (Send_Basic_Parameter + Send_PLL_Sawtooth) and then
    // reads, in that order — no separate configure() call here.
    std::cout << "[pupradar] configuring and capturing " << cfg.num_sweeps
              << " sweeps (" << cfg.num_sweeps * sweep_time_s << " s)…\n";
    auto md = session.captureIq(cfg, sink);
    writer.finalize(md);

    std::cout << "[pupradar] done: " << md.bytes_captured << " bytes in "
              << md.reads_completed  << " reads ("
              << md.actual_duration_s << " s).\n";
    std::cout << "[pupradar] wrote: " << writer.binPath()
              << " + "                << writer.jsonPath() << "\n";

    session.shutdown();
    return 0;
} catch (const std::exception& e) {
    std::cerr << "[pupradar] error: " << e.what() << "\n";
    return 1;
}
