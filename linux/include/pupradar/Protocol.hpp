// Wire protocol constants for the PUP EN24C T2R4 radar.
// All values cross-checked against Mex src/*.cpp and Matlab src/PUPradarGUI.m.
// See progress/PROGRESS_01.md for the full derivation.
#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <vector>

namespace pupradar {

// USB identification
constexpr std::uint16_t kPreFirmwareVid = 0x04B4;  // Cypress
constexpr std::uint16_t kPreFirmwarePid = 0x8613;  // Unprogrammed FX2LP default

// Endpoints (firmware-defined; do not discover)
constexpr std::uint8_t kEpOutBulk = 0x02;  // EP2 OUT — host → MCU commands
constexpr std::uint8_t kEpInBulk  = 0x86;  // EP6 IN  — MCU → host data
constexpr std::size_t  kBulkOutPacketBytes = 512;  // FX2LP HS bulk max packet
constexpr int          kInterfaceNumber    = 0;
constexpr int          kAlternateSetting   = 1;

// Cypress 8051 firmware loader (FX2LP)
constexpr std::uint8_t  kFwLoaderRequest = 0xA0;
constexpr std::uint16_t kCpucsRegister   = 0xE600;
constexpr std::uint8_t  kCpucsReset      = 0x01;  // bit 0: 1=hold reset, 0=run

// Outbound command opcodes (high byte of the 16-bit command word)
namespace opcode {
constexpr std::uint8_t kBoardInfoRequest = 0xFA;  // param 0x00
constexpr std::uint8_t kModulation       = 0xE1;  // 0=Sawtooth, 3=CW
constexpr std::uint8_t kSweepTimeIndex   = 0xE2;  // 1..5 = 0.5/1/2/4/8 ms
constexpr std::uint8_t kSamplingNumIndex = 0xE3;  // 1..4 (table-indexed)
constexpr std::uint8_t kTxMask           = 0xE4;  // 1=Tx1, 2=Tx2, 3=Both
constexpr std::uint8_t kRxMask           = 0xE5;  // bitmask 1/2/4/8/3/12/15
// PLL register byte writes — three opcodes per 24-bit register, H/M/L
constexpr std::uint8_t kPll03H = 0xC1, kPll03M = 0xC2, kPll03L = 0xC3;  // start_N integer
constexpr std::uint8_t kPll04H = 0xC4, kPll04M = 0xC5, kPll04L = 0xC6;  // start_N fraction
constexpr std::uint8_t kPll0AH = 0xC7, kPll0AM = 0xC8, kPll0AL = 0xC9;  // step size
constexpr std::uint8_t kPll0CH = 0xCA, kPll0CM = 0xCB, kPll0CL = 0xCC;  // stop integer
constexpr std::uint8_t kPll0DH = 0xCD, kPll0DM = 0xCE, kPll0DL = 0xCF;  // stop fraction
constexpr std::uint8_t kPllSweepStopH = 0xD1, kPllSweepStopL = 0xD2;
}  // namespace opcode

// Modulation values (parameter byte for opcode 0xE1)
enum class Modulation : std::uint8_t {
    Sawtooth = 0,
    CW       = 3,
};

// Build a single 512-byte bulk-OUT packet that the MCU will interpret.
// The MCU reads only the first uint16 (little-endian); we still send a full
// max-packet because the hardware MEX did, and short writes have historically
// caused FX2LP framing trouble on some hosts.
std::array<std::uint8_t, kBulkOutPacketBytes>
makeCommandPacket(std::uint8_t opcode, std::uint8_t parameter);

// Convenience overloads
inline std::array<std::uint8_t, kBulkOutPacketBytes>
boardInfoPacket() { return makeCommandPacket(opcode::kBoardInfoRequest, 0x00); }

// Sawtooth FMCW PLL register pack — given low/high frequency in Hz and
// sweep-time index, produces the ordered list of opcode/byte pairs that need
// to be sent in the same order as the MATLAB Send_PLL_Sawtooth function.
//
// The math is the BGT24 mixer ratio (F_PLL = F_Tx / 16) with a 50 MHz reference,
// 24-bit fractional N. Full derivation in progress/PROGRESS_01.md.
struct PllCommand { std::uint8_t opcode; std::uint8_t param; };
std::vector<PllCommand>
buildSawtoothPllCommands(double f_low_hz, double f_high_hz, int sweep_time_index);

// Translate sweep-time and sampling-number indices to physical values, mirroring
// SetActiveParameters() in the GUI (lines 1982..2253).
double sweepTimeFromIndex(int index_1_5);                 // → seconds
int    samplesPerSweep(int sweep_time_idx, int sn_idx, int num_rx);
int    txCount(std::uint8_t tx_mask);
int    rxCount(std::uint8_t rx_mask);

}  // namespace pupradar
