#include "pupradar/UsbDevice.hpp"

#include <libusb-1.0/libusb.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

namespace pupradar {

namespace {

[[noreturn]] void throwLibusb(const char* what, int rc) {
    std::ostringstream oss;
    oss << what << ": " << libusb_error_name(rc) << " (" << rc << ")";
    throw UsbError(oss.str());
}

// Returns a human-readable list of every USB device currently visible,
// e.g. "04B4:00F3, 1D6B:0002". Used to self-diagnose renumeration failures.
std::string listVisibleDevices(libusb_context* ctx) {
    libusb_device** list = nullptr;
    ssize_t cnt = libusb_get_device_list(ctx, &list);
    if (cnt < 0) return "(could not enumerate)";

    std::ostringstream oss;
    bool first = true;
    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device_descriptor d{};
        if (libusb_get_device_descriptor(list[i], &d) != 0) continue;
        if (!first) oss << ", ";
        oss << std::hex << std::uppercase
            << std::setw(4) << std::setfill('0') << d.idVendor << ':'
            << std::setw(4) << std::setfill('0') << d.idProduct;
        first = false;
    }
    libusb_free_device_list(list, 1);
    return first ? "(none)" : oss.str();
}

libusb_device_handle* tryOpen(libusb_context* ctx,
                              std::uint16_t vid, std::uint16_t pid) {
    libusb_device** list = nullptr;
    ssize_t cnt = libusb_get_device_list(ctx, &list);
    if (cnt < 0) throwLibusb("libusb_get_device_list", static_cast<int>(cnt));

    libusb_device_handle* h = nullptr;
    for (ssize_t i = 0; i < cnt; ++i) {
        libusb_device_descriptor d{};
        if (libusb_get_device_descriptor(list[i], &d) != 0) continue;
        if (d.idVendor == vid && d.idProduct == pid) {
            int rc = libusb_open(list[i], &h);
            if (rc == 0) break;
            // Permission or busy — keep trying others in the unlikely case of duplicates.
            h = nullptr;
        }
    }
    libusb_free_device_list(list, 1);
    return h;
}

}  // namespace

UsbDevice::UsbDevice() {
    int rc = libusb_init(&ctx_);
    if (rc != 0) throwLibusb("libusb_init", rc);
}

UsbDevice::~UsbDevice() {
    close();
    if (ctx_) libusb_exit(ctx_);
}

void UsbDevice::open(std::uint16_t vid, std::uint16_t pid) {
    close();
    handle_ = tryOpen(ctx_, vid, pid);
    if (!handle_) {
        std::ostringstream oss;
        oss << "USB device " << std::hex << vid << ":" << pid << " not found";
        throw UsbError(oss.str());
    }
}

void UsbDevice::close() {
    if (handle_) {
        if (claimed_iface_ >= 0) {
            libusb_release_interface(handle_, claimed_iface_);
            claimed_iface_ = -1;
        }
        libusb_close(handle_);
        handle_ = nullptr;
    }
}

bool UsbDevice::isOpen() const { return handle_ != nullptr; }

void UsbDevice::claimInterface(int interface_number) {
    if (!handle_) throw UsbError("claimInterface: device not open");
    // Detach kernel driver if attached (Linux usbfs requirement).
    if (libusb_kernel_driver_active(handle_, interface_number) == 1) {
        int rc = libusb_detach_kernel_driver(handle_, interface_number);
        if (rc != 0 && rc != LIBUSB_ERROR_NOT_FOUND) {
            throwLibusb("libusb_detach_kernel_driver", rc);
        }
    }
    int rc = libusb_claim_interface(handle_, interface_number);
    if (rc != 0) throwLibusb("libusb_claim_interface", rc);
    claimed_iface_ = interface_number;
}

void UsbDevice::setAltSetting(int interface_number, int alt_setting) {
    if (!handle_) throw UsbError("setAltSetting: device not open");
    int rc = libusb_set_interface_alt_setting(handle_, interface_number, alt_setting);
    if (rc != 0) throwLibusb("libusb_set_interface_alt_setting", rc);
}

std::size_t UsbDevice::controlTransfer(std::uint8_t  bm,
                                       std::uint8_t  br,
                                       std::uint16_t wv,
                                       std::uint16_t wi,
                                       std::uint8_t* data,
                                       std::size_t   length,
                                       unsigned int  timeout_ms) {
    if (!handle_) throw UsbError("controlTransfer: device not open");
    if (length > 0xFFFFu) throw UsbError("controlTransfer: length > 65535");
    int rc = libusb_control_transfer(handle_, bm, br, wv, wi,
                                     data, static_cast<std::uint16_t>(length),
                                     timeout_ms);
    if (rc < 0) throwLibusb("libusb_control_transfer", rc);
    return static_cast<std::size_t>(rc);
}

std::size_t UsbDevice::bulkWrite(std::uint8_t  ep,
                                 const std::uint8_t* data,
                                 std::size_t   length,
                                 unsigned int  timeout_ms) {
    if (!handle_) throw UsbError("bulkWrite: device not open");
    int actual = 0;
    int rc = libusb_bulk_transfer(handle_, ep,
                                  const_cast<std::uint8_t*>(data),
                                  static_cast<int>(length), &actual, timeout_ms);
    if (rc != 0) throwLibusb("libusb_bulk_transfer (out)", rc);
    return static_cast<std::size_t>(actual);
}

std::size_t UsbDevice::bulkRead(std::uint8_t  ep,
                                std::uint8_t* data,
                                std::size_t   length,
                                unsigned int  timeout_ms) {
    if (!handle_) throw UsbError("bulkRead: device not open");
    int actual = 0;
    int rc = libusb_bulk_transfer(handle_, ep, data,
                                  static_cast<int>(length), &actual, timeout_ms);
    // On LIBUSB_ERROR_TIMEOUT, actual contains the bytes received before the
    // deadline — that data is valid. Return it so the capture loop can save it
    // and retry rather than aborting the entire capture.
    if (rc == 0 || (rc == LIBUSB_ERROR_TIMEOUT && actual > 0))
        return static_cast<std::size_t>(actual);
    if (rc == LIBUSB_ERROR_TIMEOUT)
        return 0;  // genuine timeout with no data — caller treats 0 as "retry"
    throwLibusb("libusb_bulk_transfer (in)", rc);
}

void UsbDevice::reopenAfterRenumeration(const VidPid* candidates,
                                        std::size_t   num_candidates,
                                        unsigned int  timeout_ms) {
    close();  // release old handle; chip will re-enumerate
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
    while (clock::now() < deadline) {
        for (std::size_t i = 0; i < num_candidates; ++i) {
            libusb_device_handle* h = tryOpen(ctx_, candidates[i].vid,
                                              candidates[i].pid);
            if (h) {
                handle_ = h;
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::ostringstream oss;
    oss << "reopenAfterRenumeration: device did not reappear within "
        << timeout_ms << " ms.\n"
        << "  Searched for:";
    for (std::size_t i = 0; i < num_candidates; ++i) {
        oss << " " << std::hex << std::uppercase
            << std::setw(4) << std::setfill('0') << candidates[i].vid << ':'
            << std::setw(4) << std::setfill('0') << candidates[i].pid;
    }
    oss << "\n  Currently visible: " << listVisibleDevices(ctx_)
        << "\n  Hint: pass the correct VID:PID above via --post-fw-vid / --post-fw-pid";
    throw UsbError(oss.str());
}

}  // namespace pupradar
