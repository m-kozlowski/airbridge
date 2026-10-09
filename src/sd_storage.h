#pragma once

#include <stdint.h>
#include <stddef.h>
#include <type_traits>

namespace fs {
class FS;
class File;
}

namespace SdStorage {

constexpr size_t READ_CHUNK_BYTES = 4096;

enum class Mode : uint8_t { Local, ToUsb, Usb, ToLocal };

struct Status {
    bool supported;
    bool mounted;
    uint64_t card_bytes;
    uint64_t total_bytes;
    uint64_t used_bytes;
    char error[48];
    Mode mode = Mode::Local;
    bool usb_supported = false;
    bool usb_can_stop = false;
};

// Binary storage records use little-endian fields regardless of alignment.
inline uint16_t get_le16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(data[1]) << 8;
}

inline uint32_t get_le32(const uint8_t *data) {
    return static_cast<uint32_t>(data[0]) |
           static_cast<uint32_t>(data[1]) << 8 |
           static_cast<uint32_t>(data[2]) << 16 |
           static_cast<uint32_t>(data[3]) << 24;
}

inline void put_le16(uint8_t *data, uint16_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
}

inline void put_le32(uint8_t *data, uint32_t value) {
    data[0] = static_cast<uint8_t>(value);
    data[1] = static_cast<uint8_t>(value >> 8);
    data[2] = static_cast<uint8_t>(value >> 16);
    data[3] = static_cast<uint8_t>(value >> 24);
}

bool write_exact(fs::File &file, const uint8_t *data, size_t size);

// Boot coordinator only, before init or any worker/network/recorder starts.
// Replaces the card partition table and creates one empty FAT32 partition.
// Logical format, not secure erase. Leaves the card unmounted; false on failure
// or unsupported builds. Once init is attempted, formatting is forbidden.
bool factory_format();
void init();
bool mounted();
fs::FS *filesystem();
void refresh_usage();
void get_status(Status &out);
const char *state_name(const Status &status);

// Admission only; the SD worker drains local owners before changing media.
bool request_usb(bool enabled, const char **error = nullptr);
bool local_access_allowed();

// USB transport callbacks. Buffer and completion remain owned until done.
bool usb_transfer(bool write, uint32_t sector, uint8_t *buffer, size_t size,
                  void (*done)(bool));
void usb_host_changed(bool connected);
void usb_ejected();

// Explicit rename/delete steps, including partially completed jobs.
// Recorder/catalog, private cache and mount/card changes are separate.
uint32_t files_revision();
void notify_files_changed();

// Recorder/recovery only. Waits for current local I/O, never for a network
// consumer; acquisition invalidates all previously admitted background work.
bool acquire();
// Maintenance never revokes an auxiliary session.
bool try_acquire();
void release();

class Session {
public:
    // One auxiliary operation at a time. Copies borrow the token; the owner
    // must end it after closing its Readers. Recorder acquisition revokes it.
    bool begin();
    void end();
    bool valid() const;

    // Blocking task API, not for HTTP/RX callbacks. The operation must finish
    // one bounded filesystem step and must not retain File objects or do UART
    // or network work. Caller context remains alive until completion.
    bool run(bool (*operation)(fs::FS &, void *), void *context) const;
    template <typename F> bool run(F &&operation) const {
        return run([](fs::FS &fs, void *context) {
            return (*static_cast<typename std::remove_reference<F>::type *>(context))(fs);
        }, &operation);
    }

private:
    uint32_t generation_ = 0;
};

class Reader {
public:
    struct Entry {
        char name[256];
        uint64_t size;
        int64_t modified;
        bool directory;
    };
    Reader() = default;
    ~Reader() { close(); }
    Reader(const Reader &) = delete;
    Reader &operator=(const Reader &) = delete;
    bool open(const Session &session, const char *path, bool directory = false);
    bool next(Entry &entry, bool &end);
    // Absolute file offset through UINT32_MAX, including EOF; failure keeps offset.
    bool seek(uint64_t offset);
    size_t read(uint8_t *data, size_t length);
    void close();
    uint64_t size() const { return size_; }
    int64_t modified() const { return modified_; }
    explicit operator bool() const { return handle_ != 0; }

private:
    Session session_;
    uint32_t handle_ = 0;
    uint64_t size_ = 0;
    int64_t modified_ = 0;
    uint64_t offset_ = 0;
};

}  // namespace SdStorage
