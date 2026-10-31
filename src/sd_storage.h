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
constexpr size_t WRITE_CHUNK_BYTES = 4096;

enum class Mode : uint8_t { Local, ToUsb, Usb, ToLocal };
enum class FormatResult : uint8_t { None, Succeeded, Failed };
enum class CardType : uint8_t { Unknown, Sd, Sdhc, Mmc, Sdio };
enum class Bus : uint8_t { None, Sdmmc, Spi };
// Firmware is exclusive and remains valid under the ResMed owner's OTA claim.
// Only ResmedOta may request it; ordinary sessions still stop during raw UART.
enum class Access : uint8_t { Background, ReadOnly, Exclusive, Firmware };
enum class IoEvent : uint8_t { Waiting, WaitingForReader, Complete };

struct SpiError {
    bool valid = false;
    bool write = false;
    uint8_t drive = 0;
    uint8_t command = 0xff;
    uint32_t argument = 0;
    uint32_t sector = 0;
    uint32_t count = 0;
    uint32_t occurred_ms = 0;
    uint32_t elapsed_ms = 0;
    int response = -1;
    int disk_result = 0;
    char phase[16] = {};
};

struct Status {
    bool supported;
    bool mounted;
    uint64_t card_bytes;
    uint64_t total_bytes;
    uint64_t used_bytes;
    char error[96];
    Mode mode = Mode::Local;
    bool usb_supported = false;
    // Boot-time format outcome, independent of subsequent mount/USB errors.
    FormatResult format_result = FormatResult::None;
    char format_error[48] = {};
    // Valid only for the initialized local or USB connection; zero means unknown.
    bool bus_active = false;
    uint8_t bus_width = 0;
    uint32_t bus_clock_khz = 0;
    // Configured limit remains available while the connection is inactive.
    uint32_t bus_clock_limit_khz = 0;
    CardType card_type = CardType::Unknown;
    Bus bus = Bus::None;
    // First SPI fault is retained until reboot, even if a retry recovered it.
    SpiError spi_error;
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
bool valid_path(const char *path);
bool valid_write_path(const char *path);

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
const char *format_result_name(FormatResult result);
const char *card_type_name(CardType type);
const char *bus_name(Bus bus);

// Admission only; the SD worker drains admitted I/O before changing media.
// Disabling USB also works with a connected host; its unwritten cache is lost.
bool request_usb(bool enabled, const char **error = nullptr);
bool local_access_allowed();

// USB transport callbacks. Buffer and completion remain owned until done.
bool usb_transfer(bool write, uint32_t sector, uint8_t *buffer, size_t size,
                  void (*done)(bool));
void usb_ejected();

// External file changes are scoped to the owners that consume those paths.
// Recorder/catalog commits and private cache writes retain their own revisions.
enum class FileScope : uint8_t { All, Catalog, Recording, Reports, Export, Firmware, Count };
uint32_t files_revision(FileScope scope = FileScope::All);
// Canonical absolute paths; includes descendants and affected parent directories.
// Rename supplies both paths. Omit paths when the change extent is unknown.
// Unknown changes and successful remounts invalidate all scopes.
void notify_files_changed(const char *path = nullptr, const char *destination = nullptr);

// Recorder/recovery only. Waits for current local I/O, never for a network
// consumer; acquisition invalidates all previously admitted background work.
bool acquire();
// Maintenance never revokes an auxiliary session. reserve_next keeps the next
// free slot for this caller after a miss, through all steps of its maintenance
// pass. finish_maintenance releases an admitted pass; an ungranted reservation
// remains for retry. cancel_acquire abandons it even before admission.
// Background owners checkpoint; admitted readers finish before maintenance.
bool try_acquire(bool reserve_next = false);
void finish_maintenance();
void cancel_acquire();
bool maintenance_pending();
// Auxiliary work is intentionally waiting for an admitted download to finish.
bool waiting_for_readers();
// Nonblocking admission hint for the reserving owner's existing work loop.
// A true result still requires try_acquire; it does not take the card.
bool maintenance_ready();
void release();

class Session {
public:
    // One background session and two interactive readers may coexist. A read
    // root protects a download's file/subtree from mutation, not all SD work.
    // Exclusive access reserves admission until other sessions finish. Call
    // end even after failed admission to abandon its reservation/FIFO ticket.
    // Background tickets expire after 1.5 s without begin. Copies borrow the
    // token; the owner ends it after its Readers/Writer. Recording revokes all.
    bool begin(Access access = Access::Background, const char *read_root = nullptr);
    void end();
    bool valid() const;
    // Cooperative checkpoint hint for background work, not revocation. Admitted
    // readers finish their whole transfer; recording and USB still revoke them.
    // A waiting background peer requests handoff after a 100 ms grant. A pending
    // maintenance reservation takes precedence even during that useful turn.
    bool yield_requested() const;
    // Change the current protected file without releasing reader admission.
    // Close the old Reader first. Null clears the scope for directory metadata.
    // False while an existing Writer owns the new path; the caller may retry.
    bool protect_read(const char *path);

    // One-shot hint from the SD executor when a queued background turn can
    // start. Only signal the owner's existing worker; no I/O or blocking here.
    // begin still checks admission. The callback/context must live until end
    // or the pending ticket expires. Set before begin; copies borrow the token.
    using AdmissionNotify = void (*)(void *context);
    void set_admission_notify(AdmissionNotify notify, void *context = nullptr) {
        admission_notify_ = notify;
        admission_context_ = context;
    }

    // Runs on the caller, never the executor. WaitingForReader is an intentional
    // pause, unlike a queued/slow SD operation. Complete marks successful I/O.
    // Only observe owner state; do not submit SD work here.
    // The context must outlive this session's I/O, including live-handle close.
    using IoObserver = void (*)(void *context, IoEvent event, const char *write_path);
    void set_io_observer(IoObserver observer, void *context = nullptr) {
        io_observer_ = observer;
        io_context_ = context;
    }

    // Blocking task API, not for HTTP/RX callbacks. The operation must finish
    // one bounded read/metadata step and must not retain File objects or do UART
    // or network work. Caller context remains alive until completion. Mutations
    // use mutate so existing downloads keep their stable source.
    bool run(bool (*operation)(fs::FS &, void *), void *context) const;
    template <typename F> bool run(F &&operation) const {
        return run([](fs::FS &fs, void *context) {
            return (*static_cast<typename std::remove_reference<F>::type *>(context))(fs);
        }, &operation);
    }

    // Waits for overlapping download scopes; a rename protects both paths.
    bool mutate(const char *path, const char *destination,
                bool (*operation)(fs::FS &, void *), void *context) const;
    template <typename F> bool mutate(const char *path, const char *destination,
                                      F &&operation) const {
        return mutate(path, destination, [](fs::FS &fs, void *context) {
            return (*static_cast<typename std::remove_reference<F>::type *>(context))(fs);
        }, &operation);
    }
    template <typename F> bool mutate(const char *path, F &&operation) const {
        return mutate(path, nullptr, operation);
    }

    // Create missing parents of a validated absolute file path (<256 bytes).
    // One directory per executor step. Existing parents must be directories.
    // Failure may leave empty parents; each creation publishes a file revision.
    bool ensure_parent(const char *path) const;

    // Publish a finished temporary file. Overwrite retains the old file at
    // backup until rename succeeds, and restores it on a failed rename.
    // FAT is not power-loss atomic; interrupted replacement may leave backup.
    const char *publish_file(const char *temporary, const char *path,
                             bool overwrite, const char *backup) const;

private:
    uint32_t generation_ = 0;
    Access access_ = Access::Background;
    AdmissionNotify admission_notify_ = nullptr;
    void *admission_context_ = nullptr;
    IoObserver io_observer_ = nullptr;
    void *io_context_ = nullptr;
    friend class Reader;
    friend class Writer;
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
    // One exact offset read, at most READ_CHUNK_BYTES, in one executor step.
    // Success consumes the requested range. A short read returns false but
    // consumes its actual bytes; rejected/failed seek leaves offset unchanged.
    bool read_at(uint64_t offset, uint8_t *data, size_t length);
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
    // Stream position can differ from credited offset after a short read.
    uint64_t physical_offset_ = UINT64_MAX;
};

// One output and two background inputs, plus one input per interactive reader.
// For replaceable files: create/truncate a temporary path, finish, then publish
// via Session::mutate. Revocation closes it before recorder/USB access begins.
class Writer {
public:
    enum class Mode : uint8_t { Sequential, Readback };

    Writer() = default;
    ~Writer() { close(); }
    Writer(const Writer &) = delete;
    Writer &operator=(const Writer &) = delete;
    // Nonzero append_at resumes a private build only at its exact flushed size.
    // truncate_tail opts a caller-owned private scratch path into discarding an
    // uncommitted tail, even at zero. Missing/short files fail without extension.
    // append_at must fit UINT32_MAX, and off_t when a tail needs truncating.
    bool open(const Session &session, const char *path, uint64_t append_at = 0,
              bool truncate_tail = false, Mode mode = Mode::Sequential);
    bool write(const uint8_t *data, size_t length);
    // Readback mode only: one exact <=4 KiB read from the credited private
    // prefix, restoring the append position before returning. No durable sync.
    // A failed restore closes the writer; a short read never credits bytes.
    bool read_at(uint64_t offset, uint8_t *data, size_t length);
    // Make the completed prefix readable without closing the sequential writer.
    // Only private, unpublished build files may be read while still growing.
    bool flush(uint64_t expected_size);
    // Flush, check the complete length and close, including on failure.
    bool finish(uint64_t expected_size);
    void close();

private:
    Session session_;
    uint32_t handle_ = 0;
    uint64_t size_ = 0;
    Mode mode_ = Mode::Sequential;
};

}  // namespace SdStorage
