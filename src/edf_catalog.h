#pragma once

#include <stddef.h>
#include <stdint.h>

#include "json_cursor.h"

namespace EdfCatalog {

enum EntryFlags : uint8_t {
    ENTRY_LIVE_COMPLETE = 1u << 0,
    ENTRY_IDENTIFICATION_READY = 1u << 1,
    ENTRY_STR_READY = 1u << 2,
};

struct Entry {
    char therapy_day[9];
    char file_prefix[16];
    uint8_t flags;
    uint32_t finalized_epoch;
    uint32_t str_digest;

    bool recording_ready() const {
        constexpr uint8_t required = ENTRY_LIVE_COMPLETE |
            ENTRY_IDENTIFICATION_READY;
        return (flags & required) == required;
    }
};

struct Status {
    bool supported;
    bool ready;
    uint32_t entries;
    uint32_t generation;
    char error[48];
};

struct Changes {
    Status catalog;
    uint32_t indexes[32];
    uint32_t count;
};

constexpr size_t SESSION_FILE_COUNT = 5;
constexpr const char *SESSION_SUFFIXES[SESSION_FILE_COUNT] = {"BRP", "PLD", "SAD", "EVE", "CSL"};
constexpr uint32_t DEFAULT_PAGE_SIZE = 32;
constexpr uint32_t MAX_PAGE_SIZE = 64;

struct SessionMetadata {
    char therapy_day[9] = {};
    char file_prefix[16] = {};
    int64_t start_seconds = 0, end_seconds = 0;
    uint32_t file_sizes[SESSION_FILE_COUNT] = {};
    uint8_t files_mask = 0, part_mask = 0;
    bool closed = false;
};

struct StrMetadata {
    bool present = false;
    uint64_t size = 0;
    int64_t modified = 0;
    char etag[11] = {};
};

struct Page {
    SessionMetadata *sessions = nullptr;
    uint32_t count = 0;
    StrMetadata str;
    char next_cursor[25] = {};
};

enum class PageResult { Ready, Busy, Unavailable, InvalidDay, InvalidCursor, InvalidLimit };

// RAM-only, inclusive native therapy-day floor (YYYYMMDD), or nullptr for all.
// A successful snapshot, including an empty one, must be released exactly once.
// Cursor is the exclusive last day/file_prefix in the selected direction.
// Limits apply before allocation.
PageResult copy_page(const char *since, Page &out, const char *cursor = nullptr,
                     uint32_t limit = DEFAULT_PAGE_SIZE, bool descending = false);
void release_page(Page &page);
// Live updates copy a fixed-size value; closing also updates the historical row.
void publish_session(const SessionMetadata &session);
// Standby worker only. Acquires/releases its own opportunistic SD access.
void refresh_metadata();

// Recorder/storage worker only, with SD ownership. Reloads after a filesystem
// revision (including remount); does not reconstruct a missing existing catalog.
void init();
// Data APIs also require SD ownership: stale snapshots reload before access.
bool commit(const Entry &entry);
bool read(uint32_t index, Entry &entry);
bool find(const char *file_prefix, Entry &entry);
bool snapshot_prefixes(char *out, size_t out_size, uint32_t &count);
// Caller releases the matching entries with Memory::free, including on errors.
bool snapshot_day(const char *day, Entry *&out, uint32_t &count);
// Memory-only publication; ready is false until a changed filesystem is loaded.
void get_status(Status &out);
uint32_t revision();
// False requires a full snapshot (startup or the change history was exceeded).
bool changes_since(uint32_t generation, Changes &out);

class PageCursor : private JsonCursor {
public:
    size_t read(const Page &page, char *out, size_t capacity);

private:
    bool next(const Page &page);
    uint32_t row_ = 0;
    uint8_t field_ = 0, file_ = 0;
    bool entries_ = false, files_ = false, done_ = false;
    char text_[96] = {};
};

}  // namespace EdfCatalog
