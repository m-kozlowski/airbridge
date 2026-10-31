#pragma once

#include <stddef.h>
#include <stdint.h>
#include <esp_partition.h>
#include <functional>
#include "resmed_firmware_image.h"

namespace FileUpload {

constexpr size_t MAX_IMAGE_BYTES = 1024 * 1024;
constexpr size_t MAX_FILE_BYTES = UINT32_MAX;
constexpr size_t MAX_CHUNK_BYTES = 4096;
enum class Destination : uint8_t { Sd, Staging };
enum class State : uint8_t { Idle, Receiving, Finalizing, Ready, Failed, Cancelled };
enum class Result : uint8_t {
    Accepted, Busy, Unsupported, Unavailable, InvalidName, InvalidSize,
    UnknownId, InvalidChunk, QueueFull, NoMemory, Closed
};

struct Status {
    uint32_t revision = 0;
    uint32_t id = 0;
    Destination destination = Destination::Sd;
    State state = State::Idle;
    uint32_t size = 0;
    uint32_t received = 0;
    uint32_t written = 0;
    bool complete = false;
    bool source_ready = false;
    bool cancel_allowed = false;
    char path[256] = {};
    char error[48] = {};
    ResmedFirmwareImage::Result image;
    bool bootloader = false;
    bool overwrite = false;
    bool repository = false;
};

struct Source {
    const esp_partition_t *partition = nullptr;
    uint32_t size = 0;
    uint32_t token = 0;
};

void init();
// Callback-safe admission/copy/cancel/status: no SD or partition work here.
// Index is the byte offset (not chunk ordinal); order must be contiguous.
// A false submission is terminal for that ID, including queue saturation.
// final means input accepted, NOT durable completion: observe complete/state.
// size=0 accepts an unknown multipart size, bounded by the destination limit.
// bootloader explicitly selects the 16KiB canonical recovery destination. It
// requires a supported matching BID and valid CRC, not exact dump-build bytes;
// the caller must obtain the user's patched-bootloader intent/consent.
// SD uses an absolute path and the common storage writer, including repository
// files. Staging uses a basename. Overwrite applies only to SD. Ordinary SD
// content is not inspected as firmware. Empty SD files are allowed.
Result begin_upload(Destination destination, const char *path, size_t size,
                    uint32_t &id, bool bootloader = false, bool overwrite = false);
Result submit_chunk(uint32_t id, size_t index, const uint8_t *data,
                    size_t length, bool final);
bool cancel(uint32_t id);
// Transport failure/disconnect may abandon in-flight input, never discard a
// Ready staging source. The test and transition share the owner's lock.
bool cancel_input(uint32_t id);
void get_status(Status &out);
bool copy_status(uint32_t id, Status &out);
// One indexed HTTP chunk in flight, <=MAX_CHUNK_BYTES. Defer its HTTP response
// until this notification says its exclusive byte end has DRAINED. The final
// chunk waits for publication and release of SD as well. Only then
// may the client submit the next chunk; no polling or receive-window-sized RAM.
// Accepted registers exactly one notification (possibly immediate); it runs
// outside owner locks and may only publish a prepared response or signal work.
// Captures must own/weakly-reference response state, never a request pointer.
using Drained = std::function<void(bool success)>;
Result when_drained(uint32_t id, size_t end, Drained notify);

// Staging Ready retains the OtaManager manual-upload reservation and partition
// until the engine releases this exact token. Image inspection belongs to
// the programmer, not ordinary upload. Source has no owning allocation.
// SD uploads release their buffers and session before publishing complete.
bool copy_source(uint32_t id, Source &out);
// Atomically promotes the manual-upload lease to ResMed ownership. A claimed
// token cannot be cancelled/replaced; the engine ends its promoted OTA lease.
bool claim_source(uint32_t id, Source &out, const char **error = nullptr);
bool release_source(uint32_t token);
const char *result_name(Result result);

}  // namespace FileUpload
