#include "storage_smb_client.h"

#include <Arduino.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <new>

#include <lwip/dns.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>

#include "debug_log.h"
#include "lwip_core_guard.h"
#include "string_util.h"

extern "C" {
#include "smb2/smb2.h"
#include "smb2/libsmb2.h"
}

namespace aircannect {

struct StorageSmbAsyncResult {
    bool finished = false;
    bool orphaned = false;
    int status = 0;
    void *result = nullptr;
};

struct StorageSmbPendingOperation {
    StorageSmbAsyncResult callback;
    smb2_stat_64 stat = {};
    uint32_t started_ms = 0;
};

struct StorageSmbDnsResult {
    // The client and lwIP each own a reference until detach/completion.
    std::atomic<unsigned> references{2};
    std::atomic<bool> finished{false};
    ip_addr_t address = {};
    int error = ERR_OK;

    void release() {
        if (references.fetch_sub(1, std::memory_order_acq_rel) == 1)
            delete this;
    }
};

namespace {

static constexpr int SMB_COMMAND_TIMEOUT_SECONDS = 15;
static constexpr uint32_t SMB_EVENT_LOOP_TIMEOUT_MS =
    SMB_COMMAND_TIMEOUT_SECONDS * 1000UL;

void smb_generic_cb(struct smb2_context *,
                    int status,
                    void *command_data,
                    void *private_data) {
    StorageSmbAsyncResult *cb =
        static_cast<StorageSmbAsyncResult *>(private_data);
    if (!cb) return;
    cb->status = status;
    cb->result = command_data;
    cb->finished = true;
    if (cb->orphaned) delete cb;
}

StorageSmbAsyncResult *smb_alloc_cb() {
    return new (std::nothrow) StorageSmbAsyncResult();
}

bool smb_queue_close_for_destroy(struct smb2_context *ctx, struct smb2fh *fh) {
    // destroy_context invokes queued PDU callbacks with SHUTDOWN; queue close
    // first so libsmb2's close_cb frees the file handle without a network wait.
    StorageSmbAsyncResult *cb = smb_alloc_cb();
    if (!cb) return false;
    cb->orphaned = true;
    const int rc = smb2_close_async(ctx, fh, smb_generic_cb, cb);
    if (rc < 0) {
        cb->orphaned = false;
        delete cb;
        return false;
    }
    return true;
}

bool is_path_not_found_error(const char *error) {
    return error &&
           (strstr(error, "STATUS_OBJECT_NAME_NOT_FOUND") ||
            strstr(error, "STATUS_OBJECT_PATH_NOT_FOUND") ||
            strstr(error, "STATUS_NO_SUCH_FILE") ||
            strstr(error, "PATH_NOT_FOUND") ||
            strstr(error, "No such file") ||
            strstr(error, "not found"));
}

bool is_path_not_found_status(uint32_t status) {
    return status == SMB2_STATUS_NO_SUCH_FILE ||
           status == SMB2_STATUS_OBJECT_NAME_NOT_FOUND ||
           status == SMB2_STATUS_OBJECT_PATH_NOT_FOUND ||
           status == SMB2_STATUS_OBJECT_PATH_INVALID;
}

bool is_path_not_found_status_hint(int status) {
    return status == -ENOENT || status == -ENOTDIR;
}

bool valid_endpoint_char(char c) {
    const unsigned char u = static_cast<unsigned char>(c);
    return u >= 0x20 && u < 0x7F && c != '\\';
}

bool split_server_authority(const char *server,
                            char *host,
                            size_t host_size,
                            const char **port_suffix) {
    if (!server || !server[0] || !host || host_size == 0 || !port_suffix) {
        return false;
    }

    *port_suffix = "";
    const char *host_start = server;
    size_t host_len = strlen(server);

    if (server[0] == '[') {
        const char *closing = strchr(server + 1, ']');
        if (!closing) return false;

        host_start = server + 1;
        host_len = static_cast<size_t>(closing - host_start);
        if (closing[1] != '\0') {
            if (closing[1] != ':' || closing[2] == '\0') return false;
            *port_suffix = closing + 1;
        }
    } else {
        const char *first_colon = strchr(server, ':');
        const char *last_colon = strrchr(server, ':');
        if (first_colon && first_colon == last_colon) {
            if (first_colon == server || first_colon[1] == '\0') return false;
            host_len = static_cast<size_t>(first_colon - server);
            *port_suffix = first_colon;
        }
    }

    if (host_len == 0 || host_len >= host_size) return false;
    memcpy(host, host_start, host_len);
    host[host_len] = '\0';
    return true;
}

bool append_path_segment(char *path,
                         size_t path_size,
                         const char *segment,
                         size_t segment_len) {
    if (!path || !segment || segment_len == 0) return false;
    const size_t len = strlen(path);
    const size_t slash = len == 0 ? 0 : 1;
    if (len + slash + segment_len >= path_size) return false;
    if (slash) path[len] = '/';
    memcpy(path + len + slash, segment, segment_len);
    path[len + slash + segment_len] = '\0';
    return true;
}

}  // namespace

struct StorageSmbDnsCallback {
    static void complete(const char *,
                         const ip_addr_t *address,
                         void *context) {
        auto *result = static_cast<StorageSmbDnsResult *>(context);
        if (address) result->address = *address;
        result->error = address ? ERR_OK : ERR_TIMEOUT;
        result->finished.store(true, std::memory_order_release);
        result->release();
    }
};

StorageSmbClient::~StorageSmbClient() {
    abort_connection();
}

void StorageSmbClient::set_error(char *error_out,
                                 size_t error_out_size,
                                 const char *error) const {
    copy_cstr(error_out, error_out_size, error ? error : "");
}

const char *StorageSmbClient::last_error() const {
    if (!ctx_) return "not_connected";
    const char *err = smb2_get_error(ctx_);
    return err && *err ? err : "smb_error";
}

uint32_t StorageSmbClient::last_status(int status_hint) const {
    if (status_hint != 0) {
        return static_cast<uint32_t>(status_hint);
    }
    return ctx_ ? static_cast<uint32_t>(smb2_get_nterror(ctx_)) : 0;
}

void StorageSmbClient::set_smb_error(char *error_out,
                                     size_t error_out_size,
                                     const char *operation,
                                     int status_hint) const {
    if (!error_out || error_out_size == 0) return;
    if (!operation || !*operation) operation = "smb";
    if (status_hint == -ECANCELED) {
        set_error(error_out, error_out_size, "preempted");
        return;
    }
    if (status_hint == -ETIMEDOUT) {
        set_error(error_out, error_out_size, "operation_timeout");
        return;
    }

    const char *err = ctx_ ? smb2_get_error(ctx_) : "";
    if (err && *err) {
        snprintf(error_out, error_out_size, "%s:%s", operation, err);
        return;
    }

    if (status_hint < 0 && status_hint > -4096 &&
        (!ctx_ || smb2_get_nterror(ctx_) == 0)) {
        const int local_errno = -status_hint;
        snprintf(error_out, error_out_size, "%s:errno_%d:%s",
                 operation, local_errno, strerror(local_errno));
        return;
    }

    const uint32_t status = last_status(status_hint);
    if (status != 0) {
        const char *name = nterror_to_str(status);
        if (name && *name && strcmp(name, "Unknown error") != 0) {
            snprintf(error_out, error_out_size, "%s:%s", operation, name);
        } else {
            snprintf(error_out, error_out_size, "%s:0x%08lx", operation,
                     static_cast<unsigned long>(status));
        }
        return;
    }

    snprintf(error_out, error_out_size, "%s:smb_error", operation);
}

bool StorageSmbClient::parse_endpoint(const char *endpoint,
                                      char *error_out,
                                      size_t error_out_size) {
    server_[0] = '\0';
    share_[0] = '\0';
    base_path_[0] = '\0';

    if (!endpoint || strncmp(endpoint, "//", 2) != 0) {
        set_error(error_out, error_out_size, "bad_endpoint");
        return false;
    }
    for (const char *p = endpoint; *p; ++p) {
        if (!valid_endpoint_char(*p)) {
            set_error(error_out, error_out_size, "bad_endpoint_char");
            return false;
        }
    }

    const char *host_start = endpoint + 2;
    const char *share_start = strchr(host_start, '/');
    if (!share_start || share_start == host_start) {
        set_error(error_out, error_out_size, "missing_smb_host");
        return false;
    }
    const size_t host_len = static_cast<size_t>(share_start - host_start);
    if (host_len >= sizeof(server_)) {
        set_error(error_out, error_out_size, "smb_host_too_long");
        return false;
    }
    memcpy(server_, host_start, host_len);
    server_[host_len] = '\0';

    share_start++;
    const char *base_start = strchr(share_start, '/');
    const size_t share_len =
        base_start ? static_cast<size_t>(base_start - share_start)
                   : strlen(share_start);
    if (share_len == 0 || share_len >= sizeof(share_)) {
        set_error(error_out, error_out_size, "bad_smb_share");
        return false;
    }
    memcpy(share_, share_start, share_len);
    share_[share_len] = '\0';

    if (base_start && base_start[1] != '\0') {
        base_start++;
        size_t base_len = strlen(base_start);
        while (base_len > 0 && base_start[base_len - 1] == '/') base_len--;
        if (base_len >= sizeof(base_path_)) {
            set_error(error_out, error_out_size, "smb_base_too_long");
            return false;
        }
        memcpy(base_path_, base_start, base_len);
        base_path_[base_len] = '\0';
    }

    return true;
}

bool StorageSmbClient::configure(const char *endpoint,
                                 const char *user,
                                 const char *password,
                                 char *error_out,
                                 size_t error_out_size) {
    abort_connection();
    if (!parse_endpoint(endpoint, error_out, error_out_size)) return false;
    copy_cstr(user_, sizeof(user_), user);
    copy_cstr(password_, sizeof(password_), password);

    set_error(error_out, error_out_size, "");
    return true;
}

void StorageSmbClient::publish_host_resolution(const void *address,
                                               int error) {
    const ip_addr_t *ip = static_cast<const ip_addr_t *>(address);
    char raw_address[IPADDR_STRLEN_MAX] = {};
    if (!ip || !ipaddr_ntoa_r(ip, raw_address, sizeof(raw_address))) {
        resolved_address_[0] = '\0';
        host_resolution_error_ = error ? error : ERR_VAL;
        host_resolution_state_.store(
            static_cast<uint8_t>(HostResolutionState::Error),
            std::memory_order_release);
        return;
    }

    if (IP_IS_V6(ip)) {
        const int written = snprintf(resolved_address_,
                                     sizeof(resolved_address_),
                                     "[%s]",
                                     raw_address);
        if (written <= 0 ||
            static_cast<size_t>(written) >= sizeof(resolved_address_)) {
            resolved_address_[0] = '\0';
            host_resolution_error_ = ERR_BUF;
            host_resolution_state_.store(
                static_cast<uint8_t>(HostResolutionState::Error),
                std::memory_order_release);
            return;
        }
    } else {
        copy_cstr(resolved_address_, sizeof(resolved_address_), raw_address);
    }

    host_resolution_error_ = ERR_OK;
    host_resolution_state_.store(
        static_cast<uint8_t>(HostResolutionState::Ready),
        std::memory_order_release);
}

void StorageSmbClient::reset_host_resolution() {
    if (dns_result_) {
        dns_result_->release();
        dns_result_ = nullptr;
    }
    resolving_server_[0] = '\0';
    resolved_address_[0] = '\0';
    resolved_server_[0] = '\0';
    host_resolution_started_ms_ = 0;
    host_resolution_error_ = ERR_OK;
    host_resolution_state_.store(
        static_cast<uint8_t>(HostResolutionState::Idle),
        std::memory_order_release);
}

StorageSmbOperationResult StorageSmbClient::resolve_host(
    char *error_out,
    size_t error_out_size) {
    if (!server_[0]) {
        set_error(error_out, error_out_size, "not_configured");
        return StorageSmbOperationResult::Error;
    }

    HostResolutionState state = static_cast<HostResolutionState>(
        host_resolution_state_.load(std::memory_order_acquire));
    if (state == HostResolutionState::Pending) {
        if (!dns_result_->finished.load(std::memory_order_acquire))
            return StorageSmbOperationResult::Waiting;
        publish_host_resolution(dns_result_->error == ERR_OK
                                    ? &dns_result_->address : nullptr,
                                dns_result_->error);
        dns_result_->release();
        dns_result_ = nullptr;
        state = static_cast<HostResolutionState>(
            host_resolution_state_.load(std::memory_order_acquire));
    }

    if (state != HostResolutionState::Idle &&
        strcmp(resolving_server_, server_) != 0) {
        reset_host_resolution();
        state = HostResolutionState::Idle;
    }

    if (state == HostResolutionState::Error) {
        char error[AC_STORAGE_ERROR_MAX] = {};
        snprintf(error,
                 sizeof(error),
                 "dns_lookup_failed:%d",
                 host_resolution_error_);
        set_error(error_out, error_out_size, error);
        return StorageSmbOperationResult::Error;
    }

    if (state == HostResolutionState::Ready) {
        char host[AC_STORAGE_SMB_ENDPOINT_HOST_MAX] = {};
        const char *port_suffix = "";
        if (!split_server_authority(server_,
                                    host,
                                    sizeof(host),
                                    &port_suffix)) {
            set_error(error_out, error_out_size, "bad_smb_host");
            return StorageSmbOperationResult::Error;
        }

        const int written = snprintf(resolved_server_,
                                     sizeof(resolved_server_),
                                     "%s%s",
                                     resolved_address_,
                                     port_suffix);
        if (written <= 0 ||
            static_cast<size_t>(written) >= sizeof(resolved_server_)) {
            set_error(error_out, error_out_size, "resolved_host_too_long");
            return StorageSmbOperationResult::Error;
        }

        Log::logf(CAT_EXPORT,
                  LOG_DEBUG,
                  "[SMB] resolved host=%s address=%s ms=%lu\n",
                  host,
                  resolved_address_,
                  static_cast<unsigned long>(
                      millis() - host_resolution_started_ms_));
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }

    char host[AC_STORAGE_SMB_ENDPOINT_HOST_MAX] = {};
    const char *port_suffix = "";
    if (!split_server_authority(server_,
                                host,
                                sizeof(host),
                                &port_suffix)) {
        set_error(error_out, error_out_size, "bad_smb_host");
        return StorageSmbOperationResult::Error;
    }

    dns_result_ = new (std::nothrow) StorageSmbDnsResult();
    if (!dns_result_) {
        set_error(error_out, error_out_size, "dns_out_of_memory");
        return StorageSmbOperationResult::Error;
    }
    copy_cstr(resolving_server_, sizeof(resolving_server_), server_);
    host_resolution_started_ms_ = millis();
    host_resolution_state_.store(
        static_cast<uint8_t>(HostResolutionState::Pending),
        std::memory_order_release);
    Log::logf(CAT_EXPORT, LOG_DEBUG, "[SMB] resolving host=%s\n", host);

    ip_addr_t address = {};
    err_t rc = ERR_OK;
    {
        LwipCoreGuard guard;
        rc = dns_gethostbyname_addrtype(
            host,
            &address,
            StorageSmbDnsCallback::complete,
            dns_result_,
            LWIP_DNS_ADDRTYPE_IPV4_IPV6);
    }

    if (rc == ERR_INPROGRESS) return StorageSmbOperationResult::Waiting;

    // Synchronous completion does not invoke the lwIP callback.
    dns_result_->release();
    dns_result_->release();
    dns_result_ = nullptr;
    if (rc == ERR_OK) {
        publish_host_resolution(&address, ERR_OK);
        return resolve_host(error_out, error_out_size);
    }
    publish_host_resolution(nullptr, rc);
    return resolve_host(error_out, error_out_size);
}

bool StorageSmbClient::configure_socket_options(
    int fd, char *error_out, size_t error_out_size) {
    if (fd < 0) {
        set_error(error_out, error_out_size, "socket_unavailable");
        return false;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || (!(flags & O_NONBLOCK) &&
                      fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0)) {
        set_error(error_out, error_out_size, "socket_nonblocking_failed");
        Log::logf(CAT_EXPORT,
                  LOG_ERROR,
                  "[SMB] failed to configure nonblocking socket fd=%d "
                  "errno=%d\n",
                  fd,
                  errno);
        return false;
    }
    if (configured_socket_ == fd) return true;

    const int one = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0) {
        Log::logf(CAT_EXPORT,
                  LOG_WARN,
                  "[SMB] setsockopt(TCP_NODELAY) errno=%d\n",
                  errno);
    }

    configured_socket_ = fd;
    return true;
}

bool StorageSmbClient::begin_operation(OperationKind kind,
                                       char *error_out,
                                       size_t error_out_size) {
    if (operation_) {
        if (operation_kind_ == kind) return true;
        set_error(error_out, error_out_size, "operation_busy");
        return false;
    }

    operation_ = new (std::nothrow) StorageSmbPendingOperation();
    if (!operation_) {
        set_error(error_out, error_out_size, "smb_operation_alloc");
        return false;
    }

    operation_kind_ = kind;
    operation_->started_ms = millis();
    return true;
}

StorageSmbOperationResult StorageSmbClient::poll_operation(
    OperationKind kind,
    int &status_out,
    const BackgroundOperationControl *control,
    char *error_out,
    size_t error_out_size) {
    status_out = 0;
    if (!operation_ || operation_kind_ != kind) {
        set_error(error_out, error_out_size, "operation_state_mismatch");
        return StorageSmbOperationResult::Error;
    }
    if (operation_->callback.finished) {
        status_out = operation_->callback.status;
        return StorageSmbOperationResult::Ready;
    }

    const uint32_t now_ms = millis();
    if (control) {
        const BackgroundOperationStop reason = control->stop_reason(now_ms);
        if (reason == BackgroundOperationStop::Aborted) {
            set_error(error_out, error_out_size, "preempted");
            abort_connection();
            return StorageSmbOperationResult::Error;
        }
        if (reason == BackgroundOperationStop::Deadline) {
            set_error(error_out, error_out_size, "operation_timeout");
            abort_connection();
            return StorageSmbOperationResult::Error;
        }
    }
    if (static_cast<uint32_t>(now_ms - operation_->started_ms) >=
        SMB_EVENT_LOOP_TIMEOUT_MS) {
        set_error(error_out, error_out_size, "operation_timeout");
        abort_connection();
        return StorageSmbOperationResult::Error;
    }

    const int fd = smb2_get_fd(ctx_);
    if (fd < 0) {
        set_error(error_out, error_out_size, "socket_unavailable");
        abort_connection();
        return StorageSmbOperationResult::Error;
    }
    if (!configure_socket_options(fd, error_out, error_out_size)) {
        abort_connection();
        return StorageSmbOperationResult::Error;
    }

    pollfd pfd = {};
    pfd.fd = fd;
    pfd.events = smb2_which_events(ctx_);
    const int poll_status = poll(&pfd, 1, 0);
    if (poll_status < 0) {
        set_smb_error(error_out, error_out_size, "poll",
                      errno ? -errno : -EIO);
        abort_connection();
        return StorageSmbOperationResult::Error;
    }

    if (pfd.revents) {
        // libsmb2 drains all immediately available socket work in one call.
        // Run at most one such call per export tick so core 0 can run idle.
        const int service_status = smb2_service(ctx_, pfd.revents);
        if (service_status < 0 && !operation_->callback.finished) {
            set_smb_error(error_out, error_out_size,
                          "service", service_status);
            abort_connection();
            return StorageSmbOperationResult::Error;
        }
    }

    if (!operation_ || !operation_->callback.finished) {
        return StorageSmbOperationResult::Waiting;
    }

    status_out = operation_->callback.status;
    return StorageSmbOperationResult::Ready;
}

void StorageSmbClient::clear_operation() {
    delete operation_;
    operation_ = nullptr;
    operation_kind_ = OperationKind::None;
}

StorageSmbOperationResult StorageSmbClient::step_connect(
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *operation) {
    if (connected_) return StorageSmbOperationResult::Ready;
    if (!server_[0] || !share_[0]) {
        set_error(error_out, error_out_size, "not_configured");
        return StorageSmbOperationResult::Error;
    }
    if (!resolved_server_[0]) {
        set_error(error_out, error_out_size, "host_not_resolved");
        return StorageSmbOperationResult::Error;
    }

    if (!operation_) {
        ctx_ = smb2_init_context();
        if (!ctx_) {
            set_error(error_out, error_out_size, "smb_context_alloc");
            Log::logf(CAT_EXPORT, LOG_ERROR,
                      "[SMB] context allocation failed\n");
            return StorageSmbOperationResult::Error;
        }

        smb2_set_timeout(ctx_, SMB_COMMAND_TIMEOUT_SECONDS);
        smb2_set_security_mode(ctx_, SMB2_NEGOTIATE_SIGNING_ENABLED);
        if (user_[0]) {
            smb2_set_user(ctx_, user_);
            smb2_set_password(ctx_, password_);
        }

        if (!begin_operation(OperationKind::Connect,
                             error_out, error_out_size)) {
            smb2_destroy_context(ctx_);
            ctx_ = nullptr;
            return StorageSmbOperationResult::Error;
        }

        Log::logf(CAT_EXPORT, LOG_DEBUG,
                  "[SMB] connecting //%s/%s\n", server_, share_);

        const int rc = smb2_connect_share_async(
            ctx_, resolved_server_, share_, nullptr,
            smb_generic_cb, &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Connect, status, operation,
        error_out, error_out_size);
    if (result != StorageSmbOperationResult::Ready) return result;

    clear_operation();
    if (status != 0) {
        char error[AC_STORAGE_ERROR_MAX] = {};
        set_smb_error(error, sizeof(error), "connect", status);
        set_error(error_out, error_out_size, error);
        Log::logf(CAT_EXPORT, LOG_DEBUG,
                  "[SMB] connect failed error=%s\n", error);
        smb2_destroy_context(ctx_);
        ctx_ = nullptr;
        return StorageSmbOperationResult::Error;
    }

    connected_ = true;
    set_error(error_out, error_out_size, "");
    return StorageSmbOperationResult::Ready;
}

StorageSmbOperationResult StorageSmbClient::step_disconnect(
    const BackgroundOperationControl *control) {
    if (!ctx_) {
        connected_ = false;
        writer_ = nullptr;
        return StorageSmbOperationResult::Ready;
    }
    if (writer_) {
        abort_connection();
        return StorageSmbOperationResult::Ready;
    }
    if (!connected_) {
        abort_connection();
        return StorageSmbOperationResult::Ready;
    }

    if (!operation_) {
        if (!begin_operation(OperationKind::Disconnect, nullptr, 0)) {
            abort_connection();
            return StorageSmbOperationResult::Ready;
        }

        const int rc = smb2_disconnect_share_async(
            ctx_, smb_generic_cb, &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Disconnect, status, control, nullptr, 0);
    if (result == StorageSmbOperationResult::Waiting) return result;
    if (result == StorageSmbOperationResult::Error) {
        return StorageSmbOperationResult::Ready;
    }

    clear_operation();
    smb2_destroy_context(ctx_);
    ctx_ = nullptr;
    connected_ = false;
    return StorageSmbOperationResult::Ready;
}

bool StorageSmbClient::make_remote_path(const char *absolute_path,
                                        char *out,
                                        size_t out_size) const {
    if (!absolute_path || absolute_path[0] != '/' || !out || out_size == 0) {
        return false;
    }
    const char *rel = absolute_path[1] ? absolute_path + 1 : "";
    int written = 0;
    if (base_path_[0] && rel[0]) {
        written = snprintf(out, out_size, "%s/%s", base_path_, rel);
    } else if (base_path_[0]) {
        written = snprintf(out, out_size, "%s", base_path_);
    } else {
        written = snprintf(out, out_size, "%s", rel);
    }
    return written >= 0 && static_cast<size_t>(written) < out_size;
}

StorageSmbOperationResult StorageSmbClient::step_stat(
    const char *remote_path,
    StorageSmbRemoteStat &out,
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *control) {
    out = StorageSmbRemoteStat();
    if (!connected_ || !ctx_) {
        set_error(error_out, error_out_size, "not_connected");
        return StorageSmbOperationResult::Error;
    }
    if (!remote_path || remote_path[0] == '\0') {
        out.exists = true;
        out.directory = true;
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }

    if (!operation_) {
        if (!begin_operation(OperationKind::Stat,
                             error_out, error_out_size)) {
            return StorageSmbOperationResult::Error;
        }

        const int rc = smb2_stat_async(
            ctx_, remote_path, &operation_->stat,
            smb_generic_cb, &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Stat, status, control,
        error_out, error_out_size);
    if (result != StorageSmbOperationResult::Ready) return result;

    const smb2_stat_64 native = operation_->stat;
    if (status == 0) {
        out.exists = true;
        out.directory = native.smb2_type == SMB2_TYPE_DIRECTORY;
        out.size = native.smb2_size;
        clear_operation();
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }

    const char *err = last_error();
    if (is_path_not_found_status_hint(status) ||
        is_path_not_found_error(err) ||
        is_path_not_found_status(last_status(status))) {
        clear_operation();
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }

    set_smb_error(error_out, error_out_size, "stat", status);
    clear_operation();
    return StorageSmbOperationResult::Error;
}

StorageSmbOperationResult StorageSmbClient::step_mkdir(
    const char *remote_path,
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *control) {
    if (!connected_ || !ctx_) {
        set_error(error_out, error_out_size, "not_connected");
        return StorageSmbOperationResult::Error;
    }
    if (!remote_path || !remote_path[0]) {
        set_error(error_out, error_out_size, "bad_remote_path");
        return StorageSmbOperationResult::Error;
    }

    if (!operation_) {
        if (!begin_operation(OperationKind::Mkdir,
                             error_out, error_out_size)) {
            return StorageSmbOperationResult::Error;
        }

        const int rc = smb2_mkdir_async(
            ctx_, remote_path, smb_generic_cb, &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Mkdir, status, control,
        error_out, error_out_size);
    if (result != StorageSmbOperationResult::Ready) return result;

    if (status == 0) {
        clear_operation();
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }

    set_smb_error(error_out, error_out_size, "mkdir", status);
    clear_operation();
    Log::logf(CAT_EXPORT,
              LOG_DEBUG,
              "[SMB] mkdir failed path=%s error=%s\n",
              remote_path ? remote_path : "--",
              error_out && error_out[0] ? error_out : "mkdir_failed");
    return StorageSmbOperationResult::Error;
}

bool StorageSmbClient::begin_ensure_directory(const char *remote_path,
                                              char *error_out,
                                              size_t error_out_size) {
    if (ensure_phase_ != EnsureDirectoryPhase::Idle) {
        if (strcmp(ensure_target_, remote_path) == 0) return true;
        set_error(error_out, error_out_size, "operation_busy");
        return false;
    }

    copy_cstr(ensure_target_, sizeof(ensure_target_), remote_path);
    ensure_current_[0] = '\0';
    ensure_mkdir_error_[0] = '\0';
    ensure_cursor_ = 0;
    return advance_ensure_directory(error_out, error_out_size);
}

bool StorageSmbClient::advance_ensure_directory(char *error_out,
                                                size_t error_out_size) {
    while (ensure_target_[ensure_cursor_] == '/') ensure_cursor_++;
    if (ensure_target_[ensure_cursor_] == '\0') {
        reset_ensure_directory();
        return true;
    }

    const size_t segment_start = ensure_cursor_;
    while (ensure_target_[ensure_cursor_] != '/' &&
           ensure_target_[ensure_cursor_] != '\0') {
        ensure_cursor_++;
    }
    const size_t segment_len = ensure_cursor_ - segment_start;
    if (!append_path_segment(ensure_current_, sizeof(ensure_current_),
                             ensure_target_ + segment_start, segment_len)) {
        set_error(error_out, error_out_size, "remote_path_too_long");
        reset_ensure_directory();
        return false;
    }

    ensure_phase_ = EnsureDirectoryPhase::Stat;
    return true;
}

void StorageSmbClient::reset_ensure_directory() {
    ensure_target_[0] = '\0';
    ensure_current_[0] = '\0';
    ensure_mkdir_error_[0] = '\0';
    ensure_cursor_ = 0;
    ensure_phase_ = EnsureDirectoryPhase::Idle;
}

StorageSmbOperationResult StorageSmbClient::step_ensure_directory(
    const char *remote_path,
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *control) {
    if (!connected_ || !ctx_) {
        set_error(error_out, error_out_size, "not_connected");
        return StorageSmbOperationResult::Error;
    }
    if (!remote_path || !remote_path[0]) {
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }
    if (!begin_ensure_directory(remote_path, error_out, error_out_size)) {
        return StorageSmbOperationResult::Error;
    }
    if (ensure_phase_ == EnsureDirectoryPhase::Idle) {
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }

    StorageSmbRemoteStat remote;
    StorageSmbOperationResult result = StorageSmbOperationResult::Waiting;
    switch (ensure_phase_) {
        case EnsureDirectoryPhase::Stat:
            result = step_stat(ensure_current_, remote,
                               error_out, error_out_size, control);
            if (result != StorageSmbOperationResult::Ready) return result;
            if (remote.exists && !remote.directory) {
                set_error(error_out, error_out_size,
                          "remote_not_directory");
                reset_ensure_directory();
                return StorageSmbOperationResult::Error;
            }
            if (!remote.exists) {
                ensure_phase_ = EnsureDirectoryPhase::Mkdir;
                return StorageSmbOperationResult::Waiting;
            }
            if (!advance_ensure_directory(error_out, error_out_size)) {
                return StorageSmbOperationResult::Error;
            }
            break;

        case EnsureDirectoryPhase::Mkdir:
            result = step_mkdir(ensure_current_,
                                error_out, error_out_size, control);
            if (result == StorageSmbOperationResult::Waiting) return result;
            if (result == StorageSmbOperationResult::Error) {
                if (!connected_) {
                    reset_ensure_directory();
                    return result;
                }
                copy_cstr(ensure_mkdir_error_,
                          sizeof(ensure_mkdir_error_),
                          error_out);
                ensure_phase_ = EnsureDirectoryPhase::Verify;
                return StorageSmbOperationResult::Waiting;
            }
            if (!advance_ensure_directory(error_out, error_out_size)) {
                return StorageSmbOperationResult::Error;
            }
            break;

        case EnsureDirectoryPhase::Verify:
            result = step_stat(ensure_current_, remote,
                               error_out, error_out_size, control);
            if (result == StorageSmbOperationResult::Waiting) return result;
            if (result == StorageSmbOperationResult::Ready &&
                remote.exists && remote.directory) {
                if (!advance_ensure_directory(error_out, error_out_size)) {
                    return StorageSmbOperationResult::Error;
                }
                break;
            }
            set_error(error_out, error_out_size,
                      ensure_mkdir_error_[0]
                          ? ensure_mkdir_error_ : "mkdir_failed");
            reset_ensure_directory();
            return StorageSmbOperationResult::Error;

        case EnsureDirectoryPhase::Idle:
            break;
    }

    if (ensure_phase_ == EnsureDirectoryPhase::Idle) {
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }
    return StorageSmbOperationResult::Waiting;
}

StorageSmbOperationResult StorageSmbClient::step_open_writer(
    const char *remote_path,
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *control) {
    if (!connected_ || !ctx_) {
        set_error(error_out, error_out_size, "not_connected");
        return StorageSmbOperationResult::Error;
    }
    if (writer_) {
        set_error(error_out, error_out_size, "writer_busy");
        return StorageSmbOperationResult::Error;
    }
    if (!remote_path || !remote_path[0]) {
        set_error(error_out, error_out_size, "bad_remote_path");
        return StorageSmbOperationResult::Error;
    }

    if (!operation_) {
        if (!begin_operation(OperationKind::Open,
                             error_out, error_out_size)) {
            return StorageSmbOperationResult::Error;
        }

        const int rc = smb2_open_async(
            ctx_, remote_path, O_WRONLY | O_CREAT | O_TRUNC,
            smb_generic_cb, &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Open, status, control,
        error_out, error_out_size);
    if (result != StorageSmbOperationResult::Ready) return result;

    struct smb2fh *opened = static_cast<struct smb2fh *>(
        operation_->callback.result);
    if (status < 0 || !opened) {
        set_smb_error(error_out, error_out_size, "open", status);
        clear_operation();
        return StorageSmbOperationResult::Error;
    }

    writer_ = opened;
    clear_operation();
    set_error(error_out, error_out_size, "");
    return StorageSmbOperationResult::Ready;
}

StorageSmbOperationResult StorageSmbClient::step_write(
    const uint8_t *data,
    size_t len,
    size_t &written_out,
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *control) {
    written_out = 0;
    if (!connected_ || !ctx_ || !writer_) {
        set_error(error_out, error_out_size, "writer_not_open");
        return StorageSmbOperationResult::Error;
    }
    if (!data || len == 0 || len > UINT32_MAX) {
        set_error(error_out, error_out_size, "bad_write");
        return StorageSmbOperationResult::Error;
    }

    if (write_size_ == 0) {
        write_data_ = data;
        write_size_ = len;
        write_offset_ = 0;
    } else if (write_data_ != data || write_size_ != len) {
        set_error(error_out, error_out_size, "operation_busy");
        return StorageSmbOperationResult::Error;
    }

    if (!operation_) {
        if (!begin_operation(OperationKind::Write,
                             error_out, error_out_size)) {
            return StorageSmbOperationResult::Error;
        }

        uint32_t max_write = smb2_get_max_write_size(ctx_);
        if (max_write == 0) max_write = UINT32_MAX;
        size_t chunk = write_size_ - write_offset_;
        if (chunk > max_write) chunk = max_write;
        if (chunk > UINT32_MAX) chunk = UINT32_MAX;
        write_chunk_size_ = chunk;

        const int rc = smb2_write_async(
            ctx_, writer_, write_data_ + write_offset_,
            static_cast<uint32_t>(chunk),
            smb_generic_cb, &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Write, status, control,
        error_out, error_out_size);
    if (result == StorageSmbOperationResult::Waiting) return result;
    if (result == StorageSmbOperationResult::Error) {
        write_data_ = nullptr;
        write_size_ = 0;
        write_offset_ = 0;
        write_chunk_size_ = 0;
        return result;
    }

    clear_operation();
    if (status <= 0 || static_cast<size_t>(status) > write_chunk_size_) {
        if (status < 0) {
            set_smb_error(error_out, error_out_size, "write", status);
        } else {
            set_error(error_out, error_out_size, "write_short");
        }
        write_data_ = nullptr;
        write_size_ = 0;
        write_offset_ = 0;
        write_chunk_size_ = 0;
        return StorageSmbOperationResult::Error;
    }

    write_offset_ += static_cast<size_t>(status);
    write_chunk_size_ = 0;
    if (write_offset_ < write_size_) {
        return StorageSmbOperationResult::Waiting;
    }

    written_out = write_offset_;
    write_data_ = nullptr;
    write_size_ = 0;
    write_offset_ = 0;
    set_error(error_out, error_out_size, "");
    return StorageSmbOperationResult::Ready;
}

StorageSmbOperationResult StorageSmbClient::step_close_writer(
    char *error_out,
    size_t error_out_size,
    const BackgroundOperationControl *control) {
    if (!writer_ && !closing_writer_) {
        set_error(error_out, error_out_size, "");
        return StorageSmbOperationResult::Ready;
    }
    if (!ctx_) {
        set_error(error_out, error_out_size, "not_connected");
        return StorageSmbOperationResult::Error;
    }

    if (!operation_) {
        if (!begin_operation(OperationKind::Close,
                             error_out, error_out_size)) {
            return StorageSmbOperationResult::Error;
        }

        closing_writer_ = writer_;
        writer_ = nullptr;
        const int rc = smb2_close_async(
            ctx_, closing_writer_, smb_generic_cb,
            &operation_->callback);
        if (rc < 0) {
            operation_->callback.status = rc;
            operation_->callback.finished = true;
        }
    }

    int status = 0;
    const StorageSmbOperationResult result = poll_operation(
        OperationKind::Close, status, control,
        error_out, error_out_size);
    if (result != StorageSmbOperationResult::Ready) return result;

    closing_writer_ = nullptr;
    clear_operation();
    if (status < 0) {
        set_smb_error(error_out, error_out_size, "close", status);
        return StorageSmbOperationResult::Error;
    }

    set_error(error_out, error_out_size, "");
    return StorageSmbOperationResult::Ready;
}

void StorageSmbClient::abort_connection() {
    reset_host_resolution();
    if (!ctx_) {
        clear_operation();
        reset_ensure_directory();
        connected_ = false;
        writer_ = nullptr;
        closing_writer_ = nullptr;
        write_data_ = nullptr;
        write_size_ = 0;
        write_offset_ = 0;
        write_chunk_size_ = 0;
        configured_socket_ = -1;
        return;
    }
    if (writer_) {
        struct smb2fh *fh = writer_;
        writer_ = nullptr;
        if (!smb_queue_close_for_destroy(ctx_, fh)) {
            Log::logf(CAT_EXPORT, LOG_WARN,
                      "[SMB] failed to queue writer close before abort\n");
        }
    }
    writer_ = nullptr;
    connected_ = false;
    smb2_destroy_context(ctx_);
    ctx_ = nullptr;
    closing_writer_ = nullptr;
    clear_operation();
    reset_ensure_directory();
    write_data_ = nullptr;
    write_size_ = 0;
    write_offset_ = 0;
    write_chunk_size_ = 0;
    configured_socket_ = -1;
}

}  // namespace aircannect
