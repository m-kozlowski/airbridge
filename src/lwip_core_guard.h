#pragma once

#include <lwip/tcpip.h>

namespace aircannect {

class LwipCoreGuard {
public:
#ifdef CONFIG_LWIP_TCPIP_CORE_LOCKING
    LwipCoreGuard()
        : locked_(!sys_thread_tcpip(LWIP_CORE_LOCK_QUERY_HOLDER)) {
        if (locked_) LOCK_TCPIP_CORE();
    }

    ~LwipCoreGuard() {
        if (locked_) UNLOCK_TCPIP_CORE();
    }
#else
    LwipCoreGuard() = default;
#endif
    LwipCoreGuard(const LwipCoreGuard &) = delete;
    LwipCoreGuard &operator=(const LwipCoreGuard &) = delete;

private:
#ifdef CONFIG_LWIP_TCPIP_CORE_LOCKING
    bool locked_ = false;
#endif
};

}  // namespace aircannect
