#pragma once

#include "board.h"

#if AB_WIREGUARD_ENABLED

#include <stddef.h>

#include "wireguard_config.h"

struct netif;

namespace aircannect {

struct WireguardRoutes {
    WireguardCidr prefixes[AC_WIREGUARD_MAX_ALLOWED_IPS] = {};
    size_t count = 0;

    bool has_default_route() const;
};

// The service holds the lwIP core lock. Published routes stay immutable and
// alive until withdrawn with a null interface, before removing that interface.
void publish_wireguard_routes(netif *interface, const WireguardRoutes *routes);

}  // namespace aircannect

#endif  // AB_WIREGUARD_ENABLED
