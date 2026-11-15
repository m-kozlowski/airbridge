#include "wireguard_routes.h"

#if AB_WIREGUARD_ENABLED

#include <lwip/ip4_addr.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>

namespace {

netif *route_interface = nullptr;
const aircannect::WireguardRoutes *route_table = nullptr;

}  // namespace

namespace aircannect {

bool WireguardRoutes::has_default_route() const {
    for (size_t i = 0; i < count; ++i) {
        if (prefixes[i].netmask == 0) return true;
    }
    return false;
}

void publish_wireguard_routes(netif *interface, const WireguardRoutes *routes) {
    LWIP_ASSERT_CORE_LOCKED();
    route_interface = interface;
    route_table = routes;
}

}  // namespace aircannect

extern "C" netif *__real_ip4_route_src_hook(const ip4_addr_t *source,
                                          const ip4_addr_t *destination);

extern "C" netif *__wrap_ip4_route_src_hook(const ip4_addr_t *source,
                                          const ip4_addr_t *destination) {
    LWIP_ASSERT_CORE_LOCKED();

    // ip4_route_src calls before checking local subnets. Let the original hook
    // handle bound sources; an unspecified source then falls through ip4_route.
    if (source) return __real_ip4_route_src_hook(source, destination);

    if (route_interface && netif_is_up(route_interface) &&
        netif_is_link_up(route_interface)) {
        const uint32_t address = lwip_ntohl(ip4_addr_get_u32(destination));
        for (size_t i = 0; i < route_table->count; ++i) {
            const auto &prefix = route_table->prefixes[i];
            if ((address & prefix.netmask) == (prefix.address & prefix.netmask)) {
                return route_interface;
            }
        }
    }

    return __real_ip4_route_src_hook(source, destination);
}

#endif  // AB_WIREGUARD_ENABLED
