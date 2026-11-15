#include "wireguard_config.h"

#if AB_WIREGUARD_ENABLED

#include <stdio.h>
#include <string.h>

#include "string_util.h"

namespace aircannect {

namespace {

bool base64_char(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') || c == '+' || c == '/';
}

// WireGuard keys are 32 bytes in base64: 43 symbols plus one '='.
bool valid_wireguard_key(const String &key) {
    if (key.length() != AC_WIREGUARD_KEY_TEXT_LEN) return false;
    for (size_t i = 0; i + 1 < AC_WIREGUARD_KEY_TEXT_LEN; ++i) {
        if (!base64_char(key[i])) return false;
    }
    return key[AC_WIREGUARD_KEY_TEXT_LEN - 1] == '=';
}

bool parse_ipv4(const char *text, size_t length, uint8_t out[4]) {
    if (!text || length == 0 || length > 15) return false;
    size_t part = 0;
    uint32_t value = 0;
    size_t digits = 0;
    for (size_t i = 0; i <= length; ++i) {
        const char c = i < length ? text[i] : '.';
        if (c >= '0' && c <= '9') {
            value = value * 10 + static_cast<uint32_t>(c - '0');
            if (++digits > 3 || value > 255) return false;
            continue;
        }
        if (c != '.' || digits == 0 || part >= 4) return false;
        out[part++] = static_cast<uint8_t>(value);
        value = 0;
        digits = 0;
    }
    return part == 4;
}

bool parse_prefix(const char *text, size_t length, uint8_t &prefix) {
    if (!text || length == 0 || length > 2) return false;
    uint32_t value = 0;
    for (size_t i = 0; i < length; ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        value = value * 10 + static_cast<uint32_t>(text[i] - '0');
    }
    if (value > 32) return false;
    prefix = static_cast<uint8_t>(value);
    return true;
}

void format_ipv4(const uint8_t ip[4], char *out, size_t out_size) {
    snprintf(out, out_size, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

bool printable_without_space(const String &value) {
    for (size_t i = 0; i < value.length(); ++i) {
        const char c = value[i];
        if (c <= ' ' || c > '~') return false;
    }
    return true;
}

// Collapses "a, b ,c" into "a,b,c" and drops empty entries.
String normalize_list(const String &value) {
    String out;
    int start = 0;
    while (start <= static_cast<int>(value.length())) {
        int comma = value.indexOf(',', start);
        if (comma < 0) comma = value.length();
        String item = value.substring(start, comma);
        item.trim();
        if (item.length()) {
            if (out.length()) out += ',';
            out += item;
        }
        start = comma + 1;
    }
    return out;
}

}  // namespace

const char *wireguard_mode_name(WireguardMode mode) {
    switch (mode) {
        case WireguardMode::Off: return "off";
        case WireguardMode::Away: return "away";
        case WireguardMode::Always: return "always";
    }
    return "off";
}

bool parse_wireguard_mode(const String &value, WireguardMode &mode) {
    String text = value;
    text.trim();
    text.toLowerCase();
    if (text == "off") {
        mode = WireguardMode::Off;
    } else if (text == "away") {
        mode = WireguardMode::Away;
    } else if (text == "always") {
        mode = WireguardMode::Always;
    } else {
        return false;
    }
    return true;
}

void normalize_wireguard_config(WireguardConfig &config) {
    config.home_ssids = normalize_list(config.home_ssids);
    config.address.trim();
    config.private_key.trim();
    config.peer_public_key.trim();
    config.preshared_key.trim();
    config.endpoint.trim();
    config.allowed_ips = normalize_list(config.allowed_ips);
    config.allowed_ips.replace(" ", "");
    config.dns.trim();
}

bool parse_wireguard_endpoint(const String &text, WireguardEndpoint &out) {
    if (!text.length() || text.length() > AC_WIREGUARD_HOST_MAX + 6 ||
        !printable_without_space(text)) {
        return false;
    }
    // IPv6 literals are not supported by the tunnel; reject brackets so the
    // last colon is always the port separator.
    if (text.indexOf('[') >= 0 || text.indexOf(']') >= 0) return false;

    const int colon = text.lastIndexOf(':');
    if (colon >= 0 && text.indexOf(':') != colon) return false;

    out.host = colon >= 0 ? text.substring(0, colon) : text;
    out.port = AC_WIREGUARD_DEFAULT_PORT;
    if (!out.host.length() || out.host.length() > AC_WIREGUARD_HOST_MAX) {
        return false;
    }
    if (colon >= 0) {
        uint16_t port = 0;
        if (!parse_port(text.substring(colon + 1), port)) return false;
        out.port = port;
    }
    return true;
}

bool parse_wireguard_address(const String &text, char *ip, size_t ip_size) {
    const int slash = text.indexOf('/');
    const String host = slash >= 0 ? text.substring(0, slash) : text;
    if (slash >= 0 && text.substring(slash + 1) != "32") return false;

    uint8_t bytes[4] = {};
    if (!parse_ipv4(host.c_str(), host.length(), bytes)) return false;
    if (ip && ip_size) format_ipv4(bytes, ip, ip_size);
    return true;
}

bool next_wireguard_cidr(const String &list,
                         size_t &cursor,
                         WireguardCidr &out,
                         bool &malformed) {
    malformed = false;
    if (cursor >= list.length()) return false;

    int comma = list.indexOf(',', cursor);
    if (comma < 0) comma = list.length();
    const String item = list.substring(cursor, comma);
    cursor = static_cast<size_t>(comma) + 1;

    const int slash = item.indexOf('/');
    uint8_t ip[4] = {};
    uint8_t prefix = 32;
    const size_t ip_length =
        slash >= 0 ? static_cast<size_t>(slash) : item.length();
    if (!parse_ipv4(item.c_str(), ip_length, ip) ||
        (slash >= 0 &&
         !parse_prefix(item.c_str() + slash + 1,
                       item.length() - static_cast<size_t>(slash) - 1,
                       prefix))) {
        malformed = true;
        return false;
    }

    out.address = (static_cast<uint32_t>(ip[0]) << 24) |
                  (static_cast<uint32_t>(ip[1]) << 16) |
                  (static_cast<uint32_t>(ip[2]) << 8) | ip[3];
    out.netmask = prefix == 0 ? 0 : (UINT32_MAX << (32 - prefix));
    return true;
}

const char *wireguard_config_reject_reason(const WireguardConfig &config) {
    if (config.home_ssids.length() > AC_WIREGUARD_HOME_SSIDS_MAX) {
        return "home_ssids_too_long";
    }
    if (config.address.length() &&
        !parse_wireguard_address(config.address, nullptr, 0)) {
        return "bad_address";
    }
    if (config.private_key.length() &&
        !valid_wireguard_key(config.private_key)) {
        return "bad_private_key";
    }
    if (config.peer_public_key.length() &&
        !valid_wireguard_key(config.peer_public_key)) {
        return "bad_peer_public_key";
    }
    if (config.preshared_key.length() &&
        !valid_wireguard_key(config.preshared_key)) {
        return "bad_preshared_key";
    }
    WireguardEndpoint endpoint;
    if (config.endpoint.length() &&
        !parse_wireguard_endpoint(config.endpoint, endpoint)) {
        return "bad_endpoint";
    }
    if (config.allowed_ips.length() > AC_WIREGUARD_ALLOWED_IPS_MAX) {
        return "allowed_ips_too_long";
    }
    size_t cursor = 0;
    WireguardCidr cidr;
    bool malformed = false;
    size_t route_count = 0;
    while (next_wireguard_cidr(config.allowed_ips, cursor, cidr, malformed)) {
        ++route_count;
    }
    if (malformed) return "bad_allowed_ips";
    if (config.mode != WireguardMode::Off &&
        route_count > AC_WIREGUARD_MAX_ALLOWED_IPS) {
        return "too_many_allowed_ips";
    }
    if (config.dns.length() &&
        !parse_wireguard_address(config.dns, nullptr, 0)) {
        return "bad_dns";
    }

    if (config.mode != WireguardMode::Off &&
        !wireguard_config_complete(config)) {
        return "incomplete";
    }
    return nullptr;
}

bool wireguard_config_complete(const WireguardConfig &config) {
    return config.address.length() && config.private_key.length() &&
           config.peer_public_key.length() && config.endpoint.length() &&
           config.allowed_ips.length();
}

bool wireguard_ssid_is_home(const WireguardConfig &config,
                            const String &ssid) {
    if (!ssid.length()) return false;
    int start = 0;
    const String &list = config.home_ssids;
    while (start < static_cast<int>(list.length())) {
        int comma = list.indexOf(',', start);
        if (comma < 0) comma = list.length();
        if (list.substring(start, comma) == ssid) return true;
        start = comma + 1;
    }
    return false;
}

}  // namespace aircannect

#endif  // AB_WIREGUARD_ENABLED
