"""Build the patched esp_wireguard platform source without editing its install."""

from library_patches import build_patched_source


def build_source(env, node):
    return build_patched_source(env, node, "wireguard-platform")


if "Import" in globals():
    Import("env")
    flags = env.ParseFlags(env.GetProjectOption("build_flags", []))
    definitions = dict(
        item for item in flags.get("CPPDEFINES", []) if isinstance(item, (tuple, list))
    )
    if str(definitions.get("AB_WIREGUARD_ENABLED")) == "1":
        env.AppendUnique(LINKFLAGS=["-Wl,--wrap=ip4_route_src_hook"])
        env.AddBuildMiddleware(build_source, "*/esp_wireguard/src/wireguard-platform.c")
