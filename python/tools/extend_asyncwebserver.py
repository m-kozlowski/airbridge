#!/usr/bin/env python3
"""Build patched HTTP/TCP sources without changing installed libraries."""

from pathlib import Path

from library_patches import build_patched_source


def build_source(env, node):
    original = Path(node.srcnode().get_abspath())
    library = {
        "AsyncTCP.cpp": "asynctcp",
        "WebResponses.cpp": "espasyncwebserver",
        "AsyncEventSource.cpp": "espasyncwebserver-sse",
    }[original.name]
    return build_patched_source(env, node, library)


if "Import" in globals():
    Import("env")
    env.AddBuildMiddleware(build_source, "*/ESPAsyncWebServer/src/WebResponses.cpp")
    env.AddBuildMiddleware(build_source, "*/ESPAsyncWebServer/src/AsyncEventSource.cpp")
    env.AddBuildMiddleware(build_source, "*/AsyncTCP/src/AsyncTCP.cpp")
    env.AddBuildMiddleware(build_source, "*/AsyncTCP@*/src/AsyncTCP.cpp")
