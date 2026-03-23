#!/bin/bash
VER=$(git describe --tags --always --dirty 2>/dev/null || echo "unknown")
DATE=$(date +%Y-%m-%dT%H:%M)
echo "-DAIRBRIDGE_VERSION=\\\"$VER\\\" -DAIRBRIDGE_BUILD_DATE=\\\"$DATE\\\""
