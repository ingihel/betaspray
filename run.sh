#!/usr/bin/env bash
set -euo pipefail

if docker image inspect esp32 &>/dev/null; then
    docker run --rm -it --name esp32_dev -v "$(pwd)":/project esp32
else
    docker build -t esp32 .
fi
