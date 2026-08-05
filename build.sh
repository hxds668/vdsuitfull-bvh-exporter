#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

ARCH="$(uname -m)"
OUTPUT="./bin/x64/vdsuit_bvh_exporter"
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
  OUTPUT="./bin/arm64/vdsuit_bvh_exporter"
elif [[ "$ARCH" == "x86_64" ]]; then
  OUTPUT="./bin/x64/vdsuit_bvh_exporter"
else
  echo "Unsupported architecture: $ARCH"
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT")"

echo "Building for $ARCH -> $OUTPUT"
g++ -std=c++11 -O2 -Wall -Wextra -I./include \
  ./src/main.cpp ./src/bvh_exporter.cpp ./src/network_streamer.cpp \
  -ldl -lpthread -o "$OUTPUT"

echo "Build OK: $OUTPUT"
echo "Run: $OUTPUT"
