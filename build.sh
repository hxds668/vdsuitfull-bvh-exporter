#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

ARCH="$(uname -m)"
OUTPUT_DIR="./bin/x64"
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
  OUTPUT_DIR="./bin/arm64"
elif [[ "$ARCH" == "x86_64" ]]; then
  OUTPUT_DIR="./bin/x64"
else
  echo "Unsupported architecture: $ARCH"
  exit 1
fi

EXPORTER_OUTPUT="$OUTPUT_DIR/vdsuit_bvh_exporter"
RECEIVER_OUTPUT="$OUTPUT_DIR/vdsuit_wireless_receiver"
mkdir -p "$OUTPUT_DIR"

echo "Building exporter for $ARCH -> $EXPORTER_OUTPUT"
g++ -std=c++11 -O2 -Wall -Wextra -I./include \
  -I./src -rdynamic \
  ./src/main.cpp ./src/bvh_exporter.cpp ./src/network_streamer.cpp \
  ./src/wireless_hotspot.cpp ./src/wireless_sdk_bridge.cpp \
  ./src/sdk_virtual_serial.cpp ./src/vdsuit_wireless_protocol.cpp \
  -ldl -lpthread -lutil -o "$EXPORTER_OUTPUT"

echo "Building wireless receiver emulator -> $RECEIVER_OUTPUT"
g++ -std=c++11 -O2 -Wall -Wextra -I./src \
  ./src/wireless_receiver.cpp ./src/wireless_hotspot.cpp \
  ./src/vdsuit_wireless_protocol.cpp \
  -lpthread -o "$RECEIVER_OUTPUT"

echo "Build OK:"
echo "  $EXPORTER_OUTPUT"
echo "  $RECEIVER_OUTPUT"
