#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TEST_BINARY="/tmp/vdsuit_network_streamer_test"
PROTOCOL_TEST_BINARY="/tmp/vdsuit_wireless_protocol_test"
SESSION_TEST_BINARY="/tmp/vdsuit_wireless_session_test"
SDK_BRIDGE_TEST_BINARY="/tmp/vdsuit_wireless_sdk_bridge_test"
SDK_INTEGRATION_TEST_BINARY="/tmp/vdsuit_wireless_sdk_integration_test"

cd "$PROJECT_DIR"
g++ -std=c++11 -O2 -Wall -Wextra -I./include -I./src \
  ./tests/network_streamer_test.cpp ./src/bvh_exporter.cpp ./src/network_streamer.cpp \
  -lpthread -o "$TEST_BINARY"
"$TEST_BINARY"

g++ -std=c++11 -O2 -Wall -Wextra -I./src \
  ./tests/vdsuit_wireless_protocol_test.cpp \
  ./src/vdsuit_wireless_protocol.cpp ./src/wireless_hotspot.cpp \
  -lpthread -o "$PROTOCOL_TEST_BINARY"
"$PROTOCOL_TEST_BINARY"

g++ -std=c++11 -O2 -Wall -Wextra -Wno-unused-function -I./src \
  ./tests/wireless_session_test.cpp \
  ./src/vdsuit_wireless_protocol.cpp ./src/wireless_hotspot.cpp \
  -lpthread -o "$SESSION_TEST_BINARY"
"$SESSION_TEST_BINARY"

g++ -std=c++11 -O2 -Wall -Wextra -I./src \
  ./tests/wireless_sdk_bridge_test.cpp \
  ./src/wireless_sdk_bridge.cpp ./src/vdsuit_wireless_protocol.cpp \
  -lpthread -lutil -o "$SDK_BRIDGE_TEST_BINARY"
"$SDK_BRIDGE_TEST_BINARY"

SDK_LIBRARY=""
case "$(uname -m)" in
  aarch64|arm64)
    SDK_LIBRARY="./lib/arm64/libVDMocapSDK_miniArm64.so"
    ;;
  x86_64)
    SDK_LIBRARY="./lib/x64/libVDMocapSDK_mini.so"
    ;;
esac

if [[ -n "$SDK_LIBRARY" && -f "$SDK_LIBRARY" ]]; then
  SDK_LIBRARY="$(realpath "$SDK_LIBRARY")"
  g++ -std=c++11 -O2 -Wall -Wextra -I./include -I./src -rdynamic \
    ./tests/wireless_sdk_integration_test.cpp \
    ./src/wireless_sdk_bridge.cpp ./src/sdk_virtual_serial.cpp \
    ./src/vdsuit_wireless_protocol.cpp \
    -ldl -lpthread -lutil -o "$SDK_INTEGRATION_TEST_BINARY"

  SDK_TEST_WORKDIR="$(mktemp -d /tmp/vdsuit-sdk-test.XXXXXX)"
  cleanup_sdk_test_workdir() {
    case "$SDK_TEST_WORKDIR" in
      /tmp/vdsuit-sdk-test.*) rm -rf -- "$SDK_TEST_WORKDIR" ;;
    esac
  }
  trap cleanup_sdk_test_workdir EXIT
  mkdir -p "$SDK_TEST_WORKDIR/bin/arm64"
  if [[ -d ./bin/arm64/CalibrationFiles ]]; then
    cp -a ./bin/arm64/CalibrationFiles "$SDK_TEST_WORKDIR/bin/arm64/"
  fi
  (
    cd "$SDK_TEST_WORKDIR"
    timeout 30 "$SDK_INTEGRATION_TEST_BINARY" "$SDK_LIBRARY"
  )
  cleanup_sdk_test_workdir
  trap - EXIT
fi
