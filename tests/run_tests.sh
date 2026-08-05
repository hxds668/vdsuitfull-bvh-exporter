#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TEST_BINARY="/tmp/vdsuit_network_streamer_test"

cd "$PROJECT_DIR"
g++ -std=c++11 -O2 -Wall -Wextra -I./include -I./src \
  ./tests/network_streamer_test.cpp ./src/bvh_exporter.cpp ./src/network_streamer.cpp \
  -lpthread -o "$TEST_BINARY"
"$TEST_BINARY"

