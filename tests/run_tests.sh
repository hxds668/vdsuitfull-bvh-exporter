#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TEST_BINARY="/tmp/vdsuit_network_streamer_test"
MAG_TEST_BINARY="/tmp/vdsuit_mag_calibration_test"
CALIBRATION_BACKUP_TEST_BINARY="/tmp/vdsuit_calibration_backup_test"

cd "$PROJECT_DIR"
g++ -std=c++11 -O2 -Wall -Wextra -I./include -I./src \
  ./tests/network_streamer_test.cpp ./src/bvh_exporter.cpp ./src/network_streamer.cpp \
  -lpthread -o "$TEST_BINARY"
"$TEST_BINARY"

g++ -std=c++11 -O2 -Wall -Wextra -I./include -I./src \
  ./tests/mag_calibration_test.cpp ./src/mag_calibration.cpp \
  -o "$MAG_TEST_BINARY"
"$MAG_TEST_BINARY"

g++ -std=c++11 -O2 -Wall -Wextra -I./src \
  ./tests/calibration_backup_test.cpp ./src/calibration_backup.cpp \
  -o "$CALIBRATION_BACKUP_TEST_BINARY"
"$CALIBRATION_BACKUP_TEST_BINARY"
