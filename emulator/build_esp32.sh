#!/usr/bin/env bash
# Build the host ESP32-S3 emulator: segment_player's ESP32 code path compiled
# natively against emulator/esp32/stubs and linked with emulator/esp32/sim.cpp.
#   emulator/build_esp32.sh            -> build/esp32-host/segment_player
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/build/esp32-host"
CXX="${CXX:-g++}"
mkdir -p "$OUT"
FLAGS=(-std=gnu++17 -O2 -Wall -Wextra -DARDUINO_ARCH_ESP32 -I "$ROOT/emulator/esp32/stubs"
       -I "$ROOT/arduino_firmware/segment_player")
"$CXX" "${FLAGS[@]}" -x c++ "$ROOT/arduino_firmware/segment_player/segment_player.ino" \
    -x none "$ROOT/emulator/esp32/sim.cpp" -o "$OUT/segment_player"
echo "$OUT/segment_player"
