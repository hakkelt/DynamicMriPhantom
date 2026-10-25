#!/usr/bin/env bash
# Build arduino_firmware/segment_player for AVR with plain avr-gcc + ArduinoCore-avr.
#
#   emulator/build_avr.sh [atmega328p|atmega328pb]     (default atmega328p)
#
# Output: build/avr-<mcu>/segment_player.{elf,hex}
# The atmega328p build is what the avr8js emulator runs; the 328PB build checks
# that the sketch also compiles for the Xplained Mini (same Timer1/PORTD/PCINT2);
# it needs the Arduino AVR toolchain (Ubuntu's avr-libc 2.0.0 lacks complete
# ATmega328PB headers).
#
# Needs avr-gcc/avr-g++/avr-objcopy/avr-size on PATH (or AVR_GCC_BIN=<dir>).
# ArduinoCore-avr is cloned into .deps/ (pinned tag) unless ARDUINO_CORE_AVR is set.
set -euo pipefail

MCU="${1:-atmega328p}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CORE_TAG="1.8.6"
CORE="${ARDUINO_CORE_AVR:-$ROOT/.deps/ArduinoCore-avr}"
OUT="$ROOT/build/avr-$MCU"
SKETCH="$ROOT/arduino_firmware/segment_player"

if [[ -n "${AVR_GCC_BIN:-}" ]]; then export PATH="$AVR_GCC_BIN:$PATH"; fi
command -v avr-g++ >/dev/null || { echo "avr-g++ not found (set AVR_GCC_BIN)" >&2; exit 1; }

if [[ ! -d "$CORE/cores/arduino" ]]; then
    git clone --quiet --depth 1 --branch "$CORE_TAG" https://github.com/arduino/ArduinoCore-avr "$CORE"
fi

rm -rf "$OUT"
mkdir -p "$OUT/core"
FLAGS=(-mmcu="$MCU" -DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR
       -Os -g -ffunction-sections -fdata-sections -flto -Wall -Wextra
       -I"$CORE/cores/arduino" -I"$CORE/variants/standard")
CFLAGS=("${FLAGS[@]}" -std=gnu11 -fno-fat-lto-objects)
CXXFLAGS=("${FLAGS[@]}" -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics -fno-fat-lto-objects)

objs=()
for src in "$CORE"/cores/arduino/*.c "$CORE"/cores/arduino/*.cpp "$CORE"/cores/arduino/*.S; do
    obj="$OUT/core/$(basename "$src").o"
    case "$src" in
        *.c)   avr-gcc "${CFLAGS[@]}" -w -c "$src" -o "$obj" ;;
        *.cpp) avr-g++ "${CXXFLAGS[@]}" -w -c "$src" -o "$obj" ;;
        *.S)   avr-gcc -x assembler-with-cpp "${FLAGS[@]}" -c "$src" -o "$obj" ;;
    esac
    objs+=("$obj")
done
avr-gcc-ar rcs "$OUT/core.a" "${objs[@]}"

# The sketch defines every function before use, so it compiles as plain C++.
cp "$SKETCH/segment_player.ino" "$OUT/segment_player.cpp"
avr-g++ "${CXXFLAGS[@]}" -Werror -I"$SKETCH" -c "$OUT/segment_player.cpp" -o "$OUT/segment_player.o"
avr-gcc -mmcu="$MCU" -Os -flto -fuse-linker-plugin -Wl,--gc-sections \
    "$OUT/segment_player.o" "$OUT/core.a" -lm -o "$OUT/segment_player.elf"
avr-objcopy -O ihex -R .eeprom "$OUT/segment_player.elf" "$OUT/segment_player.hex"
avr-size -A "$OUT/segment_player.elf" | awk '$1==".text"||$1==".data"||$1==".bss"{print}'
echo "built $OUT/segment_player.hex"
