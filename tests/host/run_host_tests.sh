#!/bin/sh
# Builds and runs the host (PC) tests for the sketch and the Modbus library.
# Needs g++ with AddressSanitizer / UBSan (Linux or WSL). Not part of the
# Arduino build: the IDE only compiles the busductThermoSensor/ folder.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../busductThermoSensor"
OUT=$(mktemp -d)
CXX=${CXX:-g++}
FLAGS="-std=gnu++11 -w -fsanitize=address,undefined -I$HERE/stubs -I$SRC"

echo "== syntax check (Arduino stubs) =="
$CXX -std=gnu++11 -Wall -Wextra -fsyntax-only -I"$HERE/stubs" -I"$SRC" "$SRC/SimpleModbusSlave.cpp"
$CXX -std=gnu++11 -Wall -Wextra -fsyntax-only -I"$HERE/stubs" -I"$SRC" -include Arduino.h -x c++ "$SRC/busductThermoSensor.ino"

echo "== Modbus library =="
$CXX $FLAGS "$SRC/SimpleModbusSlave.cpp" "$HERE/test_modbus.cpp" -o "$OUT/test_modbus"
"$OUT/test_modbus"

echo "== Sensor sketch (sampling ISR, temperature, faults, diagnostics, EEPROM) =="
$CXX $FLAGS "$HERE/test_sensor.cpp" "$SRC/SimpleModbusSlave.cpp" -o "$OUT/test_sensor"
"$OUT/test_sensor"

rm -rf "$OUT"
echo "== all host tests passed =="
