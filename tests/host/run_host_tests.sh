#!/bin/sh
# Builds and runs the host (PC) tests for the sketch and the Modbus library.
# Needs g++ (Linux, WSL or Raspberry Pi OS). Not part of the Arduino build:
# the IDE only compiles the busductThermoSensor/ folder.
#
# Sanitizers: by default the script uses AddressSanitizer + UBSan if they work
# on this machine, else UBSan only, else none. AddressSanitizer does not run on
# some kernels (e.g. Raspberry Pi OS with a 39-bit address space).
# Override with SANITIZE=full|ub|none.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../busductThermoSensor"
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
CXX=${CXX:-g++}

# Compiles and runs an empty program with the given flags: succeeds only if the
# sanitizer runtime actually starts on this machine.
sanitizer_works()
{
  echo 'int main(){return 0;}' > "$OUT/probe.cpp"
  $CXX $1 "$OUT/probe.cpp" -o "$OUT/probe" >/dev/null 2>&1 && "$OUT/probe" >/dev/null 2>&1
}

case "${SANITIZE:-auto}" in
  full) SAN="-fsanitize=address,undefined" ;;
  ub)   SAN="-fsanitize=undefined" ;;
  none) SAN="" ;;
  auto)
    if sanitizer_works "-fsanitize=address,undefined"; then
      SAN="-fsanitize=address,undefined"
    elif sanitizer_works "-fsanitize=undefined"; then
      SAN="-fsanitize=undefined"
    else
      SAN=""
    fi ;;
  *) echo "SANITIZE must be full, ub or none" >&2; exit 2 ;;
esac
echo "== sanitizers: ${SAN:-none} =="
FLAGS="-std=gnu++11 -w $SAN -fno-sanitize-recover=all -I$HERE/stubs -I$SRC"
[ -z "$SAN" ] && FLAGS="-std=gnu++11 -w -I$HERE/stubs -I$SRC"

echo "== syntax check (Arduino stubs) =="
$CXX -std=gnu++11 -Wall -Wextra -fsyntax-only -I"$HERE/stubs" -I"$SRC" "$SRC/SimpleModbusSlave.cpp"
$CXX -std=gnu++11 -Wall -Wextra -fsyntax-only -I"$HERE/stubs" -I"$SRC" -include Arduino.h -x c++ "$SRC/busductThermoSensor.ino"

echo "== Modbus library =="
$CXX $FLAGS "$SRC/SimpleModbusSlave.cpp" "$HERE/test_modbus.cpp" -o "$OUT/test_modbus"
"$OUT/test_modbus"

echo "== Sensor sketch (sampling ISR, temperature, faults, diagnostics, EEPROM, LED) =="
$CXX $FLAGS "$HERE/test_sensor.cpp" "$SRC/SimpleModbusSlave.cpp" -o "$OUT/test_sensor"
"$OUT/test_sensor"

echo "== all host tests passed =="
