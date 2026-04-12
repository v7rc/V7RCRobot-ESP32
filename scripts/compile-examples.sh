#!/bin/sh

set -eu

if [ -n "${ARDUINO_CLI:-}" ]; then
  ARDUINO_CLI_BIN="$ARDUINO_CLI"
elif command -v arduino-cli >/dev/null 2>&1; then
  ARDUINO_CLI_BIN="arduino-cli"
elif [ -x /tmp/arduino-cli-bin/arduino-cli ]; then
  ARDUINO_CLI_BIN="/tmp/arduino-cli-bin/arduino-cli"
else
  echo "arduino-cli not found. Set ARDUINO_CLI or install arduino-cli." >&2
  exit 127
fi

FQBN="${FQBN:-esp32:esp32:esp32c3}"
ROOT_DIR="$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)"
BUILD_ROOT="${BUILD_ROOT:-/tmp/arduino-build}"

compile_example() {
  sketch_dir="$1"
  build_dir="$2"

  mkdir -p "$build_dir"
  "$ARDUINO_CLI_BIN" compile \
    --build-path "$build_dir" \
    --fqbn "$FQBN" \
    --library "$ROOT_DIR" \
    "$ROOT_DIR/$sketch_dir"
}

compile_example "examples/ESP32_C3_Differential_Drive" "$BUILD_ROOT/diff"
compile_example "examples/ESP32_C3_Car_Runtime_Demo" "$BUILD_ROOT/car-runtime"
compile_example "examples/ESP32_C3_Differential_Drive_Clean" "$BUILD_ROOT/diff-clean"
compile_example "examples/ESP32_C3_Differential_Runtime_Demo" "$BUILD_ROOT/diff-runtime"
compile_example "examples/ESP32_C3_Dog_Runtime_Demo" "$BUILD_ROOT/dog-runtime"
compile_example "examples/ESP32_C3_Dog_Runtime_Demo_V2" "$BUILD_ROOT/dog-runtime-v2"
compile_example "examples/ESP32_C3_Drone_V7RC_BLE_Control" "$BUILD_ROOT/drone-v7rc-ble"
compile_example "examples/ESP32_C3_Drone_Runtime_Demo" "$BUILD_ROOT/drone-runtime"
compile_example "examples/ESP32_C3_Mecanum_Runtime_Demo" "$BUILD_ROOT/mec-runtime"
compile_example "examples/ESP32_C3_Mecanum_V7RC_BLE_Control" "$BUILD_ROOT/mec-v7rc-ble"
compile_example "examples/ESP32_C3_Mini_Mecanum_Clean" "$BUILD_ROOT/mec-clean"
compile_example "examples/ESP32_C3_Mini_Mecanum_V3" "$BUILD_ROOT/mec"
compile_example "examples/ESP32_C3_OTTO_Runtime_Demo" "$BUILD_ROOT/otto-runtime"
compile_example "examples/ESP32_C3_RobotARM_Runtime_Demo" "$BUILD_ROOT/robotarm-runtime"
compile_example "examples/ESP32_C3_Tank_V7RC_BLE_Control" "$BUILD_ROOT/tank-v7rc-ble"
compile_example "examples/ESP32_C3_Tank_Runtime_Demo" "$BUILD_ROOT/tank-runtime"
