#!/bin/sh

set -eu

ARDUINO_LINT="${ARDUINO_LINT:-arduino-lint}"
ROOT_DIR="$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)"

"$ARDUINO_LINT" \
  --project-type library \
  --library-manager submit \
  --compliance specification \
  --format text \
  "$ROOT_DIR"
