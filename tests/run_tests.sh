#!/usr/bin/env bash
# Host-side unit tests for the hardware-independent modules (no Pico needed).
#   tests/run_tests.sh
# Requires gcc and python3 with pymavlink (pip install pymavlink).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/lego_dbw_tests"
mkdir -p "$OUT"

gcc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined \
    -I"$ROOT/src" \
    "$ROOT/tests/test_logic.c" \
    "$ROOT/src/lwp3.c" "$ROOT/src/steering.c" "$ROOT/src/drive.c" "$ROOT/src/mavlink_lite.c" "$ROOT/src/supervisor.c" \
    -lm -o "$OUT/test_logic"

"$OUT/test_logic" "$OUT/mavlink_frames.bin"
python3 "$ROOT/tests/check_mavlink.py" "$OUT/mavlink_frames.bin"
