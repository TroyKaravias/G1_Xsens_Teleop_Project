#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
RECORDING="${1:-$ROOT_DIR/data/xsens_20260723_114126.xudp}"
PUBLISHER_LOG="${TMPDIR:-/tmp}/g1_xsens_publisher_preflight.log"

cd "$ROOT_DIR"

if pgrep -f "tools/live_xsens_sonic.py" >/dev/null; then
  echo "A live Xsens publisher is already running. Stop it before preflight."
  exit 2
fi

publisher_pid=""
cleanup() {
  if [[ -n "$publisher_pid" ]]; then
    kill "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

env PYTHONPATH=. "$PYTHON_BIN" tools/live_xsens_sonic.py \
  --bind 127.0.0.1 \
  --xsens-port 9763 \
  --zmq-bind tcp://127.0.0.1:5556 \
  --calibration-seconds 9 \
  --batch-frames 1 \
  >"$PUBLISHER_LOG" 2>&1 &
publisher_pid="$!"

sleep 1

env PYTHONPATH=. "$PYTHON_BIN" tools/check_sonic_zmq_stream.py \
  --endpoint tcp://127.0.0.1:5556 \
  --duration 28 \
  --min-messages 100 &
checker_pid="$!"

sleep 1
env PYTHONPATH=. "$PYTHON_BIN" -m xsens_bridge replay \
  "$RECORDING" \
  --host 127.0.0.1 \
  --port 9763 \
  --speed 1.0

wait "$checker_pid"

echo
echo "Publisher summary:"
tail -n 12 "$PUBLISHER_LOG"
echo
echo "PASS: direct Jetson replay preflight completed without robot output"
