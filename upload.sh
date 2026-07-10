#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_TELEMETRY_DIR="${REMOTE_TELEMETRY_DIR:-$ROOT_DIR/remote-telemetry}"

PIO_ENV="${PIO_ENV:-teensy40}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
BRIDGE_LISTEN="${BRIDGE_LISTEN:-0.0.0.0:9002}"
UPLOAD_PORT="${UPLOAD_PORT:-}"

if ! command -v platformio >/dev/null 2>&1; then
    echo "Error: platformio is not installed or not in PATH." >&2
    exit 1
fi

if ! command -v cargo >/dev/null 2>&1; then
    echo "Error: cargo is not installed or not in PATH." >&2
    exit 1
fi

if [[ ! -d "$REMOTE_TELEMETRY_DIR" ]]; then
    echo "Error: remote-telemetry directory not found at: $REMOTE_TELEMETRY_DIR" >&2
    exit 1
fi

echo "Uploading firmware (env: $PIO_ENV)..."
upload_cmd=(platformio run -e "$PIO_ENV" -t upload)
if [[ -n "$UPLOAD_PORT" ]]; then
    upload_cmd+=(--upload-port "$UPLOAD_PORT")
fi
"${upload_cmd[@]}"

echo "Waiting for serial device: $SERIAL_PORT"
for _ in {1..80}; do
    if [[ -e "$SERIAL_PORT" ]]; then
        break
    fi
    sleep 0.25
done

if [[ ! -e "$SERIAL_PORT" ]]; then
    echo "Error: serial device did not reappear after upload: $SERIAL_PORT" >&2
    exit 1
fi

echo "Starting telemetry bridge on ws://$BRIDGE_LISTEN"
cd "$REMOTE_TELEMETRY_DIR"
exec cargo run -- --bridge --serial "$SERIAL_PORT" --listen "$BRIDGE_LISTEN"