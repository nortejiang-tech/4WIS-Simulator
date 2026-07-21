#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Reproduce the Windows / low-end-GPU rendering path on a Mac.
#
# The 4WIS portable package is a localhost web app: the launcher just opens the
# built frontend in the machine's DEFAULT browser. So "Windows compatibility" is
# really "how does the WebGL scene render on that machine's browser + GPU". Macs
# get ANGLE→Metal (24-bit depth, fast); many Windows machines — VMs, RDP
# sessions, blocklisted drivers, integrated GPUs — fall back to SwiftShader
# (software WebGL) or a lower-precision D3D path. That is what this script
# reproduces locally, by forcing Google Chrome onto a chosen WebGL backend.
#
# Usage:
#   scripts/render_verify/launch.sh software   # SwiftShader — the weak-Windows path (default)
#   scripts/render_verify/launch.sh gl         # ANGLE desktop-GL backend
#   scripts/render_verify/launch.sh default    # hardware (Metal) — your normal baseline
#   scripts/render_verify/launch.sh probe      # just the depth/FPS probe page, software backend
#
# Env:
#   PORT=8017         backend/app port to serve on
#   BACKEND_PY=...    python to run uvicorn (default: backend/.venv/bin/python)
# ---------------------------------------------------------------------------
set -euo pipefail

MODE="${1:-software}"
PORT="${PORT:-8017}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
BACKEND_PY="${BACKEND_PY:-$ROOT/backend/.venv/bin/python}"
PROFILE="$(mktemp -d /tmp/4wis-render-verify.XXXXXX)"

[ -x "$CHROME" ] || { echo "Google Chrome not found at: $CHROME" >&2; exit 1; }

case "$MODE" in
  software|probe) FLAGS=(--use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader) ;;
  gl)             FLAGS=(--use-gl=angle --use-angle=gl) ;;
  default)        FLAGS=() ;;
  *) echo "unknown mode '$MODE' (use: software | gl | default | probe)" >&2; exit 1 ;;
esac

# --- start the backend (serves the built prod frontend at /) if not already up ---
STARTED_BACKEND=""
if curl -sf "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
  echo "[render-verify] reusing backend already on :$PORT"
else
  [ -x "$BACKEND_PY" ] || { echo "backend python not found: $BACKEND_PY" >&2; exit 1; }
  [ -d "$ROOT/frontend/dist" ] || { echo "frontend/dist missing — run 'npm run build' in frontend/ first" >&2; exit 1; }
  echo "[render-verify] starting backend on :$PORT (serving frontend/dist) …"
  ( cd "$ROOT/backend" && PYTHONPATH="src" "$BACKEND_PY" -m uvicorn sim4wis.main:app \
      --host 127.0.0.1 --port "$PORT" --log-level warning ) &
  STARTED_BACKEND=$!
  for _ in $(seq 1 30); do
    curl -sf "http://127.0.0.1:$PORT/health" >/dev/null 2>&1 && break
    sleep 0.5
  done
fi

cleanup() {
  [ -n "$STARTED_BACKEND" ] && kill "$STARTED_BACKEND" 2>/dev/null || true
  rm -rf "$PROFILE"
}
trap cleanup EXIT

if [ "$MODE" = "probe" ]; then
  URL="file://$ROOT/scripts/render_verify/webgl_probe.html"
else
  URL="http://127.0.0.1:$PORT/"
fi

echo "[render-verify] backend: ANGLE mode = ${MODE}"
echo "[render-verify] opening: $URL"
echo "[render-verify] verify the backend at chrome://gpu (look for 'SwiftShader' under WebGL)."
echo "[render-verify] close the Chrome window to stop; the temp backend + profile are cleaned up."

"$CHROME" \
  --user-data-dir="$PROFILE" \
  --no-first-run --no-default-browser-check --new-window \
  --window-size=1440,900 \
  "${FLAGS[@]}" \
  "$URL"
