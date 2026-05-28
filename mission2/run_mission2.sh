#!/usr/bin/env bash
# run_mission2.sh — WebSocket bridge + HTTP server tek komutla başlatır.
# Görev düğümünü (joystick_control.py) ayrı terminalde çalıştırın.

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# ROS2 ortamı
[ -z "$ROS_DISTRO" ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/Desktop/SANCAK/px4_ws/install/setup.bash" ] && \
    source "$HOME/Desktop/SANCAK/px4_ws/install/setup.bash"

if ! python3 -c "import websockets" 2>/dev/null; then
    echo "[HATA] 'websockets' yok. Kur: pip3 install websockets --break-system-packages"
    exit 1
fi

BRIDGE_PID=""
HTTP_PID=""
cleanup() {
    echo "[run] Kapatılıyor..."
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true
    [ -n "$HTTP_PID" ]   && kill $HTTP_PID   2>/dev/null || true
    wait 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "════════════════════════════════════════════════"
echo "  SANCAK — Görev 2 · Sanal Kumanda"
echo "════════════════════════════════════════════════"

echo "[1/3] WebSocket bridge (port 8765)..."
python3 joystick_bridge.py &
BRIDGE_PID=$!
sleep 1.5

echo "[2/3] HTTP server (port 8080)..."
python3 -m http.server 8080 --bind 127.0.0.1 >/dev/null 2>&1 &
HTTP_PID=$!
sleep 1

URL="http://localhost:8080/joystick_control.html"
echo "[3/3] Tarayıcı: $URL"
if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$URL" >/dev/null 2>&1 &
fi

echo ""
echo "════════════════════════════════════════════════"
echo "  Hazır. Başka terminalde:"
echo "    cd ~/Desktop/SANCAK/mission2"
echo "    python3 joystick_control.py"
echo "  Ctrl+C = bu pencereyi kapat"
echo "════════════════════════════════════════════════"

wait
