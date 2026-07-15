# start basalt vio and logger concurrently
# handle cleanup upon exit


cleanup() {
    echo
    echo "[shell] Stopping child processes..."

    if [[ -n "$PID1" ]] && kill -0 "$PID1" 2>/dev/null; then
        echo "[shell] Stopping first Python script PID=$PID1"
        kill -INT "$PID1" 2>/dev/null
    fi

    if [[ -n "$PID2" ]] && kill -0 "$PID2" 2>/dev/null; then
        echo "[shell] Stopping second Python script PID=$PID2"
        kill -TERM "$PID2" 2>/dev/null
    fi

    sleep 1

    if [[ -n "$PID1" ]] && kill -0 "$PID1" 2>/dev/null; then
        echo "[shell] Force killing first Python script PID=$PID1"
        kill -KILL "$PID1" 2>/dev/null
    fi

    if [[ -n "$PID2" ]] && kill -0 "$PID2" 2>/dev/null; then
        echo "[shell] Force killing second Python script PID=$PID2"
        kill -KILL "$PID2" 2>/dev/null
    fi

    echo "[shell] Done"
}

trap cleanup INT TERM EXIT

python3 basalt_vio.py &
PID1=$!

python3 vio_to_mavlink.py &
PID2=$!

echo "[shell] Started:"
echo "  basalt_vio.py PID=$PID1"
echo "  vio_to_mavlink.py PID=$PID2"
echo
echo "Press Ctrl+C once to stop both."

wait