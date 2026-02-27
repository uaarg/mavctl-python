#!/usr/bin/env bash
set -e

source /root/venv-ardupilot/bin/activate 
pip install --upgrade pip
pip install pytest pymavlink pexpect

# Start SITL if not running
if ! pgrep -f ArduCopter > /dev/null; then
    echo "Starting SITL..."
    Tools/autotest/sim_vehicle.py -v ArduCopter \
        --no-mavproxy \
        --no-rebuild \
        --speedup 1 \
        --udp \
        --out=udp:127.0.0.1:14550 \
        --instance 0 &
    SITL_PID=$!
else
    echo "SITL already running"
fi

mavproxy.py \
    --master=udp:127.0.0.1:5760 \
    --out=udp:127.0.0.1:14551 \
    --daemon &
MAVPROXY_PID=$!

# Ensure SITL is ready
echo "Waiting for SITL process..."
until pgrep -f ArduCopter > /dev/null; do
    sleep 1
done
echo "SITL process detected"

echo "SITL ready"

sleep 30

cd /workspace
# Run tests
echo "Running python script"

#echo "Running pytest..."
PYTHONPATH=. pytest

# Kill SITL if started here
if [ ! -z "$SITL_PID" ]; then
    echo "Stopping SITL..."
    kill $SITL_PID
    wait $SITL_PID || true
fi

# Kill Mavproxy if started here
if [ ! -z "$MAVPROXY_PID" ]; then
    echo "Stopping MAVROXY"
    kill $MAVPROXY_PID
    wait $MAVPROXY_PID || true
fi
