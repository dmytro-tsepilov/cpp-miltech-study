#!/bin/bash

SCENARIO=${1:-training_ground.yaml}
SCENARIO_NAME=${SCENARIO%.yaml}

cd "$(dirname "$0")/robot_ws"
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null || echo "Warning: install not sourced"

# Create bag directory (absolute path to avoid issues)
# NOTE: Do NOT pre-create the bag directory - ros2 bag record -o will create it
BAG_BASE="$(pwd)/../../bags"
mkdir -p "${BAG_BASE}"
BAG_DIR="${BAG_BASE}/${SCENARIO_NAME}"

# Remove old bag data if exists (ros2 bag record -o needs a clean or non-existent dir)
rm -rf "${BAG_DIR}"

echo "=== Running scenario: ${SCENARIO} ==="
echo "=== Bag directory: ${BAG_DIR} (will be created by ros2 bag) ==="

# Start rosbag recording in background with correct syntax
(
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
    ros2 bag record -a -o "${BAG_DIR}" > /tmp/rosbag_output.log 2>&1
) &
BAG_PID=$!

# Give bag recorder time to start
sleep 3

echo "=== Bag PID: ${BAG_PID} ==="

# Run launch with mine simulation and fake odometry enabled (with timeout for closed-loop perimeters)
timeout 60 ros2 launch perimeter_miner system.launch.py \
    scenario_file:=${SCENARIO} \
    simulate_mines:=true \
    enable_fake_odom:=true \
    enable_reporter:=false \
    2>&1 | tee /tmp/launch_output_${SCENARIO_NAME}.txt || true

# Stop bag recorder gracefully
echo "=== Stopping bag recorder ==="
kill ${BAG_PID} 2>/dev/null || true
sleep 2
kill -9 ${BAG_PID} 2>/dev/null || true
wait ${BAG_PID} 2>/dev/null || true

echo "=== Scenario ${SCENARIO_NAME} completed ==="
echo "=== Bag location: ${BAG_DIR} ==="

# Show bag recorder output
echo "=== Bag recorder log ==="
cat /tmp/rosbag_output.log 2>/dev/null || echo "No bag recorder log"

# Verify bag has content
if [ -d "${BAG_DIR}" ]; then
    echo "=== Bag files ==="
    ls -lh "${BAG_DIR}/" 2>/dev/null || echo "No files in bag directory"
    
    BAG_INFO=$(ros2 bag info "${BAG_DIR}" 2>&1) || { echo "Bag info failed"; }
    if [ -n "$BAG_INFO" ]; then
        echo "$BAG_INFO"
        
        # Check if bag has messages
        MSG_COUNT=$(echo "$BAG_INFO" | grep -oP 'messages:\s*\K[0-9]+' 2>/dev/null || echo "")
        if [ -z "$MSG_COUNT" ]; then
            MSG_COUNT=$(echo "$BAG_INFO" | grep -oP '[0-9]+.*files' | grep -oP '^[0-9]+' 2>/dev/null || echo "")
        fi
        if [ -n "$MSG_COUNT" ] && [ "$MSG_COUNT" -gt 0 ] 2>/dev/null; then
            echo "SUCCESS: Bag contains $MSG_COUNT messages"
        elif [ -n "$MSG_COUNT" ] && [ "$MSG_COUNT" -eq 0 ] 2>/dev/null; then
            echo "WARNING: Bag is empty (0 messages)"
        else
            echo "Bag info available but message count could not be parsed"
        fi
    fi
else
    echo "Bag directory does not exist"
fi

# Check mission result
echo "=== Mission results from log ==="
MISSION_RESULT=$(grep -E "(SUCCESS|FAILED|Mission summary)" /tmp/launch_output_${SCENARIO_NAME}.txt 2>/dev/null || echo "")
if [ -n "$MISSION_RESULT" ]; then
    echo "$MISSION_RESULT"
else
    echo "~ No mission result found (expected SUCCESS or FAILED in logs)"
fi
