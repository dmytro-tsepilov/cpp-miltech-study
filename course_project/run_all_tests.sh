#!/bin/bash

cd "$(dirname "$0")/robot_ws"
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null || echo "Warning: install not sourced"

SCENARIOS=(
    "training_ground.yaml:training_ground"
    "patrol_alpha.yaml:patrol_alpha"
    "large_patrol.yaml:large_patrol"
)

echo "============================================"
echo "=== Perimeter Miner Test Suite ==="
echo "============================================"
echo ""

# Run unit tests first
echo ">>> Running unit tests..."
colcon test --packages-select perimeter_miner mine_simulator http_reporter 2>&1 || echo "Unit tests had errors (expected if dependencies not built)"
colcon test-result --all 2>/dev/null || echo "No test results to display"
echo ""

for entry in "${SCENARIOS[@]}"; do
    SCENARIO=${entry%%:*}
    NAME=${entry##*:}
    
    echo "============================================"
    echo "=== Running scenario: ${SCENARIO} ==="
    echo "============================================"
    
    # Create bag directory (absolute path)
    BAG_DIR="$(pwd)/../../bags/${NAME}"
    rm -rf "${BAG_DIR}"
    mkdir -p "${BAG_DIR}"
    
    # Start rosbag recording in background with correct syntax
    (
        source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
        ros2 bag record -a -o "${BAG_DIR}" > /tmp/rosbag_output_${NAME}.log 2>&1
    ) &
    BAG_PID=$!
    
    # Give bag recorder time to start
    sleep 3
    
    echo "=== Bag PID: ${BAG_PID} ==="
    
    # Run launch and capture output (with timeout for closed-loop perimeters)
    timeout 60 ros2 launch perimeter_miner system.launch.py \
        scenario_file:=${SCENARIO} \
        simulate_mines:=true \
        enable_fake_odom:=true \
        enable_reporter:=false \
        2>&1 | tee /tmp/launch_output_${NAME}.txt || true
    
    # Stop bag recorder gracefully
    echo "=== Stopping bag recorder ==="
    kill ${BAG_PID} 2>/dev/null || true
    sleep 2
    kill -9 ${BAG_PID} 2>/dev/null || true
    wait ${BAG_PID} 2>/dev/null || true
    
    # Verify bag
    echo "============================================"
    echo "=== Bag info for ${NAME} ==="
    if [ -d "${BAG_DIR}" ]; then
        echo "Bag files:"
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
        
        echo "=== Bag recorder log ==="
        cat /tmp/rosbag_output_${NAME}.log 2>/dev/null || echo "No bag recorder log"
    else
        echo "Bag directory does not exist"
    fi
    echo "============================================"
    
    # Check mission result
    echo "=== Mission results from log ==="
    MISSION_RESULT=$(grep -E "(SUCCESS|FAILED|Mission summary)" /tmp/launch_output_${NAME}.txt 2>/dev/null || echo "")
    if [ -n "$MISSION_RESULT" ]; then
        echo "$MISSION_RESULT"
    else
        echo "No mission result found (expected SUCCESS or FAILED in logs)"
    fi
done

echo ""
echo "============================================"
echo "=== All scenarios completed ==="
echo "=== Bags saved to: ../../bags/ ==="
echo "============================================"
