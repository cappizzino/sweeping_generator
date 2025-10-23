#!/bin/bash

export UAV_NAMES="[drone_0, drone_1, drone_2]"

# Absolute path to launch files
ROS_LAUNCH_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/../launch" && pwd)"
UAV_NAMES_ARRAY=($(echo "$UAV_NAMES" | tr -d '[]' | tr ',' ' '))

echo "Launching sweeping generator for UAVs: ${UAV_NAMES_ARRAY[*]}"
echo "Using launch path: $ROS_LAUNCH_PATH"
echo

# Check if the launch file exists
if [[ ! -f "$ROS_LAUNCH_PATH/swarm_followers_unity.launch" ]]; then
    echo "Error: Launch file not found at $ROS_LAUNCH_PATH/swarm_followers_unity.launch"
    exit 1
fi

# Launch one instance per UAV (in separate terminals or background processes)
for UAV in "${UAV_NAMES_ARRAY[@]}"; do
    echo "Launching for $UAV..."
    
    # Example: passing UAV name as an argument to the launch file
    # Adjust argument name as per your .launch file
    gnome-terminal -- bash -c "roslaunch --wait $ROS_LAUNCH_PATH/swarm_followers_unity.launch UAV_NAME:=$UAV"
    
    sleep 2  # short delay between launches to avoid race conditions
done

echo "All UAVs launched."
echo "Press ENTER to terminate all UAV-related ROS nodes..."
read  # Wait for user input

# Get list of running nodes
RUNNING_NODES=$(rosnode list 2>/dev/null)

if [[ -z "$RUNNING_NODES" ]]; then
    echo "No ROS nodes are currently running."
    exit 0
fi

# Terminate all UAV-related ROS nodes
for NODE in $RUNNING_NODES; do
    if [[ $NODE == *"swarm_followers_unity_"* ]]; then
        echo "Terminating $NODE..."
        rosnode kill $NODE
    fi
done

echo "All UAV-related ROS nodes terminated."