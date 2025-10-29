#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: $0 <number_of_drones>"
    exit 1
fi

N=$1

# Generate UAV_NAMES automatically
UAV_LIST=()
for ((i=0; i<N; i++)); do
    UAV_LIST+=("drone_$i")
done

# Join UAV_LIST into the desired export format: [drone_0, drone_1, ...]
UAV_NAMES="[$(IFS=', '; echo "${UAV_LIST[*]}")]"
export UAV_NAMES

# Absolute path to launch files
ROS_LAUNCH_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/../launch" && pwd)"
UAV_NAMES_ARRAY=($(echo "$UAV_NAMES" | tr -d '[]' | tr ',' ' '))

# Roscore
gnome-terminal -- bash -c "roscore"

# launch ROS-TCP
gnome-terminal -- bash -c "roslaunch --wait $ROS_LAUNCH_PATH/endpoint.launch"

# Publish number of UAVs
gnome-terminal -- bash -c "rostopic pub /uav/drone_count std_msgs/Int32 \"$N\" --once"

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
    
    sleep 4  # short delay between launches to avoid race conditions
done

# WayPoint Navigation
gnome-terminal -- bash -c "roslaunch --wait $ROS_LAUNCH_PATH/swarm_waypoint_unity.launch"