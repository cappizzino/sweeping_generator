#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import psutil
import rospy
from std_msgs.msg import Empty
import argparse
import json

# Create an argument parser
parser = argparse.ArgumentParser(description="Script that uses command-line arguments.")

# Define an argument named --arg1
parser.add_argument("--arg1", type=int, required=True, help="Argument passed from another script.")

# Parse the arguments from command line
args = parser.parse_args()

N = args.arg1
print(f"Argument received: {N}")

# ============================================================
# Path to your launch script
LAUNCH_SCRIPT = "./launch_swarm.sh"

# How long to let each swarm run before measuring (seconds)
WARMUP_TIME = 5 #20

# Measurement duration (seconds)
MEASUREMENT_DURATION = 10

# ============================================================
# Helper Functions
# pip install XlsxWriter
# ============================================================
def stop_callback(msg):
    global stop_measurement
    rospy.loginfo("Stop signal received via /stop_measurement topic.")
    stop_measurement = True


def measure_system_usage(N_drones, interval=1):
    """Measure CPU and memory usage asynchronously until a ROS Empty message arrives."""
    global stop_measurement
    stop_measurement = False

    n_drones = str(N_drones)
    name = "system_usage_monitor_" + n_drones
    rospy.init_node(name, disable_signals=True, anonymous=True)

    # Subscribe to the stop topic
    rospy.Subscriber("/stop_measurement", Empty, stop_callback)

    cpu_samples = []
    mem_samples = []

    rospy.loginfo("Started system usage monitoring... Waiting for /stop_measurement message to finish.")

    # Keep measuring until stop signal
    while not stop_measurement and not rospy.is_shutdown():
        cpu_sum, mem_sum = 0.0, 0.0
        for proc in psutil.process_iter(['name', 'cmdline', 'cpu_percent', 'memory_info']):
            try:
                if any(key in ' '.join(proc.info['cmdline']) for key in ['roslaunch', 'rosmaster', 'rosout', 'python3']):
                    cpu_sum += proc.info['cpu_percent']
                    mem_sum += proc.info['memory_info'].rss / (1024 * 1024)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        cpu_samples.append(cpu_sum)
        mem_samples.append(mem_sum)
        time.sleep(interval)

    rospy.loginfo("Measurement stopped. Returning samples.")
    return cpu_samples, mem_samples


def kill_ros_processes():
    """Kill all ROS-related processes cleanly."""
    os.system("pkill -f roslaunch")
    os.system("pkill -f rosmaster")
    os.system("pkill -f rosout")
    os.system("pkill -f gzserver")
    os.system("pkill -f gzclient")
    time.sleep(5)

# ============================================================
# Main Experiment Logic
# ============================================================

# Launch swarm
proc = subprocess.Popen([LAUNCH_SCRIPT, str(N)], preexec_fn=os.setsid)
proc.wait()
print(f"Swarm launched with {N} drones. Waiting {WARMUP_TIME}s for stabilization...")
time.sleep(WARMUP_TIME)

# Measure system usage
cpu_samples, mem_samples = measure_system_usage(N)

# Stop measurement by publishing to /stop_measurement
kill_ros_processes()
time.sleep(5)

# Return structured data as JSON
result = {
    "N": N,
    "CPU": cpu_samples,
    "MEM": mem_samples
}
print(json.dumps(result))
