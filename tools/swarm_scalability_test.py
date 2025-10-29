#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import psutil
import pandas as pd
import matplotlib.pyplot as plt

# Swarm sizes to test
# SWARM_SIZES = [1, 5, 10, 20, 50, 100, 150, 200]
# SWARM_SIZES = [3, 6, 9, 15, 20, 25, 30, 40, 50, 75, 100]
# SWARM_SIZES = [3, 5, 7, 10, 25, 50, 100, 200]
SWARM_SIZES = [3]

# Path to your launch script
LAUNCH_SCRIPT = "./launch_swarm.sh"

# How long to let each swarm run before measuring (seconds)
WARMUP_TIME = 20

# Measurement duration (seconds)
MEASUREMENT_DURATION = 10

# Results CSV
RESULTS_FILE = "scalability_results.csv"

# ============================================================
# Helper Functions
# ============================================================

def measure_system_usage(duration=10, interval=1):
    """Measure average CPU (%) and Memory (MB) usage across all ROS-related processes."""
    cpu_samples = []
    mem_samples = []

    for _ in range(duration):
        # Filter only ROS processes
        cpu_sum, mem_sum = 0.0, 0.0
        for proc in psutil.process_iter(['name', 'cmdline', 'cpu_percent', 'memory_info']):
            try:
                if any(key in ' '.join(proc.info['cmdline']) for key in ['roslaunch', 'rosmaster', 'rosout', 'python3']):
                    cpu_sum += proc.info['cpu_percent']
                    mem_sum += proc.info['memory_info'].rss / (1024 * 1024)  # MB
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        cpu_samples.append(cpu_sum)
        mem_samples.append(mem_sum)
        time.sleep(interval)

    avg_cpu = sum(cpu_samples) / len(cpu_samples)
    avg_mem = sum(mem_samples) / len(mem_samples)
    return avg_cpu, avg_mem


def kill_ros_processes():
    """Kill all ROS-related processes cleanly."""
    os.system("pkill -f roslaunch")
    os.system("pkill -f rosmaster")
    os.system("pkill -f rosout")
    os.system("pkill -f gzserver")
    os.system("pkill -f gzclient")
    time.sleep(5)


# ============================================================
# Main Experiment Loop
# ============================================================

results = []

for N in SWARM_SIZES:
    print(f"\n==========================================")
    print(f"Running scalability test for {N} drones")
    print(f"==========================================")

    # Launch swarm
    proc = subprocess.Popen([LAUNCH_SCRIPT, str(N)], preexec_fn=os.setsid)
    proc.wait()
    print(f"Swarm launched with {N} drones. Waiting {WARMUP_TIME}s for stabilization...")
    time.sleep(WARMUP_TIME)

    print(f"Measuring system usage for {MEASUREMENT_DURATION}s...")
    cpu, mem = measure_system_usage(duration=MEASUREMENT_DURATION)

    print(f"Results for {N} drones: CPU={cpu:.2f}% | MEM={mem:.2f} MB")
    results.append({"N": N, "CPU_Usage(%)": cpu, "Memory_Usage(MB)": mem})

    print("Terminating swarm...")
    kill_ros_processes()
    # os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    time.sleep(5)

# ============================================================
# Save Results
# ============================================================

df = pd.DataFrame(results)
df.to_csv(RESULTS_FILE, index=False)
print(f"\nScalability test completed. Results saved to {RESULTS_FILE}")

# ============================================================
# Plot Results
# ============================================================

plt.figure(figsize=(8, 5))
plt.plot(df["N"], df["CPU_Usage(%)"], 'o-', label="CPU Usage (%)", color='tab:red')
plt.plot(df["N"], df["Memory_Usage(MB)"], 's-', label="Memory Usage (MB)", color='tab:blue')
plt.xlabel("Number of Drones (N)")
plt.ylabel("Resource Usage")
plt.title("Scalability of ROS UAV Swarm Formation")
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.tight_layout()
plt.savefig("scalability_plot.png", dpi=300)
plt.show()
