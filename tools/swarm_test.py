import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import json

# Swarm sizes to test
SWARM_SIZES = [1, 3, 6, 10, 20, 30, 50]

# Results Excel file (one sheet per swarm size)
RESULTS_FILE = "scalability_results.xlsx"

writer = pd.ExcelWriter(RESULTS_FILE, engine='xlsxwriter')
summary = []

# ============================================================
# Main Experiment Loop
# ============================================================
for N in SWARM_SIZES:
    print(f"\n==========================================")
    print(f"Running scalability test for {N} drones")
    print(f"==========================================")
    cpu_samples = []
    mem_samples = []

    result = subprocess.run(
        ["python3", "swarm_scalability.py", "--arg1", str(N)],
        capture_output=True,
        text=True
    )

    # Parse the JSON output from the subprocess
    output_lines = result.stdout.strip().split('\n')
    json_output = output_lines[-1]  # Assuming the last line is the JSON result
    data = json.loads(json_output)
    cpu_samples = data.get("CPU", [])
    mem_samples = data.get("MEM", [])
    avg_cpu = sum(cpu_samples) / len(cpu_samples)
    avg_mem = sum(mem_samples) / len(mem_samples)
    print(f"Results for {N} drones: CPU={avg_cpu:.2f}% | MEM={avg_mem:.2f} MB")

    # Save detailed samples to a sheet
    df_detail = pd.DataFrame({
        "Time (s)": list(range(1, len(cpu_samples)+1)),
        "CPU_Usage(%)": cpu_samples,
        "Memory_Usage(MB)": mem_samples
    })
    df_detail.to_excel(writer, sheet_name=f"N={N}", index=False)

    # Add summary for main chart
    summary.append({"N": N, "Avg_CPU(%)": avg_cpu, "Avg_Memory(MB)": avg_mem})

# ============================================================
# Save Summary Sheet
# ============================================================

df_summary = pd.DataFrame(summary)
df_summary.to_excel(writer, sheet_name="Summary", index=False)
writer.close()
print(f"\nScalability test completed. Results saved to {RESULTS_FILE}")

# ============================================================
# Plot Results
# ============================================================

plt.figure(figsize=(8, 5))
plt.plot(df_summary["N"], df_summary["Avg_CPU(%)"], 'o-', label="CPU Usage (%)", color='tab:red')
plt.plot(df_summary["N"], df_summary["Avg_Memory(MB)"], 's-', label="Memory Usage (MB)", color='tab:blue')
plt.xlabel("Number of Drones (N)")
plt.ylabel("Resource Usage")
plt.title("Scalability of ROS UAV Swarm Formation")
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()
plt.tight_layout()
plt.savefig("scalability_plot.png", dpi=300)
plt.show()