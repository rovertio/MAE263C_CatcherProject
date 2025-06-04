import os
import csv
import matplotlib.pyplot as plt
import pandas as pd

# Update this to where your CSVs are stored
LOG_DIR = os.path.expanduser("~/ros2_eval_logs")

def load_data():
    data = {}
    for controller in os.listdir(LOG_DIR):
        ctrl_path = os.path.join(LOG_DIR, controller)
        if not os.path.isdir(ctrl_path):
            continue
        trials = []
        for file in os.listdir(ctrl_path):
            if file.endswith(".csv"):
                filepath = os.path.join(ctrl_path, file)
                df = pd.read_csv(filepath)
                trials.append((file, df))
        data[controller] = trials
    return data

def plot_all_trials(data):
    for controller, trials in data.items():
        # ─── Error vs Time ────────────────────────────────
        plt.figure()
        for name, df in trials:
            plt.plot(df["time_sec"], df["error"], label=name)
        plt.title(f"Error over Time – {controller}")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{controller}_error_plot.png")
        plt.close()

        # ─── Energy vs Time ──────────────────────────────
        plt.figure()
        for name, df in trials:
            plt.plot(df["time_sec"], df["energy"], label=name)
        plt.title(f"Control Energy (τ²∫) – {controller}")
        plt.xlabel("Time (s)")
        plt.ylabel("Cumulative τ² dt")
        plt.legend()
        plt.grid(True)
        plt.savefig(f"{controller}_energy_plot.png")
        plt.close()


if __name__ == "__main__":
    data = load_data()
    plot_all_trials(data)
