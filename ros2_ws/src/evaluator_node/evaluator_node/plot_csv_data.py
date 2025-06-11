import os
import pandas as pd
import matplotlib.pyplot as plt

def get_plot_folder(csv_path):
    # csv_path is assumed to be like: .../Data Set/<controller>/<csv_filename>.csv
    csv_dir = os.path.dirname(csv_path)                  # .../Data Set/<controller>
    controller = os.path.basename(csv_dir)               # controller name
    # Get the parent folder of "Data Set"
    parent = os.path.dirname(csv_dir)                    # .../Data Set
    # Build Data Plots folder in the same workspace root as "Data Set"
    plot_folder = os.path.join(parent.replace("Data Set", "Data Plots"), controller)
    os.makedirs(plot_folder, exist_ok=True)
    return plot_folder, controller

def plot_csv(csv_path):
    # Load CSV with comma delimiter; expected header as below:
    # time, PWM1, PWM2, PWM Efforts, Joint Angle Current 1, Joint Angle Current 2, Joint Angle Desired 1, Joint Angle Desired 2, Controller Name
    df = pd.read_csv(csv_path)
    # Base name from the CSV file: e.g. default_controller_DataSet[20250611_220334]
    base_filename = os.path.splitext(os.path.basename(csv_path))[0]
    # Determine the folder to save plots
    plot_folder, controller = get_plot_folder(csv_path)
    base = os.path.join(plot_folder, base_filename)

    # Plot 1: PWM1 & PWM2 vs Time
    plt.figure()
    plt.plot(df['time'], df['PWM1'], label="PWM1")
    plt.plot(df['time'], df['PWM2'], label="PWM2")
    plt.xlabel("Time (s)")
    plt.ylabel("PWM")
    plt.title(f"PWM vs Time - {controller}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(base + "_PWM.jpeg", format="jpeg")
    plt.close()

    # Plot 2: Effort vs Time (Units: V^2)
    plt.figure()
    plt.plot(df['time'], df['PWM Efforts'], label="Effort")
    plt.xlabel("Time (s)")
    plt.ylabel("Effort (V^2)")
    plt.title(f"Effort vs Time - {controller}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(base + "_Effort.jpeg", format="jpeg")
    plt.close()

    # Plot 3: q1 Actual & q1 Desired vs Time (angles in degrees)
    plt.figure()
    plt.plot(df['time'], df['Joint Angle Current 1'], label="q1 Actual")
    plt.plot(df['time'], df['Joint Angle Desired 1'], label="q1 Desired")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title(f"Angle Trajectory q1 - {controller}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(base + "_AngleTrajectory_q1.jpeg", format="jpeg")
    plt.close()

    # Plot 4: q2 Actual & q2 Desired vs Time
    plt.figure()
    plt.plot(df['time'], df['Joint Angle Current 2'], label="q2 Actual")
    plt.plot(df['time'], df['Joint Angle Desired 2'], label="q2 Desired")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title(f"Angle Trajectory q2 - {controller}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(base + "_AngleTrajectory_q2.jpeg", format="jpeg")
    plt.close()

    print(f"Plots saved for {csv_path} in folder: {os.path.abspath(plot_folder)}")

def main():
    # Always search for CSV files in the Data Set folder in the source tree
    data_set_dir = "/workspaces/ros2_ws/src/evaluator_node/Data Set"
    csv_files = []
    for root, dirs, files in os.walk(data_set_dir):
        for f in files:
            if f.lower().endswith(".csv"):
                csv_files.append(os.path.join(root, f))
    if not csv_files:
        print(f"No CSV files found in {data_set_dir}.")
        return
    for file in csv_files:
        print(f"Processing {file}...")
        try:
            plot_csv(file)
        except Exception as e:
            print(f"Failed to process {file}: {e}")

if __name__ == "__main__":
    main()
