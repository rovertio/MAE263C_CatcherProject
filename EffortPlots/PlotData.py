import matplotlib.pyplot as plt
import pandas as pd
# Load the data using the correct delimiter
file_path = r"C:\Users\elipp\OneDrive\Documents\GitHub\MAE263C_CatcherProject\EffortPlots\log.csv"
df = pd.read_csv(file_path)

# Plot 1: PWM1 & PWM2 vs Time

plt.figure()
plt.plot(df['time'], df['PWM1'], label='PWM1')
plt.plot(df['time'], df['PWM2'], label='PWM2')
plt.xlabel('Time (s)')
plt.ylabel('PWM Value')
plt.title('PWM1 and PWM2 vs Time')
plt.legend()
plt.grid(True)
# Example: Save in the same folder as the script
plt.savefig(r"C:\Users\elipp\OneDrive\Documents\GitHub\MAE263C_CatcherProject\EffortPlots\pwm_vs_time.jpeg")

# Plot 2: Effort vs Time
plt.figure()
plt.plot(df['time'], df['PWM Efforts'], label='Effort (V^2)')
plt.xlabel('Time (s)')
plt.ylabel('Effort (V^2)')
plt.title('Effort vs Time')
plt.grid(True)
# Example: Save in the same folder as the script
plt.savefig(r"C:\Users\elipp\OneDrive\Documents\GitHub\MAE263C_CatcherProject\EffortPlots\effort_vs_time.jpeg")

# Plot 3: q1 actual & q1 desired vs Time
plt.figure()
plt.plot(df['time'], df['Joint Angle Current 1'], label='q1 Actual')
plt.plot(df['time'], df['Joint Angle Desired 1'], label='q1 Desired')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Joint 1 Actual and Desired vs Time')
plt.legend()
plt.grid(True)
# Example: Save in the same folder as the script
plt.savefig(r"C:\Users\elipp\OneDrive\Documents\GitHub\MAE263C_CatcherProject\EffortPlots\q1.jpeg")

# Plot 4: q2 actual & q2 desired vs Time
plt.figure()
plt.plot(df['time'], df['Joint Angle Current 2'], label='q2 Actual')
plt.plot(df['time'], df['Joint Angle Desired 2'], label='q2 Desired')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Joint 2 Actual and Desired vs Time')
plt.legend()
plt.grid(True)
plt.savefig(r"C:\Users\elipp\OneDrive\Documents\GitHub\MAE263C_CatcherProject\EffortPlots\q2.jpeg")
