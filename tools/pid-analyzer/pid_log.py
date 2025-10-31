import pandas as pd
import matplotlib.pyplot as plt
log_file_path = 'log.csv' 

column_names = ['Error', 'Integral', 'Derivative', 'MV']

try:
    df = pd.read_csv(log_file_path, header=None, names=column_names)
except FileNotFoundError:
    print(f"Log file '{log_file_path}' not found.")
    exit()
time_axis = df.index
fig, axs = plt.subplots(4, 1, figsize=(15, 12), sharex=True)

fig.suptitle('PID Controller Log Analysis', fontsize=16)

axs[0].plot(time_axis, df['Error'], label='Error', color='blue')
axs[0].axhline(0, color='r', linestyle='--', linewidth=1) 
axs[0].set_ylabel('Error')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(time_axis, df['Integral'], label='Integral', color='green')
axs[1].set_ylabel('Integral')
axs[1].legend()
axs[1].grid(True)

axs[2].plot(time_axis, df['Derivative'], label='Derivative', color='orange')
axs[2].set_ylabel('Derivative')
axs[2].legend()
axs[2].grid(True)

axs[3].plot(time_axis, df['MV'], label='MV (Output)', color='purple')
axs[3].set_xlabel('Time (Data Points)')
axs[3].set_ylabel('MV (Output)')
axs[3].legend()
axs[3].grid(True)


plt.tight_layout(rect=[0, 0.03, 1, 0.95]) 
plt.show()
