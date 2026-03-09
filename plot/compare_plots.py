import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# 自动处理中文显示问题
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS'] 
plt.rcParams['axes.unicode_minus'] = False 

def load_csv(filenames):
    for f in filenames:
        if os.path.exists(f):
            try:
                print(f"Reading {f}...")
                df = pd.read_csv(f)
                # Normalize time
                if 'Time(ms)' in df.columns:
                    t0 = df['Time(ms)'].iloc[0]
                    df['Time_Sec'] = (df['Time(ms)'] - t0) / 1000.0
                return df
            except Exception as e:
                print(f"Error reading {f}: {e}")
    return None

# Load SDK Data
sdk_files = [
    'build/bin/basic_demo_log.csv',
    'basic_demo_log.csv', 
    r'd:\桌面\黑漫灵巧手\WA100-SDK-Publish-version1.1\wa100-sdk-publish\build\bin\basic_demo_log.csv'
]
df_sdk = load_csv(sdk_files)

# Load Raw Data
raw_files = [
    'raw_protocol_log.csv',
    r'd:\桌面\黑漫灵巧手\WA100-SDK-Publish-version1.1\wa100-sdk-publish\raw_protocol_log.csv'
]
df_raw = load_csv(raw_files)

if df_sdk is None or df_raw is None:
    print("Error: Could not find one of the log files (basic_demo_log.csv or raw_protocol_log.csv)")
    # Fallback to plotting whatever is available? 
    # The user specifically asked for comparison.
    if df_sdk is None: print("Missing: basic_demo_log.csv")
    if df_raw is None: print("Missing: raw_protocol_log.csv")
    sys.exit(1)

plt.figure(figsize=(14, 10))

# === Global View ===
plt.subplot(2, 1, 1)
# Plot SDK Desired
plt.plot(df_sdk['Time_Sec'], df_sdk['CmdPos'], label='Desired (Target)', linestyle='--', color='black', alpha=0.5, linewidth=2)
# Re-plot Raw Desired (optional, to check alignment) -> Decided to skip to confuse less, assuming similar trajectory. 
# Or better: plot raw target as thinner line if needed.
# Let's trust they are similar P2P moves 1000->2000.

# Plot Real SDK
plt.plot(df_sdk['Time_Sec'], df_sdk['RealPos'], label='Real (SDK C++)', color='blue', linewidth=1.5, alpha=0.8)

# Plot Real Raw
# To align visually, they both start at t=0. 
plt.plot(df_raw['Time_Sec'], df_raw['RealPos'], label='Real (Raw Python)', color='red', linewidth=1.5, alpha=0.8, linestyle='-')

plt.title('整体对比: SDK vs Raw Protocol (Global View)')
plt.ylabel('Position (0-4095)')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)

# === Zoomed View ===
# Zoom into a typical transition area, e.g., 2.0s to 4.0s (Where step/slope likely happens)
# Or adjust based on data. Step usually happens every 1s or 2s.
zoom_start = 2.0
zoom_end = 4.0

plt.subplot(2, 1, 2)

# Filter Data
mask_sdk = (df_sdk['Time_Sec'] >= zoom_start) & (df_sdk['Time_Sec'] <= zoom_end)
mask_raw = (df_raw['Time_Sec'] >= zoom_start) & (df_raw['Time_Sec'] <= zoom_end)

df_sdk_z = df_sdk[mask_sdk]
df_raw_z = df_raw[mask_raw]

# Plot Desired (Using SDK's as ref)
plt.plot(df_sdk_z['Time_Sec'], df_sdk_z['CmdPos'], label='Desired', linestyle='--', color='black', alpha=0.5, linewidth=2)

# Plot Real SDK - with dots to show discrete updates
plt.plot(df_sdk_z['Time_Sec'], df_sdk_z['RealPos'], label='Real (SDK)', color='blue', marker='o', markersize=3, linewidth=1, alpha=0.7)

# Plot Real Raw
plt.plot(df_raw_z['Time_Sec'], df_raw_z['RealPos'], label='Real (Raw)', color='red', marker='x', markersize=3, linewidth=1, alpha=0.7)

plt.title(f'细节放大 ({zoom_start}s - {zoom_end}s) - 检查阶梯与平滑 (Zoomed Detail)')
plt.ylabel('Position')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True, which='both', linestyle='-', alpha=0.5)

plt.tight_layout()
print("Comparison plot generated.")
plt.show()
