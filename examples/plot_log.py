import matplotlib.pyplot as plt
import csv
import sys
import os

def plot_log(filename):
    times = []
    desired = []
    cmd = []
    real = []
    amp = []
    
    start_time = None

    print(f"Reading file: {filename}")
    
    with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f)
        header = next(reader, None) # Skip header

        for row in reader:
            if not row or len(row) < 6:
                continue
            
            try:
                # 兼容两种格式：
                # 旧: DATA, TIME, ID3, DESIRED, CMD, REAL, AMP (7 cols)
                # 新: TIME, ID, DESIRED, CMD, REAL, AMP (6 cols)
                
                col_idx = 0
                if row[0].strip() == "DATA":
                    col_idx = 1
                
                t_ms = int(row[col_idx])
                # ID is at col_idx + 1
                d_val = int(row[col_idx + 2])
                c_val = int(row[col_idx + 3])
                r_val = int(row[col_idx + 4])
                a_val = int(row[col_idx + 5])
                
                if start_time is None:
                    start_time = t_ms
                
                # Convert to seconds relative to start
                t_rel = (t_ms - start_time) / 1000.0
                
                times.append(t_rel)
                desired.append(d_val)
                cmd.append(c_val)
                real.append(r_val)
                amp.append(a_val)
            except ValueError:
                continue

    if not times:
        print("No valid data found.")
        return

    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Subplot 1: Positions
    ax1.plot(times, desired, label='Desired (Vision)', color='blue', linestyle='--')
    ax1.plot(times, cmd, label='Compliant Cmd (Admittance)', color='green', linewidth=2)
    ax1.plot(times, real, label='Real Position', color='red', alpha=0.7)
    ax1.set_ylabel('Position (0-2000)')
    ax1.set_title(f'Admittance Control Response ({filename})')
    ax1.legend()
    ax1.grid(True)

    # Subplot 2: Current/Force
    ax2.plot(times, amp, label='Current (mA)', color='orange')
    ax2.axhline(y=300, color='r', linestyle=':', label='Threshold (300)') # Assuming 300 is threshold
    ax2.set_ylabel('Current (mA)')
    ax2.set_xlabel('Time (s)')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        # Default fallback or instructions
        log_file = "admittance_log.csv"
        
    if not os.path.exists(log_file):
        print(f"File not found: {log_file}")
        print("Usage: python plot_log.py <logfile.csv>")
        # Create a dummy file for testing/demo if needed, or just exit
        sys.exit(1)
        
    plot_log(log_file)
