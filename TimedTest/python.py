import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
filename = 'peak_data.txt'
FSR_MHz = 1500.0        # Free Spectral Range of your cavity
Scan_Time_ms = 50.0      # Your sweep period (50ms)
Conversion = FSR_MHz / (Scan_Time_ms * 1000) # MHz per microsecond

def analyze():
    # 1. Load Data
    try:
        data = np.loadtxt(filename)
    except Exception as e:
        print(f"Error loading file: {e}")
        return

    # 2. Handle "Hops" (Wrap-around)
    # Since the data hops back up, we find the longest continuous segment 
    # of downward drift to get the most accurate jitter result.
    diffs = np.diff(data)
    jumps = np.where(np.abs(diffs) > 5000)[0] # Identify jumps > 5000us
    if len(jumps) > 0:
        # Use the segment between the first two jumps
        start, end = 0, jumps[0]
        data = data[start:end]
        print(f"Analyzing segment of {len(data)} points between wrap-arounds.")

    indices = np.arange(len(data))

    # 3. Detrending (Remove the systematic drift)
    p = np.polyfit(indices, data, 1)
    drift_line = np.polyval(p, indices)
    detrended = data - drift_line

    # 4. Statistics
    sigma_us = np.std(detrended)
    sigma_MHz = sigma_us * Conversion

    # 5. Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Histogram
    ax1.hist(detrended, bins=25, color='skyblue', edgecolor='black')
    ax1.set_title(f"Peak Detection Jitter (N={len(data)})")
    ax1.set_xlabel("Timing Deviation from Mean (μs)")
    ax1.set_ylabel("Counts")
    
    # Summary Box
    textstr = '\n'.join((
        f'Timing Jitter (σ): {sigma_us:.2f} μs',
        f'Freq Precision: {sigma_MHz:.3f} MHz',
        f'Target (Subhankar): 0.900 MHz'
    ))
    ax1.text(0.05, 0.95, textstr, transform=ax1.transAxes, fontsize=10,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.5))

    # Drift Plot
    ax2.plot(indices, data, 'k.', label='Raw Data (Drifting)')
    ax2.plot(indices, drift_line, 'r-', label='Linear Drift Trend')
    ax2.set_title("Systematic Timing Drift Analysis")
    ax2.set_xlabel("Consecutive Scan Index")
    ax2.set_ylabel("Timestamp (μs)")
    ax2.legend()

    plt.tight_layout()
    plt.savefig('jitter_results.png') # Saves the graph as an image
    print(f"Analysis complete. Results saved to 'jitter_results.png'.")
    print(f"Calculated Frequency Precision: {sigma_MHz:.3f} MHz")

if __name__ == "__main__":
    analyze()