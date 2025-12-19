import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import firwin, lfilter, filtfilt, welch, find_peaks

# --- Configuration ---
FS = 12.5  # Frames per second. CHANGE THIS to your actual video fps!
TOP_CV_PERCENTILE = 90  # We will take the top 10% of pixels with highest CV
CALIBRATION_FILE = "spo2_calibration.csv"  # Name of your calibration file


def load_data(filename):
    arr = np.load(filename)
    print(f"Loaded {filename} | Shape: {arr.shape}")
    return arr


def calculate_cv_and_mask(data):
    """
    Calculates Coefficient of Variation (CV = std/mean) for every pixel.
    Returns a mask for pixels with the highest CV.
    """
    # data shape: (Frames, H, W, Channels)

    # 1. Compute Mean and Std along time axis (axis 0)
    # Adding 1e-9 to mean to avoid division by zero
    pixel_mean = np.mean(data, axis=0)
    pixel_std = np.std(data, axis=0)

    # 2. Compute CV
    pixel_cv = pixel_std / (pixel_mean + 1e-9)

    # 3. Aggregate CV across channels (optional, here we average CV of R, G, B)
    # Alternatively, you could just look at Green channel CV
    avg_cv_map = np.mean(pixel_cv, axis=2)

    # 4. Determine Threshold
    thresh = np.percentile(avg_cv_map, TOP_CV_PERCENTILE)

    # 5. Create Mask
    mask = avg_cv_map >= thresh

    print(f"CV Threshold: {thresh:.4f}. Selected {np.sum(mask)} pixels out of {mask.size}.")

    # Visualization of the mask
    plt.figure(figsize=(5, 4))
    plt.imshow(mask, cmap='gray')
    plt.title(f"Pixels with Top {100 - TOP_CV_PERCENTILE}% CV")
    plt.axis('off')
    plt.show()

    return mask


def apply_fir_filters(signal, fs):
    """
    Applies FIR LPF and BPF.
    """
    nyquist = 0.5 * fs

    # --- Constraints for FIR taps ---
    # The number of taps (N) determines filter sharpness.
    # N must be less than the signal length (99 frames).
    # Usually N should be odd.
    num_taps = min(65, len(signal))
    if num_taps % 2 == 0: num_taps -= 1

    # --- Design Filters ---

    # 1. LPF: Cutoff 0.05 Hz (Extract DC)
    # Note: 0.05Hz is extremely low. For short videos, this essentially returns the mean.
    cutoff_lpf = 0.05
    b_lpf = firwin(num_taps, cutoff_lpf, fs=fs, pass_zero=True)

    # 2. BPF: 0.2 Hz to 2.5 Hz (Extract AC)
    cutoff_bpf = [0.2, 2.5]
    b_bpf = firwin(num_taps, cutoff_bpf, fs=fs, pass_zero=False)

    # --- Apply Filters ---
    # filtfilt applies filter forward and backward to ensure zero phase shift
    dc_signal = filtfilt(b_lpf, 1.0, signal)
    ac_signal = filtfilt(b_bpf, 1.0, signal)

    return dc_signal, ac_signal

def compute_ror(data, mask, fs):
    """
    Extracts signals from masked pixels, filters them, and computes RoR.
    """
    # 1. Extract raw signals for Red and Green
    # data: (Frames, H, W, 3) -> Mask applies to (H, W)

    # We take the mean of all selected pixels for every frame
    # Result shape: (Frames,)
    # Channel 0 = Red, Channel 1 = Green
    raw_red = np.mean(data[:, mask, 0], axis=1)
    raw_green = np.mean(data[:, mask, 1], axis=1)

    # 2. Filter Red Channel
    dc_curve_red, ac_curve_red = apply_fir_filters(raw_red, fs)

    # 3. Filter Green Channel
    dc_curve_green, ac_curve_green = apply_fir_filters(raw_green, fs)

    # 4. Compute Statistics for Formula
    # AC amplitude: Standard Deviation (RMS) of the band-passed signal
    # DC amplitude: Mean of the low-passed signal

    ac_val_red = np.std(ac_curve_red)
    dc_val_red = np.mean(dc_curve_red)

    ac_val_green = np.std(ac_curve_green)
    dc_val_green = np.mean(dc_curve_green)

    # 5. Compute Ratios
    # Protect against division by zero
    R_red = ac_val_red / (dc_val_red + 1e-9)
    R_green = ac_val_green / (dc_val_green + 1e-9)

    RoR = R_red / (R_green + 1e-9)

    # --- Plotting Signals ---
    time_axis = np.arange(len(raw_red)) / fs

    fig, ax = plt.subplots(2, 2, figsize=(12, 8))

    # Red Plots
    ax[0, 0].plot(time_axis, raw_red, label='Raw Red', alpha=0.5)
    ax[0, 0].plot(time_axis, dc_curve_red, label='DC (LPF 0.05)', color='darkred')
    ax[0, 0].set_title("Red: Raw & DC")
    ax[0, 0].legend()

    ax[0, 1].plot(time_axis, ac_curve_red, label='AC (BPF 0.2-2.5)', color='red')
    ax[0, 1].set_title("Red: AC Component")
    ax[0, 1].legend()

    # Green Plots
    ax[1, 0].plot(time_axis, raw_green, label='Raw Green', alpha=0.5)
    ax[1, 0].plot(time_axis, dc_curve_green, label='DC (LPF 0.05)', color='darkgreen')
    ax[1, 0].set_title("Green: Raw & DC")
    ax[1, 0].legend()

    ax[1, 1].plot(time_axis, ac_curve_green, label='AC (BPF 0.2-2.5)', color='green')
    ax[1, 1].set_title("Green: AC Component")
    ax[1, 1].legend()

    plt.tight_layout()
    plt.show()

    return RoR, (ac_val_red, dc_val_red, ac_val_green, dc_val_green)

def calibrate_spo2_from_csv(current_ror, csv_path):
    """
    Reads a CSV, fits a linear regression model, and predicts SpO2.
    CSV Format: ror_value, spo2_value
    """
    print(f"\n--- Calibrating using {csv_path} ---")

    # 1. Read CSV
    try:
        # Load data, skipping header
        data = np.loadtxt(csv_path, delimiter=',', skiprows=1)

        # Handle case with only 1 row of data
        if data.ndim == 1:
            data = data.reshape(1, -1)

        if data.shape[0] < 2:
            print("Error: Need at least 2 data points in CSV to fit a line.")
            return None

        X_train = data[:, 0] # ror_value
        y_train = data[:, 1] # spo2_value

    except Exception as e:
        print(f"Error reading CSV: {e}")
        return None

    # 2. Linear Regression (Fit Line: y = mx + c)
    # Degree 1 means linear fit
    slope, intercept = np.polyfit(X_train, y_train, 1)

    print(f"Model Fit: SpO2 = {slope:.4f} * RoR + {intercept:.4f}")

    # 3. Predict Current SpO2
    estimated_spo2 = slope * current_ror + intercept

    # 4. Plot Calibration Curve
    plt.figure(figsize=(8, 5))

    # Plot training data
    plt.scatter(X_train, y_train, color='blue', label='Calibration Data', alpha=0.6)

    # Plot regression line
    # Create a range of x values for the line
    x_range = np.linspace(min(min(X_train), current_ror) * 0.9,
                          max(max(X_train), current_ror) * 1.1, 100)
    y_range = slope * x_range + intercept
    plt.plot(x_range, y_range, 'b--', label='Regression Line')

    # Plot current measurement
    plt.scatter([current_ror], [estimated_spo2], color='red', s=100, zorder=5, marker='*', label='Current Reading')
    plt.annotate(f"  {estimated_spo2:.1f}%", (current_ror, estimated_spo2), color='red', fontweight='bold')

    plt.title(f"SpO2 Calibration (y = {slope:.2f}x + {intercept:.2f})")
    plt.xlabel("Ratio of Ratios (RoR)")
    plt.ylabel("SpO2 (%)")
    plt.legend()
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.show()

    return estimated_spo2

def plot_aggregated_psd_rgb(arr, fs=12.5, red_ch=0, green_ch=1, blue_ch=2):
    """
    Compute and plot aggregated PSD separately for Red, Green, and Blue channels.

    This function sums the PSD of every pixel in the ROI/Image to show the
    total spectral energy per channel.

    arr: shape (num_frames, H, W, num_channels)
    """

    num_frames, H, W, C = arr.shape

    # Configuration for the 3 channels
    channels_config = [
        {'idx': red_ch, 'name': 'Red', 'color': 'red'},
        {'idx': green_ch, 'name': 'Green', 'color': 'green'},
        {'idx': blue_ch, 'name': 'Blue', 'color': 'blue'}
    ]

    plt.figure(figsize=(12, 6))

    # --- Process each channel ---
    for config in channels_config:
        ch_idx = config['idx']
        name = config['name']
        col = config['color']

        # 1. Extract Data
        # Flatten H and W dimensions: (Frames, H, W) -> (Frames, Pixels)
        # This allows us to calculate Welch on all pixels at once (Vectorization)
        channel_data = arr[:, :, :, ch_idx].reshape(num_frames, -1)

        # 2. Compute PSD (Vectorized)
        # axis=0 ensures we calculate PSD along the time dimension
        freqs, psd_all_pixels = welch(channel_data, fs=fs,
                                      nperseg=min(256, num_frames),
                                      axis=0)

        # 3. Aggregate (Sum) PSDs across all pixels (axis 1)
        agg_psd = np.sum(psd_all_pixels, axis=1)

        # 4. Peak Detection
        # Threshold: 5% of the max peak for this specific channel
        peaks, _ = find_peaks(agg_psd, height=np.max(agg_psd) * 0.05, distance=3)

        # 5. Plotting
        plt.plot(freqs, agg_psd, color=col, label=f"{name} Aggregated PSD", lw=2, alpha=0.8)
        plt.scatter(freqs[peaks], agg_psd[peaks], color=col, s=40, zorder=5)

        # 6. Annotations
        for idx in peaks:
            freq_val = freqs[idx]
            psd_val = agg_psd[idx]

            # Offset text slightly differently for each color to avoid overlap
            # Red high, Green mid, Blue low (just a visual heuristic)
            offset_mult = 1.05 + (ch_idx * 0.03)

            plt.annotate(f"{freq_val:.2f} Hz",
                         xy=(freq_val, psd_val),
                         xytext=(freq_val, psd_val * offset_mult),
                         color=col,
                         fontsize=9,
                         fontweight='bold',
                         arrowprops=dict(arrowstyle="->", lw=0.7, color=col))

    # --- Final Formatting ---
    plt.title(f"Aggregated PSD — RGB Channels (Summed over {H}x{W} pixels)")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Summed Power Spectral Density")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.show()

def process_data():
    # Load Data (Replace with your actual path)
    data_arr = load_data("../04_2_pc_receive/frames_rg_888_64w-64h_20251205_162109.npy")

    # # --- MOCK DATA FOR DEMONSTRATION ---
    # # Creating a dummy array of shape (99, 120, 160, 3) with a pulse
    # # Remove this block when using real data
    # print("Generating mock data...")
    # T = 99
    # H, W = 120, 160
    # t = np.linspace(0, 3, T)
    # pulse = 10 * np.sin(2 * np.pi * 1.2 * t)  # 1.2 Hz pulse
    # data_arr = np.ones((T, H, W, 3)) * 100
    # # Add pulse to center pixels
    # data_arr[:, 50:70, 70:90, 0] += pulse[:, None, None]  # Red pulse
    # data_arr[:, 50:70, 70:90, 1] += (pulse[:, None, None] * 0.5)  # Green pulse
    # data_arr += np.random.normal(0, 1, data_arr.shape)  # Noise
    # # -----------------------------------

    # 1. Find ROI based on Highest CV
    mask = calculate_cv_and_mask(data_arr)

    # 2. Compute RoR
    ror_value, components = compute_ror(data_arr, mask, fs=FS)

    print("-" * 30)
    print(f"Red   | AC: {components[0]:.4f} | DC: {components[1]:.4f} | Ratio: {components[0] / components[1]:.5f}")
    print(f"Green | AC: {components[2]:.4f} | DC: {components[3]:.4f} | Ratio: {components[2] / components[3]:.5f}")
    print("-" * 30)
    print(f"Calculated RoR (R_red / R_green): {ror_value:.5f}")
    print("-" * 30)

    # PSD
    plot_aggregated_psd_rgb(data_arr, fs=FS)

    # 4. Calibration Layer

    # Generate a dummy CSV if it doesn't exist (FOR TESTING ONLY)
    if not os.path.exists(CALIBRATION_FILE):
        print(f"Creating dummy {CALIBRATION_FILE} for demonstration...")
        with open(CALIBRATION_FILE, "w") as f:
            f.write("ror_value,spo2_value\n")
            # Typical inverse relationship: Higher RoR = Lower SpO2
            f.write("0.6,99.0\n")
            f.write("0.8,97.0\n")
            f.write("1.0,95.0\n")
            f.write("1.3,90.0\n")

    spo2 = calibrate_spo2_from_csv(ror_value, CALIBRATION_FILE)

    if spo2 is not None:
        print(f"Final Estimated SpO2: {spo2:.2f}%")


if __name__ == '__main__':
    process_data()
