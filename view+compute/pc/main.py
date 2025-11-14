import os
import re
import serial
import pandas as pd
import time
from sklearn.linear_model import LinearRegression
import numpy as np

# -------- CONFIG --------
PORT = "/dev/tty.usbserial-110"   # change if needed
BAUD = 115200
CAL_FILE = "spo2_calibration.csv"  # saved paired data (camera_R, spo2_ref)
READINGS_FILE = "spo2_readings.csv"

# -------- REGEX --------
pattern = re.compile(
    r"\"r_ac\":(?P<r_ac>[\d.]+), \"r_dc\":(?P<r_dc>[\d.]+), "
    r"\"g_ac\":(?P<g_ac>[\d.]+), \"g_dc\":(?P<g_dc>[\d.]+)"
    # r"\"b_ac\":(?P<b_ac>[\d.]+), \"b_dc\":(?P<b_dc>[\d.]+)"
)

# -------- GLOBALS --------
model = None


# -------- FUNCTIONS --------
def load_model():
    """Load linear regression model from CSV calibration data."""
    global model
    if not os.path.exists(CAL_FILE):
        print("⚠️  No calibration file found. Please calibrate first (press 'c').")
        model = None
        return

    df = pd.read_csv(CAL_FILE)
    if len(df) < 2:
        print("⚠️  Not enough calibration points for regression. Need at least 2.")
        model = None
        return

    X = df["RoR"].values.reshape(-1, 1)
    y = df["SpO2_ref"].values
    reg = LinearRegression()
    reg.fit(X, y)
    model = reg
    print(f"📈 Loaded regression model: SpO₂ = {reg.coef_[0]:.3f} * RoR + {reg.intercept_:.3f}")


def save_calibration(ror, spo2_value):
    """Save a calibration pair (RoR, SpO₂_ref) to the CSV file."""
    df_new = pd.DataFrame([[ror, spo2_value]], columns=["RoR", "SpO2_ref"])
    if os.path.exists(CAL_FILE):
        df_existing = pd.read_csv(CAL_FILE)
        df = pd.concat([df_existing, df_new], ignore_index=True)
    else:
        df = df_new
    df.to_csv(CAL_FILE, index=False)
    print(f"✅ Saved calibration point: RoR={ror:.4f}, SpO₂={spo2_value}% -> {CAL_FILE}")

    # retrain the model
    load_model()

def save_ror(ratio_red, ratio_green, ror, spo2_value):
    df_new = pd.DataFrame([[ratio_red, ratio_green, ror, spo2_value]], columns=["Ratio Red", "Ratio Green", "RoR", "SpO2_value"])
    if os.path.exists(READINGS_FILE):
        df_existing = pd.read_csv(READINGS_FILE)
        df = pd.concat([df_existing, df_new], ignore_index=True)
    else:
        df = df_new
    df.to_csv(READINGS_FILE, index=False)


def calibration_mode(ror):
    """Pause data collection and ask user for manual SpO₂ input."""
    print("\n=== CALIBRATION MODE ===")
    print(f"Current RoR: {ror:.4f}")
    while True:
        spo2_input = input("Enter measured SpO₂ from pulse oximeter (or 'cancel'): ").strip()
        if spo2_input.lower() == "cancel":
            print("Calibration canceled.\n")
            return
        try:
            spo2_value = float(spo2_input)
            if not (50 <= spo2_value <= 100):
                print("Value out of realistic range (50–100%). Try again.")
                continue
            save_calibration(ror, spo2_value)
            print("Calibration complete.\n")
            return
        except ValueError:
            print("Invalid input. Please enter a numeric value or 'cancel'.")


# -------- SERIAL SETUP --------
print(f"Connecting to {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=1)

try:
    ser.setDTR(False)
    ser.setRTS(False)
except Exception:
    pass

print(f"Listening on {PORT} at {BAUD} baud...\n")

# Load regression model if available
load_model()

# -------- MAIN LOOP --------
try:
    while True:
        # check for calibration request
        if os.name == "posix":
            import select, sys
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if dr:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == "c":
                    if 'last_ror' in locals():
                        calibration_mode(last_ror)
                    else:
                        print("No valid RoR available yet.")

        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue



        match = pattern.search(line)
        if not match:
            print(f"{line}")
            continue

        try:
            r_ac = float(match.group("r_ac"))
            r_dc = float(match.group("r_dc"))
            g_ac = float(match.group("g_ac"))
            g_dc = float(match.group("g_dc"))

            ratio_red = r_ac / r_dc if r_dc != 0 else 0
            ratio_green = g_ac / g_dc if g_dc != 0 else 0
            ror = ratio_green / ratio_red if ratio_red != 0 else 0

            last_ror = ror  # store last valid RoR for calibration

            output = f"[ratio_red: {ratio_red:.4f}, ratio_green:{ratio_green:.4f}] RoR = {ror:.4f}"
            if model is not None:
                spo2_pred = model.predict(np.array([[ror]]))[0]
                output += f" → Estimated SpO₂ = {spo2_pred:.2f}%"

                save_ror(ratio_red, ratio_green, last_ror, spo2_pred)
            else:
                output += " (no calibration model)"
            print(output)

        except ValueError:
            continue

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    ser.close()
    print("Serial closed.")
