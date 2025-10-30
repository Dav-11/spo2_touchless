import serial
import re
import time

# -------- CONFIG --------
PORT = "/dev/tty.usbserial-110"   # change if needed
BAUD = 115200

# -------- REGEX --------
pattern = re.compile(
    r"\"r_ac\":(?P<r_ac>[\d.]+), \"r_dc\":(?P<r_dc>[\d.]+), "
    r"\"g_ac\":(?P<g_ac>[\d.]+), \"g_dc\":(?P<g_dc>[\d.]+), "
    r"\"b_ac\":(?P<b_ac>[\d.]+), \"b_dc\":(?P<b_dc>[\d.]+)"
)

# -------- SERIAL SETUP --------
print(f"Connecting to {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=1)

try:
    ser.setDTR(False)   # clear DTR
    ser.setRTS(False)   # clear RTS
except Exception:
    # setDTR/setRTS not supported on some platforms; ignore
    pass

# time.sleep(2)
print(f"Listening on {PORT} at {BAUD} baud...\n")

# -------- MAIN LOOP --------
try:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        print(f"RECEIVED: {line}")

        match = pattern.search(line)
        if not match:
            continue

        try:
            r_ac = float(match.group("r_ac"))
            r_dc = float(match.group("r_dc"))
            g_ac = float(match.group("g_ac"))
            g_dc = float(match.group("g_dc"))

            ratio_red = r_ac / r_dc if r_dc != 0 else 0
            ratio_green = g_ac / g_dc if g_dc != 0 else 0
            ror = ratio_green / ratio_red if ratio_red != 0 else 0

            print(f"RoR = {ror:.4f}")
        except ValueError:
            continue

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    ser.close()
    print("Serial closed.")
