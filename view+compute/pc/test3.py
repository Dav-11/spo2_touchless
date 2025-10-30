import serial
import time

# Adjust the port if needed (check with ls /dev/tty.*)
PORT = '/dev/cu.usbserial-110'
BAUD = 115200

# Open serial
ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(2)  # wait for the board to reset when serial connects (important for Arduinos)

print(f"Connected to {PORT} at {BAUD} baud.")
print("Reading...\n")

while True:
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode(encoding='ascii', errors='replace').strip()
            if line:
                print(line)
    except KeyboardInterrupt:
        print("\nExiting...")
        break
    except Exception as e:
        print(f"Error: {e}")
        break

ser.close()
