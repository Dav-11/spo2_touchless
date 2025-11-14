import serial
import numpy as np

# -------- CONFIG --------
PORT = "/dev/tty.usbserial-110"
BAUD = 115200

STORAGE_BFFER_SIZE = 200

ROI_W = 65
ROI_H = 65
BUFFER_LEN = 10
POINT_SIZE = 2               # R,G
FRAMEBLOCK_SIZE = BUFFER_LEN * ROI_H * ROI_W * POINT_SIZE

HEADER = b"FRAMEBLOCK"             # must match ESP32 send_full_buffer()

print(f"Connecting to {PORT}...")
ser = serial.Serial(PORT, BAUD, timeout=1)

try:
    ser.setDTR(False)
    ser.setRTS(False)
except Exception:
    pass

print(f"Listening on {PORT} at {BAUD} baud...\n")

print("Connected.\nWaiting for frame blocks...\n")

def read_exact(num_bytes):
    """Read exactly N bytes from serial (blocking)."""
    data = b""
    while len(data) < num_bytes:
        packet = ser.read(num_bytes - len(data))
        if not packet:
            continue
        data += packet
    return data

try:
    while True:

        # ---- 1. sync to header ----
        byte = ser.read(1)
        if not byte:
            continue

        if byte == HEADER[:1]:
            rest = ser.read(len(HEADER) - 1)
            if rest == HEADER[1:]:
                print("Header received → reading block...")

                # ---- 2. read binary block ----
                raw = read_exact(FRAMEBLOCK_SIZE)

                # ---- 3. convert to numpy array ----
                arr = np.frombuffer(raw, dtype=np.uint8)
                arr = arr.reshape((BUFFER_LEN, ROI_H, ROI_W, 2))

                print("Block received.")
                print("Shape:", arr.shape)
                print("Example point [frame0,0,0]:", arr[0,0,0])  # (R,G)

                # TODO:
                #  1) keep last STORAGE_BFFER_SIZE frames
                #  2) Draw a 2D graph of the size of the frame where each point is the Coefficient of variation of each pixel ?

except KeyboardInterrupt:
    print("Stopping...")

finally:
    ser.close()
    print("Serial closed.")
