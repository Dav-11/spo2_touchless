import socket
from datetime import datetime

import numpy as np

# ---------------- CONFIG ----------------
HOST = '0.0.0.0'       # listen on all interfaces
PORT = 50000

ROI_WIDTH = 160
ROI_HEIGHT = 120
POINT_SIZE = 2         # RGB565 = 2 bytes
FRAMES_PER_CHUNK = 1   # must match what ESP32 sends at once

# File to save received frames
BASE_FILE_NAME = 'frames_rg_888'
now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
FILE_NAME = f"{BASE_FILE_NAME}_{ROI_WIDTH}w-{ROI_HEIGHT}h_{now_str}.npy"


def rgb565_buffer_to_rgb888(frame_565):
    """Convert an NxHxWx2 uint8 array (RGB565) to NxHxWx3 uint8 RGB888."""

    # frame_565 shape: H x W x 2 (uint8)
    # Combine low + high byte into uint16
    pixels = frame_565.astype(np.uint16)
    pixels = (pixels[..., 1] << 8) | pixels[..., 0]

    # Extract channels
    r = (pixels >> 11) & 0x1F
    g = (pixels >> 5) & 0x3F
    b = pixels & 0x1F

    # Convert to 8-bit
    r8 = (r << 3).astype(np.uint8)
    g8 = (g << 2).astype(np.uint8)
    b8 = (b << 3).astype(np.uint8)

    # Stack to RGB888 image
    rgb888 = np.stack([r8, g8, b8], axis=-1)

    return rgb888

def receive_exact(sock, size):
    """Receive exactly `size` bytes or raise."""
    buf = b''
    while len(buf) < size:
        data = sock.recv(size - len(buf))
        if not data:
            raise ConnectionError("Socket closed")
        buf += data
    return buf

def main():
    print(f"Listening on {HOST}:{PORT} ...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    
    conn, addr = s.accept()
    print(f"Connected by {addr}")

    samples = []

    try:
        while True:
            # 1) Compute expected size
            frame_size = ROI_WIDTH * ROI_HEIGHT * POINT_SIZE * FRAMES_PER_CHUNK

            # 2) Read frame batch
            frame_bytes = receive_exact(conn, frame_size)

            # 3) Read sample frequency (single float for batch)
            fs_bytes = receive_exact(conn, 4)
            fs = np.frombuffer(fs_bytes, dtype=np.float32)[0]

            # 4) Convert frames into array
            frames = np.frombuffer(frame_bytes, dtype=np.uint8)
            frames = frames.reshape((FRAMES_PER_CHUNK, ROI_HEIGHT, ROI_WIDTH, POINT_SIZE))  # shape: N x H x W x 2

            # 5) Store each frame + fs
            for i in range(FRAMES_PER_CHUNK):
                rgb888 = rgb565_buffer_to_rgb888(frames[i])
                samples.append({'fs': fs, 'frame': rgb888})

            print(f"Received batch of {FRAMES_PER_CHUNK} frames, fs={fs:.2f} Hz, total frames={len(samples)}")

    except KeyboardInterrupt:
        print("Stopping server...")
    except ConnectionError:
        print("Connection closed by client")

    # Save to .npy
    np.save(FILE_NAME, samples)
    print(f"Saved {len(samples)} frames to {FILE_NAME}")

if __name__ == "__main__":
    main()