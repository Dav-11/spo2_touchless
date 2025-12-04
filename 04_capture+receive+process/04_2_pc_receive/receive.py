import socket
from datetime import datetime

import numpy as np
import csv
import cv2  # OpenCV for video writing

# ---------------- CONFIG ----------------
HOST = '0.0.0.0'
PORT = 50000

ROI_WIDTH = 64
ROI_HEIGHT = 64
POINT_SIZE = 2  # RGB565 = 2 bytes
FRAMES_PER_CHUNK = 1
MAX_FRAMES = 350

BASE_FILE_NAME = 'frames_rg_888'
now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
NPY_FILE_NAME = f"{BASE_FILE_NAME}_{ROI_WIDTH}w-{ROI_HEIGHT}h_{now_str}.npy"
CSV_FILE_NAME = f"{BASE_FILE_NAME}_{ROI_WIDTH}w-{ROI_HEIGHT}h_{now_str}_fs.csv"
VIDEO_FILE_NAME = f"{BASE_FILE_NAME}_{ROI_WIDTH}w-{ROI_HEIGHT}h_{now_str}.mp4"

RAW_DUMP_FILE = "raw_frame_dump.bin"  #### NEW


def rgb565_buffer_to_rgb888_pad(frame_565):
    pixels = frame_565.astype(np.uint16)

    # Little endian conversion (our best guess for ESP32)
    pixels = (pixels[..., 1] << 8) | pixels[..., 0]

    r = (pixels >> 11) & 0x1F
    g = (pixels >> 5) & 0x3F
    b = pixels & 0x1F

    r8 = ((r << 3) | (r >> 2)).astype(np.uint8)
    g8 = ((g << 2) | (g >> 4)).astype(np.uint8)
    b8 = ((b << 3) | (b >> 2)).astype(np.uint8)

    rgb888 = np.stack([r8, g8, b8], axis=-1)
    return rgb888

def rgb565_buffer_to_rgb888_pad_BE(frame_565):

    # Big-endian: [high][low]
    pixels = (frame_565[..., 0].astype(np.uint16) << 8) | frame_565[..., 1].astype(np.uint16)

    r = (pixels >> 11) & 0x1F
    g = (pixels >> 5) & 0x3F
    b = pixels & 0x1F

    r8 = ((r << 3) | (r >> 2)).astype(np.uint8)
    g8 = ((g << 2) | (g >> 4)).astype(np.uint8)
    b8 = ((b << 3) | (b >> 2)).astype(np.uint8)

    rgb888 = np.stack([r8, g8, b8], axis=-1)
    return rgb888


def receive_exact(sock, size):
    buf = b''
    while len(buf) < size:
        data = sock.recv(size - len(buf))
        if not data:
            raise ConnectionError("Socket closed")
        buf += data
    return buf


def save_frames_as_video(frames_array, output_file, fps):
    height, width = frames_array.shape[1:3]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
    for frame in frames_array:
        out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    out.release()
    print(f"Saved video to {output_file} at {fps:.2f} FPS")


def main():
    print(f"Listening on {HOST}:{PORT} ...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)

    conn, addr = s.accept()
    print(f"Connected by {addr}")

    samples = []
    fs_list = []
    raw_dump_done = False  #### NEW FLAG

    try:
        frame_counter = 0

        while frame_counter < MAX_FRAMES:
            frame_size = ROI_WIDTH * ROI_HEIGHT * POINT_SIZE * FRAMES_PER_CHUNK

            # Receive raw bytes
            frame_bytes = receive_exact(conn, frame_size)

            # #### Save exactly 1 raw frame for debugging ####
            if not raw_dump_done:
                with open(RAW_DUMP_FILE, "wb") as f:
                    f.write(frame_bytes)
                raw_dump_done = True
                print(f"Saved RAW frame dump to {RAW_DUMP_FILE}")

            # Receive fs
            fs_bytes = receive_exact(conn, 4)
            fs = np.frombuffer(fs_bytes, dtype=np.float32)[0]

            frames = np.frombuffer(frame_bytes, dtype=np.uint8)
            frames = frames.reshape(FRAMES_PER_CHUNK, ROI_HEIGHT, ROI_WIDTH, POINT_SIZE)

            for i in range(FRAMES_PER_CHUNK):
                rgb888 = rgb565_buffer_to_rgb888_pad_BE(frames[i])

                samples.append(rgb888)
                fs_list.append({'frame_id': frame_counter, 'fs': fs})
                frame_counter += 1

            print(f"Received {FRAMES_PER_CHUNK} frames, fs={fs:.2f} Hz, total={len(samples)}")

    except KeyboardInterrupt:
        print("Stopping server...")
    except ConnectionError:
        print("Connection closed by client")

    # Save all data
    frames_array = np.stack(samples, axis=0)
    np.save(NPY_FILE_NAME, frames_array)
    print(f"Saved {len(frames_array)} frames to {NPY_FILE_NAME}")

    with open(CSV_FILE_NAME, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=['frame_id', 'fs'])
        writer.writeheader()
        writer.writerows(fs_list)
    print(f"Saved frame metadata to {CSV_FILE_NAME}")

    #avg_fs = np.mean([f['fs'] for f in fs_list]) if fs_list else 12.5
    avg_fs = 12.5
    save_frames_as_video(frames_array, VIDEO_FILE_NAME, fps=avg_fs)


if __name__ == "__main__":
    main()
