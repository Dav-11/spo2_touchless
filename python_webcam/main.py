import cv2
import mediapipe as mp
import numpy as np
import time
from scipy.signal import butter, filtfilt
from sklearn.linear_model import LinearRegression
import pandas as pd
import argparse
import os

# ---- params ----
FPS = 30
WINDOW_SEC = 8          # seconds for AC/DC estimation
BP_LOW = 0.5            # Hz
BP_HIGH = 5.0           # Hz
CAL_FILE = "spo2_calibration.csv"  # saved paired data (camera_R, spo2_ref)

# ---- helpers ----
def bandpass_filter(signal, fs, low=0.5, high=5.0, order=3):
    ny = 0.5 * fs
    lown, highn = low/ny, high/ny
    b, a = butter(order, [lown, highn], btype='band')
    return filtfilt(b, a, signal, method="gust")

def get_forehead_roi_coords(landmarks, img_w, img_h, expand=0.3):
    # use top of forehead using landmarks indices from mediapipe face mesh
    # Mediapipe landmarks: 10 (forehead/forehead center), 151, 9, 454 roughly top area
    # We'll take a small rectangle around landmark 10
    lm = landmarks[10]
    cx = int(lm.x * img_w)
    cy = int(lm.y * img_h)
    # build a square ROI
    size = int(min(img_w, img_h) * 0.12 * (1+expand))  # adjustable
    x1 = max(0, cx - size//2)
    y1 = max(0, cy - size//2)
    x2 = min(img_w, cx + size//2)
    y2 = min(img_h, cy + size//2)
    return x1, y1, x2, y2

def compute_ac_dc(signal_window):
    # raw DC = mean of raw window; AC measure = std of bandpassed window
    dc = np.mean(signal_window)
    ac = np.std(signal_window)
    return ac, dc

# ---- main ----
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true",
                        help="Run in calibration mode to record paired samples.")
    args = parser.parse_args()

    mp_face = mp.solutions.face_mesh
    face_mesh = mp_face.FaceMesh(static_image_mode=False, max_num_faces=1, refine_landmarks=True)
    cap = cv2.VideoCapture(0)  # should work with builtin camera

    # ensure FPS known
    fps = cap.get(cv2.CAP_PROP_FPS) or FPS
    fs = fps

    # buffers
    win_len = int(WINDOW_SEC * fs)
    r_buf = []
    g_buf = []
    b_buf = []
    t_buf = []

    # calibration storage
    if os.path.exists(CAL_FILE):
        cal_df = pd.read_csv(CAL_FILE)
    else:
        cal_df = pd.DataFrame(columns=["Rratio", "SpO2_ref"])

    print("Press 'q' to quit. Press 'c' to capture a calibration sample (type reference SpO2).")
    print("If --calibrate is set, use 'c' to record samples; otherwise run live estimation.")
    model = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't read camera. Exiting.")
            break

        # converts the image color format from BGR (Blue, Green, Red) to RGB (Red, Green, Blue) using OpenCV.
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = img.shape
        
        # Run the MediaPipe Face Mesh model on the RGB image to detect forehead landmarks.
        results = face_mesh.process(img)
        if results.multi_face_landmarks:
            face = results.multi_face_landmarks[0]
            x1,y1,x2,y2 = get_forehead_roi_coords(face.landmark, w, h)
            roi = frame[y1:y2, x1:x2]
            
            # compute mean color
            mean_b = np.mean(roi[:,:,0]) # Blue channel
            mean_g = np.mean(roi[:,:,1]) # Green channel
            mean_r = np.mean(roi[:,:,2]) # Red channel
            
            # append to buffers
            b_buf.append(mean_b)
            g_buf.append(mean_g)
            r_buf.append(mean_r)
            t_buf.append(time.time())

            # draw ROI
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        else:
            # no face detected: append last values to keep buffer length consistent
            if r_buf:
                r_buf.append(r_buf[-1])
                g_buf.append(g_buf[-1])
                b_buf.append(b_buf[-1])
                t_buf.append(time.time())

        # keep buffer sizes
        if len(r_buf) > win_len:
            r_buf = r_buf[-win_len:]
            g_buf = g_buf[-win_len:]
            b_buf = b_buf[-win_len:]
            t_buf = t_buf[-win_len:]

        # compute ratio if window is full
        spo2_est = None
        if len(r_buf) >= win_len:
            r_arr = np.array(r_buf)
            g_arr = np.array(g_buf)
            b_arr = np.array(b_buf)
            
            # bandpass each (use sampling rate from measured timestamps)
            # approximate fs by num samples per second
            if len(t_buf) >= 2:

                # This estimates the sampling frequency in Hz for the current data window
                fs_est = len(t_buf) / (t_buf[-1] - t_buf[0] + 1e-6)
            else:
                fs_est = fs
            try:
                r_f = bandpass_filter(r_arr, fs_est, low=BP_LOW, high=BP_HIGH)
                g_f = bandpass_filter(g_arr, fs_est, low=BP_LOW, high=BP_HIGH)
                b_f = bandpass_filter(b_arr, fs_est, low=BP_LOW, high=BP_HIGH)
            except Exception as e:
                r_f, g_f, b_f = r_arr - np.mean(r_arr), g_arr - np.mean(g_arr), b_arr - np.mean(b_arr)
            
            # AC/DC (std/mean) per channel (window)
            ac_r, dc_r = compute_ac_dc(r_f), np.mean(r_arr)
            ac_g, dc_g = compute_ac_dc(g_f), np.mean(g_arr)
            ac_b, dc_b = compute_ac_dc(b_f), np.mean(b_arr)
            
            # pick ratio of red/blue or red/green depending on preference;
            # many works use R = (AC_r/DC_r) / (AC_b/DC_b)
            # compute (AC/DC) for red and blue:
            ac_r_val = np.std(r_f)
            ac_b_val = np.std(b_f)
            if dc_r == 0 or dc_b == 0:
                Rratio = np.nan
            else:
                Rratio = (ac_r_val / dc_r) / (ac_b_val / dc_b)

            print("Rratio: {:.3f}".format(Rratio))

            # If calibration model exists, predict
            if model is not None:
                spo2_est = model.predict(np.array([[Rratio]]))[0]
        # show onscreen
        disp = frame.copy()
        text = "SpO2: --"
        if spo2_est is not None and not np.isnan(spo2_est):
            text = f"SpO2_est: {spo2_est:.1f}%   R: {Rratio:.3f}"
        cv2.putText(disp, text, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,200,255), 2)
        cv2.imshow("Contactless SpO2", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('c'):
            # calibration capture: save current Rratio and ask user to enter reference SpO2
            if 'Rratio' in locals() and not np.isnan(Rratio):
                print(f"Captured R = {Rratio:.4f}. Enter reference SpO2 (leave blank to discard): ", end="", flush=True)
                s = input().strip()
                if s:
                    try:
                        val = float(s)
                        new_row = pd.DataFrame([{"Rratio": Rratio, "SpO2_ref": val}])
                        cal_df = pd.concat([cal_df, new_row], ignore_index=True)
                        cal_df.to_csv(CAL_FILE, index=False)
                        print("Saved calibration sample.")
                        # fit model if at least 3 samples
                        if len(cal_df) >= 3:
                            X = cal_df[["Rratio"]].values
                            y = cal_df["SpO2_ref"].values
                            model = LinearRegression().fit(X, y)
                            print("Fitted linear calibration model. Coef:", model.coef_, "Intercept:", model.intercept_)
                    except Exception as e:
                        print("Invalid entry:", e)
                else:
                    print("Discarded sample.")
            else:
                print("No valid Rratio computed yet; try again later.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
