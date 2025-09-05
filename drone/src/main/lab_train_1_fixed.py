#!/usr/bin/env python3
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt

import time
import threading
import cv2
import keyboard
import os
import numpy as np
import queue
import sys  # for immediate exit on emergency land

# --- Config / Globals ---
DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")  # optional, not used directly now

drone_should_land = False
drone_should_hover = False

# QR results log file
QR_LOG_FILE = "qr_scan_log.txt"

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


# ---------- Small helper: append QR scan results to a text file ----------
def log_qr_result(stage_label: str, found_list):
    """
    stage_label: e.g., "Scan 1/3", "Scan 2/3", "Scan 3/3"
    found_list: list of decoded QR strings (possibly empty)
    """
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    status = "DETECTED" if found_list else "NOT DETECTED"
    line = f"[{ts}] {stage_label}: {status}"
    if found_list:
        line += f" | codes={'; '.join(found_list)}"
    with open(QR_LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")
    print(f"[LOG] {line}")


# ---------- Keyboard handling ----------
def keyboard_listener():
    global drone_should_land, drone_should_hover
    while True:
        if keyboard.is_pressed("q"):
            print("[KEYBOARD] Emergency land triggered (q pressed)!")
            drone_should_land = True
            break
        elif keyboard.is_pressed("s"):
            print("[KEYBOARD] Hover triggered (s pressed)!")
            drone_should_hover = True


# ---------- Movement helper (UPDATED: immediate land / hover-then-land) ----------
def safe_move(drone, dx, dy, dz, dpsi):
    global drone_should_land, drone_should_hover

    # Immediate emergency land: stop all remaining steps
    if drone_should_land:
        print("[MOVE] Emergency land requested! Landing now.")
        try:
            drone(Landing()).wait()
        finally:
            sys.exit(0)

    # Hover mode: stay put, but allow switching to emergency land (q) at any time
    if drone_should_hover:
        print("[MOVE] Hover requested. Holding position...")
        drone(FlyingStateChanged(state="hovering", _timeout=5)).wait()
        while True:
            if drone_should_land:
                print("[MOVE] Emergency land requested during hover! Landing now.")
                try:
                    drone(Landing()).wait()
                finally:
                    sys.exit(0)
            time.sleep(0.1)

    # Normal movement
    print(f"[MOVE] Executing moveBy: dx={dx}, dy={dy}, dz={dz}, dpsi={dpsi}")
    move = drone(
        moveBy(dx, dy, dz, dpsi)
        >> FlyingStateChanged(state="hovering", _timeout=10)
    ).wait()
    if not move.success():
        print("[MOVE] Movement failed!")


# ---------- QR Scanner using Olympe decoded frames ----------
class OlympeQRScanner:
    """
    Start/stop Olympe streaming and scan for QR codes for a fixed duration.
    - Uses raw YUV decoded frames (works for H.264/H.265).
    - Shows a live OpenCV window with overlays (press 'q' to abort scan early).
    - Collects all unique QR strings seen during the scan window.
    - Cleans up callbacks and streaming cleanly to avoid teardown races.
    """
    def __init__(self, drone, window_name="Drone QR Debug View"):
        self.drone = drone
        self.window_name = window_name

        self.qr_detector = cv2.QRCodeDetector()

        # Frame pipeline
        self.frame_queue = queue.Queue()
        self.worker = None
        self.running = False

        # Preview buffer for main-thread display
        self.last_bgr = None
        self.last_lock = threading.Lock()

        # Events
        self.stop_event = threading.Event()
        self.first_frame_evt = threading.Event()

        # YUV->BGR conversion map
        self.cvt_map = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }

        # Results
        self.detected = set()

    # ---- Olympe callbacks ----
    def _yuv_frame_cb(self, yuv_frame):
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)
        self.first_frame_evt.set()

    def _flush_raw_cb(self, stream):
        if stream.get("vdef_format") != olympe.VDEF_I420:
            return True
        try:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        except Exception:
            pass
        return True

    def _h264_frame_cb(self, h264_frame):
        pass

    # ---- Worker thread: decode and detect ----
    def _worker_loop(self, show_window):
        while self.running and not self.stop_event.is_set():
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                fmt = yuv_frame.format()
                if fmt not in self.cvt_map:
                    continue

                yuv_np = yuv_frame.as_ndarray()
                bgr = cv2.cvtColor(yuv_np, self.cvt_map[fmt])

                # Detect QR
                data, bbox, _ = self.qr_detector.detectAndDecode(bgr)

                # Draw overlay
                if bbox is not None and len(bbox) > 0:
                    bb = np.int32(bbox)
                    for i in range(len(bb[0])):
                        p1 = tuple(bb[0][i]); p2 = tuple(bb[0][(i + 1) % len(bb[0])])
                        cv2.line(bgr, p1, p2, (0, 255, 0), 2)
                if data:
                    if data not in self.detected:
                        self.detected.add(data)
                        print(f"[QR] Detected: {data}")
                    cv2.putText(bgr, data, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                # Publish latest frame for preview
                if show_window:
                    with self.last_lock:
                        self.last_bgr = bgr
            finally:
                yuv_frame.unref()

    # ---- Public API ----
    def scan(self, duration_s=20, show_window=True):
        """
        Start streaming, scan for QR for up to duration_s seconds, then stop.
        Returns list of unique QR strings detected (in arbitrary order).
        Exits early as soon as a QR code is detected.
        """
        self.detected.clear()
        self.stop_event.clear()
        self.first_frame_evt.clear()

        # Register callbacks and start streaming
        self.drone.streaming.set_callbacks(
            raw_cb=self._yuv_frame_cb,
            h264_cb=self._h264_frame_cb,
            flush_raw_cb=self._flush_raw_cb,
        )
        self.drone.streaming.start()

        self.running = True
        self.worker = threading.Thread(target=self._worker_loop, args=(show_window,), daemon=True)
        self.worker.start()

        # Wait briefly for first decoded frame
        if not self.first_frame_evt.wait(timeout=5.0):
            print("[ERR] No decoded frames in 5s. Check camera encoding (H.265 recommended) or install codecs.")

        # Main-thread preview / timing loop (with EARLY EXIT on detection)
        start = time.time()
        try:
            while time.time() - start < duration_s and not self.stop_event.is_set():
                # ✅ EARLY EXIT: if any code was detected, stop right away
                if self.detected:
                    print("[QR] Early exit: QR code detected.")
                    break

                if show_window:
                    with self.last_lock:
                        frame = None if self.last_bgr is None else self.last_bgr.copy()
                    if frame is not None:
                        cv2.imshow(self.window_name, frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            print("[QR] Scan aborted by user")
                            break
                time.sleep(0.01)
        finally:
            # Clean shutdown
            self.running = False
            try:
                self.drone.streaming.set_callbacks(raw_cb=None, h264_cb=None, flush_raw_cb=None)
            except Exception:
                pass

            time.sleep(0.2)
            if self.worker and self.worker.is_alive():
                self.worker.join(timeout=1.5)

            try:
                while not self.frame_queue.empty():
                    self.frame_queue.get_nowait().unref()
            except Exception:
                pass

            try:
                self.drone.streaming.stop()
            except Exception:
                pass

            if show_window:
                try:
                    cv2.destroyWindow(self.window_name)
                except Exception:
                    pass

        return list(self.detected)


    def record_360_scan(drone):
        """Record a 360-degree scan video using OpenCV."""
        log_message("Setting up 360 scan video recording")
        scan_start_time = time.time()
        video_filename = "360_scan.mp4"
        cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

        if not cap.isOpened():
            log_message("Failed to open video stream for 360 scan", scan_start_time)
            return None, None

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 30
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        out = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        if not out.isOpened():
            log_message("Failed to initialize video writer for 360 scan", scan_start_time)
            cap.release()
            return None, None

        log_message(f"Recording started: {video_filename}")
        return cap, out



# ---------- Main mission ----------
def main():
    global drone_should_land, drone_should_hover

    print("[MAIN] Connecting to drone...")
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    print("[MAIN] Starting keyboard listener...")
    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()

    print("[MAIN] Taking off...")
    drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait().success()
    print("[MAIN] Drone is hovering.")

    print("[MAIN] Setting max tilt to 5 (slower speed)...")
    drone(MaxTilt(5)).wait().success()

    # Step 1: Move up 1.5 m (in 3 steps)
    for _ in range(3):
        safe_move(drone, 0, 0, -0.5, 0)
        time.sleep(0.5)

    # Step 2: Move forward 1.12 m
    print("[MAIN] Moving forward 1.12 m")
    safe_move(drone, 1.12, 0, 0, 0)
    time.sleep(0.5)

    # Step 3: Rotate 90° left (counter-clockwise)
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)

    # Step 4: Approach QR area
    safe_move(drone, 1.4, 0, 0, 0)
    time.sleep(0.5)

    # Step 5: Scan for QR (up to 20s, early exit on detection)
    print("[MAIN] Scanning for QR code (1/3)...")
    scanner = OlympeQRScanner(drone, window_name="Drone QR Debug View 1")
    qr_found = scanner.scan(duration_s=20, show_window=True)
    if qr_found:
        print(f"[MAIN] QR code(s) detected: {qr_found}")
    else:
        print("[MAIN] No QR code found within time limit.")
    log_qr_result("Scan 1/3", qr_found)

    # Step 6: Go back (rotate back and return)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 1, 0, 0, 0);       time.sleep(0.5)

    # Step 7: turn left
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)

    # Step 8: (not safe) go 2 m
    safe_move(drone, 1, 0, 0, 0);   time.sleep(0.5)
    safe_move(drone, 1.2, 0, 0, 0); time.sleep(0.5)

    # Step 9: turn right -> approach 2nd qr
    safe_move(drone, 0, 0, 0, 1.5708); time.sleep(0.5)

    # Step 10: Scan for 2nd QR (up to 20s, early exit on detection)
    print("[MAIN] Scanning for QR code (2/3)...")
    scanner2 = OlympeQRScanner(drone, window_name="Drone QR Debug View 2")
    qr_found = scanner2.scan(duration_s=20, show_window=True)
    if qr_found:
        print(f"[MAIN] QR code(s) detected: {qr_found}")
    else:
        print("[MAIN] No QR code found within time limit.")
    log_qr_result("Scan 2/3", qr_found)

    # Step 11 turn left  -> approach 3rd qr
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 1.5, 0, 0, 0);     time.sleep(0.5)
    safe_move(drone, 1.5, 0, 0, 0);     time.sleep(0.5)
    safe_move(drone, 1, 0, 0, 0);       time.sleep(0.5)
    safe_move(drone, 0.8, 0, 0, 0);     time.sleep(0.5)
    time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)

    # Step 12: approach 3rd qr
    safe_move(drone, 1.5, 0, 0, 0); time.sleep(0.5)

    # Step 13: Scan for 3rd QR (up to 20s, early exit on detection)
    print("[MAIN] Scanning for QR code (3/3)...")
    scanner3 = OlympeQRScanner(drone, window_name="Drone QR Debug View 3")
    qr_found = scanner3.scan(duration_s=20, show_window=True)
    if qr_found:
        print(f"[MAIN] QR code(s) detected: {qr_found}")
    else:
        print("[MAIN] No QR code found within time limit.")
    log_qr_result("Scan 3/3", qr_found)

    # Step 14 go back
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 0.9, 0, 0, 0);     time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    time.sleep(0.5)

    # Step 15: Move forward 1.5 m and do 360 scan (recording)
    print("[MAIN] Moving forward 1.5 m for 360 scan...")
    safe_move(drone, 1.5, 0, 0, 0); time.sleep(1)

    # Start video recording via streaming pipeline
    print("[VIDEO] Setting up 360 scan video recording...")
    video_filename = "360_scan.mp4"
    metadata_filename = "360_scan_metadata.json"
    drone.streaming.set_output_files(video=video_filename, metadata=metadata_filename)
    drone.streaming.start()
    print(f"[VIDEO] Recording started: {video_filename}")

    print("[MAIN] Performing 360-degree scan...")
    for _ in range(4):
        safe_move(drone, 0, 0, 0, 1.5708)  # 90 degrees
        time.sleep(2)  # Simulate scan delay

    print("[VIDEO] Stopping 360 scan recording...")
    drone.streaming.stop()
    print(f"[VIDEO] Video saved to {video_filename}")

    time.sleep(2)
    # Step 16: turn right right to go to initial position
    safe_move(drone, 0, 0, 0, 1.5708); time.sleep(1)
    safe_move(drone, 0, 0, 0, 1.5708); time.sleep(1)

    # Step 17: Move back 1.5 m
    print("[MAIN] Returning back 1.5 m to original position...")
    safe_move(drone, 1.5, 0, 0, 0); time.sleep(1)

    # Step 18: Land
    print("[MAIN] Landing...")
    drone(Landing()).wait()
    print("[MAIN] Drone landed.")

    drone.disconnect()


if __name__ == "__main__":
    main()
