#!/usr/bin/env python3
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.battery import capacity  # <-- battery (mAh)

import time
import threading
import cv2
import keyboard
import os
import numpy as np
import queue
import sys  # for immediate exit on emergency land
from pathlib import Path

# --- Paths: save logs alongside this code file ---
BASE_DIR = Path(__file__).resolve().parent
MISSION_LOG_DIR = BASE_DIR / "mission_logs"
MISSION_LOG_DIR.mkdir(parents=True, exist_ok=True)
QR_LOG_FILE = str(BASE_DIR / "qr_scan_log.txt")  # quick per-scan log

# --- Config / Globals ---
DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")  # optional, not used directly now

drone_should_land = False
drone_should_hover = False

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

# ---------- Helpers: time/battery/log files ----------
def now_wall():
    return time.strftime("%Y-%m-%d %H:%M:%S")

def get_battery_state(drone, retries=3, delay=0.5):
    """
    Returns (remaining_mAh, full_mAh, percent) or (None, None, None).
    """
    for _ in range(retries):
        st = drone.get_state(capacity)
        if st and "remaining" in st and "full_charge" in st:
            rem = float(st["remaining"])
            full = float(st["full_charge"]) if st["full_charge"] else None
            pct = (rem / full * 100.0) if (full and full > 0.0) else None
            return rem, full, pct
        time.sleep(delay)
    return None, None, None

def fmt_pct(p): return f"{p:.1f}%" if p is not None else "n/a"
def fmt_mAh(v): return f"{v:.0f} mAh" if v is not None else "n/a"

def log_qr_result(stage_label: str, found_list):
    """
    stage_label: e.g., "Scan 1/3"
    found_list: list of decoded QR strings (possibly empty)
    """
    ts = now_wall()
    status = "DETECTED" if found_list else "NOT DETECTED"
    line = f"[{ts}] {stage_label}: {status}"
    if found_list:
        line += f" | codes={'; '.join(found_list)}"
    with open(QR_LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")
    print(f"[LOG] {line}")

def write_mission_report(filepath: Path, text: str):
    filepath.parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"[MISSION] Wrote mission report: {filepath}")

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
    Start/stop Olympe streaming and scan for a fixed duration (early exit on detection).
    Tracks:
      - first_seen_time (perf_counter) when any bbox first appears
      - decode_time (perf_counter) when any decoded string first arrives
      - time_to_decode = decode_time - first_seen_time
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

        # Results + timings
        self.detected = set()
        self.first_seen_time = None   # perf_counter
        self.decode_time = None       # perf_counter

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

                # Detect+decode QR
                data, bbox, _ = self.qr_detector.detectAndDecode(bgr)

                # mark first seen when bbox first appears
                if bbox is not None and len(bbox) > 0 and self.first_seen_time is None:
                    self.first_seen_time = time.perf_counter()

                # Draw overlay
                if bbox is not None and len(bbox) > 0:
                    bb = np.int32(bbox)
                    for i in range(len(bb[0])):
                        p1 = tuple(bb[0][i]); p2 = tuple(bb[0][(i + 1) % len(bb[0])])
                        cv2.line(bgr, p1, p2, (0, 255, 0), 2)
                if data:
                    if self.decode_time is None:
                        self.decode_time = time.perf_counter()
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
        Start streaming, scan for up to duration_s seconds, then stop.
        Returns list of unique QR strings detected (in arbitrary order).
        Exits early as soon as a QR code is detected.
        Exposes self.first_seen_time and self.decode_time (perf_counter).
        """
        self.detected.clear()
        self.stop_event.clear()
        self.first_frame_evt.clear()
        self.first_seen_time = None
        self.decode_time = None

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
        start = time.perf_counter()
        try:
            while (time.perf_counter() - start) < duration_s and not self.stop_event.is_set():
                # Early exit: if any code was detected, stop right away
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

    mission_wall_start = now_wall()
    mission_perf_start = time.perf_counter()

    print("[MAIN] Connecting to drone...")
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    # Battery at mission start
    batt_start_mAh, batt_full_mAh, batt_start_pct = get_battery_state(drone)

    print("[MAIN] Starting keyboard listener...")
    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()

    print("[MAIN] Taking off...")
    drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait().success()
    print("[MAIN] Drone is hovering.")

    print("[MAIN] Setting max tilt to 5 (slower speed)...")
    drone(MaxTilt(5)).wait().success()

    # Metrics accumulator per scan
    scan_metrics = []

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
    batt_s1_start_mAh, _, batt_s1_start_pct = get_battery_state(drone)
    s1_perf_start = time.perf_counter()

    scanner = OlympeQRScanner(drone, window_name="Drone QR Debug View 1")
    qr_found = scanner.scan(duration_s=20, show_window=True)

    s1_perf_end = time.perf_counter()
    batt_s1_end_mAh, _, batt_s1_end_pct = get_battery_state(drone)

    log_qr_result("Scan 1/3", qr_found)

    scan_metrics.append({
        "label": "Scan 1/3",
        "detected_codes": qr_found,
        "scan_duration_s": s1_perf_end - s1_perf_start,
        "first_seen_to_decode_s": (scanner.decode_time - scanner.first_seen_time) if (scanner.decode_time and scanner.first_seen_time) else None,
        "battery_start_mAh": batt_s1_start_mAh,
        "battery_end_mAh": batt_s1_end_mAh,
        "battery_delta_mAh": (batt_s1_start_mAh - batt_s1_end_mAh) if (batt_s1_start_mAh is not None and batt_s1_end_mAh is not None) else None,
        "battery_start_pct": batt_s1_start_pct,
        "battery_end_pct": batt_s1_end_pct,
    })

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
    batt_s2_start_mAh, _, batt_s2_start_pct = get_battery_state(drone)
    s2_perf_start = time.perf_counter()

    scanner2 = OlympeQRScanner(drone, window_name="Drone QR Debug View 2")
    qr_found = scanner2.scan(duration_s=20, show_window=True)

    s2_perf_end = time.perf_counter()
    batt_s2_end_mAh, _, batt_s2_end_pct = get_battery_state(drone)

    log_qr_result("Scan 2/3", qr_found)

    scan_metrics.append({
        "label": "Scan 2/3",
        "detected_codes": qr_found,
        "scan_duration_s": s2_perf_end - s2_perf_start,
        "first_seen_to_decode_s": (scanner2.decode_time - scanner2.first_seen_time) if (scanner2.decode_time and scanner2.first_seen_time) else None,
        "battery_start_mAh": batt_s2_start_mAh,
        "battery_end_mAh": batt_s2_end_mAh,
        "battery_delta_mAh": (batt_s2_start_mAh - batt_s2_end_mAh) if (batt_s2_start_mAh is not None and batt_s2_end_mAh is not None) else None,
        "battery_start_pct": batt_s2_start_pct,
        "battery_end_pct": batt_s2_end_pct,
    })

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
    batt_s3_start_mAh, _, batt_s3_start_pct = get_battery_state(drone)
    s3_perf_start = time.perf_counter()

    scanner3 = OlympeQRScanner(drone, window_name="Drone QR Debug View 3")
    qr_found = scanner3.scan(duration_s=20, show_window=True)

    s3_perf_end = time.perf_counter()
    batt_s3_end_mAh, _, batt_s3_end_pct = get_battery_state(drone)

    log_qr_result("Scan 3/3", qr_found)

    scan_metrics.append({
        "label": "Scan 3/3",
        "detected_codes": qr_found,
        "scan_duration_s": s3_perf_end - s3_perf_start,
        "first_seen_to_decode_s": (scanner3.decode_time - scanner3.first_seen_time) if (scanner3.decode_time and scanner3.first_seen_time) else None,
        "battery_start_mAh": batt_s3_start_mAh,
        "battery_end_mAh": batt_s3_end_mAh,
        "battery_delta_mAh": (batt_s3_start_mAh - batt_s3_end_mAh) if (batt_s3_start_mAh is not None and batt_s3_end_mAh is not None) else None,
        "battery_start_pct": batt_s3_start_pct,
        "battery_end_pct": batt_s3_end_pct,
    })

    # Step 14 go back
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    safe_move(drone, 0.9, 0, 0, 0);     time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708); time.sleep(0.5)
    time.sleep(0.5)

    # Step 15: Move forward 1.5 m and do 360 scan (recording)
    print("[MAIN] Moving forward 1.5 m for 360 scan...")
    safe_move(drone, 1.5, 0, 0, 0); time.sleep(1)

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

    # Mission end metrics
    batt_end_mAh, _, batt_end_pct = get_battery_state(drone)

    drone.disconnect()

    mission_wall_end = now_wall()
    mission_perf_end = time.perf_counter()
    mission_duration_s = mission_perf_end - mission_perf_start
    total_mAh_used = (batt_start_mAh - batt_end_mAh) if (batt_start_mAh is not None and batt_end_mAh is not None) else None

    # Build mission report
    ts_label = time.strftime("%Y%m%d_%H%M%S")
    report_path = MISSION_LOG_DIR / f"mission_{ts_label}.txt"

    lines = []
    lines.append("Mission Report")
    lines.append("==============")
    lines.append(f"Start (wall): {mission_wall_start}")
    lines.append(f"End   (wall): {mission_wall_end}")
    lines.append(f"Duration    : {mission_duration_s:.2f} s")
    lines.append("")
    lines.append("Battery (overall)")
    lines.append("-----------------")
    lines.append(f"Start: {fmt_mAh(batt_start_mAh)} ({fmt_pct(batt_start_pct)})  | Full capacity: {fmt_mAh(batt_full_mAh)}")
    lines.append(f"End  : {fmt_mAh(batt_end_mAh)} ({fmt_pct(batt_end_pct)})")
    lines.append(f"Used : {fmt_mAh(total_mAh_used)}")
    lines.append("")
    lines.append("Per-Scan Details")
    lines.append("----------------")
    for sm in scan_metrics:
        lines.append(f"{sm['label']}:")
        lines.append(f"  Detected codes        : {', '.join(sm['detected_codes']) if sm['detected_codes'] else 'None'}")
        lines.append(f"  Scan duration         : {sm['scan_duration_s']:.2f} s")
        if sm["first_seen_to_decode_s"] is not None:
            lines.append(f"  First-seen → decoded  : {sm['first_seen_to_decode_s']:.3f} s")
        else:
            lines.append("  First-seen → decoded  : n/a")
        lines.append(f"  Battery @ start       : {fmt_mAh(sm['battery_start_mAh'])} ({fmt_pct(sm['battery_start_pct'])})")
        lines.append(f"  Battery @ end         : {fmt_mAh(sm['battery_end_mAh'])} ({fmt_pct(sm['battery_end_pct'])})")
        lines.append(f"  Battery used this scan: {fmt_mAh(sm['battery_delta_mAh'])}")
        lines.append("")

    write_mission_report(report_path, "\n".join(lines))
    print(f"[MAIN] Mission report saved to: {report_path}")
    print(f"[MAIN] QR scan log             : {QR_LOG_FILE}")


if __name__ == "__main__":
    main()
