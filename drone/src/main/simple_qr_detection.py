#!/usr/bin/env python3
import csv
import os
import queue
import tempfile
import threading
import time

import cv2
import numpy as np
import olympe

# Quieter logs
olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")  # e.g., "554"


class QRScanWithPreview:
    """
    - NO takeoff/movement
    - OpenCV preview window with QR overlay
    - Scans for full timeout_s seconds, collects ALL detections
    - Press 'q' to stop early
    """

    def __init__(self, timeout_s: int = 40, show_window: bool = True, window_name: str = "ANAFI QR"):
        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_qr_")
        print(f"[INFO] Output dir: {self.tempd}")

        # Optional stats file
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.tempd, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ["fps", "bitrate"])
        self.h264_stats_writer.writeheader()

        # Decoded frames pipeline
        self.frame_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self._yuv_frame_processing, daemon=True)

        # Shared preview buffer (shown from main thread)
        self.last_bgr = None
        self.last_lock = threading.Lock()

        self.running = False
        self.timeout_s = timeout_s
        self.qr_detector = cv2.QRCodeDetector()
        self.detected = set()                 # <- collect all QR strings
        self.stop_event = threading.Event()
        self._timeout_timer = None

        self.show_window = show_window
        self.window_name = window_name
        self.first_frame_evt = threading.Event()

    # ---------- lifecycle ----------

    def start(self):
        assert self.drone.connect(retry=3), "Failed to connect to drone"
        if DRONE_RTSP_PORT:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        # Optional: record stream
        self.drone.streaming.set_output_files(
            video=os.path.join(self.tempd, "streaming.mp4"),
            metadata=os.path.join(self.tempd, "streaming_metadata.json"),
        )

        # Register callbacks
        self.drone.streaming.set_callbacks(
            raw_cb=self._yuv_frame_cb,
            h264_cb=self._h264_frame_cb,
            flush_raw_cb=self._flush_cb,
        )

        self.drone.streaming.start()
        self.running = True
        self.processing_thread.start()

        if not self.first_frame_evt.wait(timeout=5.0):
            print("[ERR] No decoded YUV frames in 5s. Check codec install or switch camera to H.265.")

        # Watchdog timeout
        self._timeout_timer = threading.Timer(self.timeout_s, self._on_timeout)
        self._timeout_timer.daemon = True
        self._timeout_timer.start()

    def stop(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()

        # signal worker to finish
        self.running = False

        # Clear callbacks first (prevents new frames during teardown)
        try:
            self.drone.streaming.set_callbacks(raw_cb=None, h264_cb=None, flush_raw_cb=None)
        except Exception:
            pass

        time.sleep(0.3)  # small drain

        if self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)

        # Drain any queued frames
        try:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        except Exception:
            pass

        # Stop streaming
        try:
            self.drone.streaming.stop()
        except Exception:
            pass
        time.sleep(0.3)

        # Disconnect / close
        try:
            self.drone.disconnect()
        finally:
            try:
                self.h264_stats_file.close()
            except Exception:
                pass

        # Close window from main thread
        if self.show_window:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:
                pass

    # ---------- callbacks ----------

    def _yuv_frame_cb(self, yuv_frame):
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)
        self.first_frame_evt.set()

    def _flush_cb(self, stream):
        if stream.get("vdef_format") != olympe.VDEF_I420:
            return True
        try:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        except Exception:
            pass
        return True

    def _h264_frame_cb(self, h264_frame):
        # 1s sliding window bitrate/FPS 
        _, frame_size = h264_frame.as_ctypes_pointer()
        info = h264_frame.info()
        ts = info["ntp_raw_timestamp"]
        if not bool(info["is_sync"]):
            while self.h264_frame_stats and (self.h264_frame_stats[0][0] + 1e6) < ts:
                self.h264_frame_stats.pop(0)
            self.h264_frame_stats.append((ts, frame_size))
            fps = len(self.h264_frame_stats)
            bitrate = 8 * sum(sz for _, sz in self.h264_frame_stats)
            self.h264_stats_writer.writerow({"fps": fps, "bitrate": bitrate})

    # ---------- processing (worker thread) ----------

    def _yuv_frame_processing(self):
        cvt_map = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }

        while self.running and not self.stop_event.is_set():
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                fmt = yuv_frame.format()
                if fmt not in cvt_map:
                    continue

                yuv_np = yuv_frame.as_ndarray()
                bgr = cv2.cvtColor(yuv_np, cvt_map[fmt])

                # Detect QR (keep scanning; don't stop here)
                data, bbox, _ = self.qr_detector.detectAndDecode(bgr)
                if bbox is not None and len(bbox) > 0:
                    bb = np.int32(bbox)
                    for i in range(len(bb[0])):
                        p1 = tuple(bb[0][i]); p2 = tuple(bb[0][(i+1) % len(bb[0])])
                        cv2.line(bgr, p1, p2, (0, 255, 0), 2)
                if data:
                    if data not in self.detected:
                        self.detected.add(data)
                        print(f"[QR] Detected: {data}")
                    cv2.putText(bgr, data, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                # Publish latest frame for preview (main thread shows it)
                if self.show_window:
                    with self.last_lock:
                        self.last_bgr = bgr
            finally:
                yuv_frame.unref()

    # ---------- helpers ----------

    def _on_timeout(self):
        self.stop_event.set()

    def scan(self):
        self.start()
        start = time.time()
        try:
            while not self.stop_event.is_set():
                # Preview / UI loop in main thread
                if self.show_window:
                    with self.last_lock:
                        frame = None if self.last_bgr is None else self.last_bgr.copy()
                    if frame is not None:
                        cv2.imshow(self.window_name, frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.stop_event.set()
                            break
                # Timeout check (redundant to timer, but cheap)
                if time.time() - start >= self.timeout_s:
                    self.stop_event.set()
                time.sleep(0.01)
        finally:
            self.stop()
        return list(self.detected)


if __name__ == "__main__":
    scanner = QRScanWithPreview(timeout_s=40, show_window=True)
    results = scanner.scan()
    if results:
        print(f"[RESULT] QR codes detected: {results}")
    else:
        print("[RESULT] No QR detected within timeout.")
