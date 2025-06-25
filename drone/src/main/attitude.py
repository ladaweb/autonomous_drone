import argparse
import asyncio
import olympe
import os
import re
import sys
import cv2
import numpy as np
import time
import math
from pyzbar.pyzbar import decode
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AttitudeChanged, AltitudeChanged
from olympe.video.pdraw import Pdraw, PdrawState

# Fix asyncio issue with Olympe on Python >=3.10
asyncio.set_event_loop(asyncio.new_event_loop())

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")


def main(argv):
    parser = argparse.ArgumentParser(description="Drone QR Code Pose Logger")
    parser.add_argument(
        "-u", "--url", default=f"rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live",
        help="RTSP URL for drone video stream",
    )
    parser.add_argument("-m", "--media-name", default="Front camera")
    args = parser.parse_args(argv)

    drone_ip = re.search(r"\d+\.\d+\.\d+\.\d+", args.url)
    if not drone_ip:
        print("Invalid drone IP")
        return

    drone = olympe.Drone(drone_ip.group())
    if not drone.connect():
        print("Failed to connect to drone")
        return

    print("Taking off...")
    if not drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait().success():
        print("Takeoff failed")
        drone.disconnect()
        return
    print("Drone is hovering")

    # Initialize video stream
    pdraw = Pdraw()
    pdraw.play(url=args.url, media_name=args.media_name)
    if not pdraw.wait(PdrawState.Playing, timeout=5):
        print("Video stream failed")
        drone(Landing()).wait()
        drone.disconnect()
        pdraw.destroy()
        return

    cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    try:
        qr_detected = False
        move_up_attempts = 0
        last_move_time = time.time()
        timeout_durations = [6, 10]  # seconds
        qr_memory = {}

        while True:
            ret, img = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            sharpen_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
            sharpened = cv2.filter2D(gray, -1, sharpen_kernel)

            decoded_objects = decode(sharpened)
            for obj in decoded_objects:
                data = obj.data.decode("utf-8")

                # Get pose data
                attitude = drone.get_state(AttitudeChanged)
                altitude = drone.get_state(AltitudeChanged)
                pose = {
                    "yaw_deg": round(math.degrees(attitude["yaw"]), 2),
                    "pitch_deg": round(math.degrees(attitude["pitch"]), 2),
                    "roll_deg": round(math.degrees(attitude["roll"]), 2),
                    "altitude_m": round(altitude["altitude"], 2),
                    "timestamp": round(time.time(), 2)
                }

                print(f"✅ QR Code detected: {data}")
                print(f"📍 Pose at detection: {pose}")
                qr_memory[data] = pose
                qr_detected = True

                pts = obj.polygon
                for i in range(len(pts)):
                    pt1 = (pts[i].x, pts[i].y)
                    pt2 = (pts[(i + 1) % len(pts)].x, pts[(i + 1) % len(pts)].y)
                    cv2.line(img, pt1, pt2, (0, 255, 0), 2)
                cv2.putText(img, data, (pts[0].x, pts[0].y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                break

            cv2.imshow("QR Scanner", img)

            if qr_detected:
                print("Landing due to QR detection...")
                drone(Landing() >> FlyingStateChanged(state="landing", _timeout=10)).wait()
                break

            if move_up_attempts < len(timeout_durations):
                elapsed = time.time() - last_move_time
                if elapsed > timeout_durations[move_up_attempts]:
                    print(f"❌ No QR after {timeout_durations[move_up_attempts]}s → Moving up 20cm")
                    assert drone(
                        moveBy(0.0, 0.0, -0.2, 0.0) >> FlyingStateChanged(state="hovering", _timeout=10)
                    ).wait().success()
                    move_up_attempts += 1
                    last_move_time = time.time()

            if cv2.waitKey(1) == ord("q"):
                print("Manual quit — landing drone...")
                drone(Landing() >> FlyingStateChanged(state="landing", _timeout=10)).wait()
                break

    finally:
        print(f"\n📦 Final QR Memory (dead reckoning):\n{qr_memory}")
        cap.release()
        cv2.destroyAllWindows()
        pdraw.close()
        drone.disconnect()
        pdraw.wait(PdrawState.Closed, timeout=5)
        pdraw.destroy()
        print("Cleanup complete.")


if __name__ == "__main__":
    main(sys.argv[1:])
