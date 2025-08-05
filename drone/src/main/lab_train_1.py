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

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")
RTSP_URL = f"rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live"

drone_should_land = False
drone_should_hover = False


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


def safe_move(drone, dx, dy, dz, dpsi):
    global drone_should_land, drone_should_hover
    if drone_should_land:
        print("[MOVE] Land requested. Skipping movement.")
        return
    if drone_should_hover:
        print("[MOVE] Hover requested. Holding position...")
        drone(FlyingStateChanged(state="hovering", _timeout=5)).wait()
        while not drone_should_land:
            time.sleep(0.5)
        return

    print(f"[MOVE] Executing moveBy: dx={dx}, dy={dy}, dz={dz}, dpsi={dpsi}")
    move = drone(
        moveBy(dx, dy, dz, dpsi)
        >> FlyingStateChanged(state="hovering", _timeout=10)
    ).wait()
    if not move.success():
        print("[MOVE] Movement failed!")


def scan_qr_for_20_seconds(drone):
    print("[QR] Connecting to drone video stream...")
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    if not cap.isOpened():
        print("[QR] Failed to open video stream")
        return []

    detector = cv2.QRCodeDetector()
    qr_found = []
    start_time = time.time()

    while time.time() - start_time < 20:
        ret, frame = cap.read()
        if not ret:
            continue

        data, bbox, _ = detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = np.int32(bbox)
            for i in range(len(bbox[0])):
                pt1 = tuple(bbox[0][i])
                pt2 = tuple(bbox[0][(i + 1) % len(bbox[0])])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            if data:
                cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if data and data not in [q[0] for q in qr_found]:
            print(f"[QR] QR Code detected: {data}")
            qr_found.append((data, time.time() - start_time))
            break

        # ✅ Show live drone video feed with QR overlay
        cv2.imshow("Drone QR Debug View", frame)
        if cv2.waitKey(1) & 0xFF == ord("q") or drone_should_land:
            break

    cap.release()
    cv2.destroyAllWindows()
    return qr_found


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

    # Step 5: Scan for QR
    print("[MAIN] Scanning for QR code...")
    qr_found = scan_qr_for_20_seconds(drone)

    if qr_found:
        print(f"[MAIN] QR code(s) detected: {qr_found}")
    else:
        print("[MAIN] No QR code found within time limit.")

    # Step 6: Go back (rotate back and return)
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)
    safe_move(drone, 1, 0, 0, 0)
    time.sleep(0.5)

    #Step 7: turn left
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)


    #Step 8: (not safe) go 2 m
    safe_move(drone, 1, 0, 0, 0)
    time.sleep(0.5)

    safe_move(drone, 1.2, 0, 0, 0)
    time.sleep(0.5)

    #Step 9: turn right -> approach 2nd qr
    safe_move(drone, 0, 0, 0, 1.5708)
    time.sleep(0.5)


    # Step 10: Scan for 2nd QR
    print("[MAIN] Scanning for QR code...")
    qr_found = scan_qr_for_20_seconds(drone)

    if qr_found:
        print(f"[MAIN] QR code(s) detected: {qr_found}")
    else:
        print("[MAIN] No QR code found within time limit.")


    #Step 11 turn left (not safeeeee!) 3 m forward -> approach 3rd qr
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)

    safe_move(drone, 1.5, 0, 0, 0)
    time.sleep(0.5)
    safe_move(drone, 1.5, 0, 0, 0)
    time.sleep(0.5)
    safe_move(drone, 1, 0, 0, 0)
    time.sleep(0.5)
    safe_move(drone, 1, 0, 0, 0)
    time.sleep(0.5)
    time.sleep(0.5)

    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)

    #Step 12: approach 3rd qr  
    safe_move(drone, 1.5, 0, 0, 0)
    time.sleep(0.5)

    #Step 13: Scan for 3rd QR
    print("[MAIN] Scanning for QR code...")
    qr_found = scan_qr_for_20_seconds(drone)

    if qr_found:
        print(f"[MAIN] QR code(s) detected: {qr_found}")
    else:
        print("[MAIN] No QR code found within time limit.")

    #Step 14 go back

    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)

    safe_move(drone, 1.1, 0, 0, 0)
    time.sleep(0.5)


    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)
    time.sleep(0.5)



    # Step 15: Land
    print("[MAIN] Landing...")
    drone(Landing()).wait()
    print("[MAIN] Drone landed.")

    drone.disconnect()
 

if __name__ == "__main__":
    main()
