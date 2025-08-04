import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt

import time
import threading
import cv2
import keyboard

DRONE_IP = "192.168.42.1"

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


def scan_qr_for_15_seconds():
    print("[QR] Starting QR scan for 15 seconds...")
    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()
    start_time = time.time()
    while time.time() - start_time < 15:
        ret, frame = cap.read()
        if not ret:
            print("[QR] Frame not read correctly.")
            continue
        data, bbox, _ = detector.detectAndDecode(frame)
        if data:
            print(f"[QR] QR Code detected: {data}")
            break
        if drone_should_land:
            print("[QR] Land requested. Aborting QR scan.")
            break
    cap.release()
    cv2.destroyAllWindows()
    print("[QR] QR scan complete.")


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

    # Step 1 - Move forward 1.4478 m (144.78 cm)

    #up 0.5m
    safe_move(drone, 0, 0, -0.5, 0)
    time.sleep(0.5)
    safe_move(drone, 0, 0, -0.5, 0)
    time.sleep(0.5)
    safe_move(drone, 0, 0, -0.5, 0)
    time.sleep(0.5)

    print("[MAIN] Moving forward 144.78 cm")
    safe_move(drone, 1.1, 0, 0, 0)
    time.sleep(0.5)
    time.sleep(0.5)
    safe_move(drone, 0, 0, 0, -1.5708)
    time.sleep(0.5)


    print("[MAIN] Landing now after 10 second hover.")
    drone(Landing()).wait()
    print("[MAIN] Landed successfully.")

if __name__ == "__main__":
    main()
