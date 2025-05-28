import argparse
import olympe
import os
import re
import sys
import cv2
import webbrowser
import time
from olympe.video.pdraw import Pdraw, PdrawState
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")

def main(argv):
    parser = argparse.ArgumentParser(description="Drone QR Code Scanner")
    parser.add_argument(
        "-u",
        "--url",
        default=f"rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live",
        help="RTSP URL for drone video stream",
    )
    parser.add_argument("-m", "--media-name", default="Front camera")
    args = parser.parse_args(argv)

    # Extract drone IP from URL
    drone_ip = re.search(r"\d+\.\d+\.\d+\.\d+", args.url)
    if not drone_ip:
        print("Invalid drone IP in URL")
        return

    # Connect to the drone
    drone = olympe.Drone(drone_ip.group())
    if not drone.connect():
        print("Failed to connect to drone")
        return

    # Initialize OpenCV QR code detector
    detector = cv2.QRCodeDetector()

    # Initialize Pdraw for video streaming
    pdraw = Pdraw()
    pdraw.play(url=args.url, media_name=args.media_name)
    if not pdraw.wait(PdrawState.Playing, timeout=5):
        print("Failed to start video stream")
        drone.disconnect()
        pdraw.destroy()
        return

    # Configure OpenCV to use FFmpeg backend for better RTSP handling
    cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("Error: Could not open video stream")
        pdraw.close()
        drone.disconnect()
        pdraw.destroy()
        return

    # Set buffer size to reduce latency and packet loss
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    # State tracking
    has_taken_off = False
    qr_code_count = 0  # Track number of unique QR codes detected
    moving_right = False  # Track if drone is moving right

    try:
        while True:
            # Read frame with error handling
            ret, img = cap.read()
            if not ret:
                print("Failed to grab frame, retrying...")
                # Attempt to reopen the stream
                cap.release()
                cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
                if not cap.isOpened():
                    print("Error: Could not reopen video stream")
                    break
                continue

            # Detect and decode QR code
            try:
                data, bbox, _ = detector.detectAndDecode(img)
                if data:
                    print(f"QR Code detected: {data}")
                    if not has_taken_off:
                        # First QR code: Take off
                        print("Initiating drone takeoff...")
                        if drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success():
                            print("Drone has taken off and is hovering")
                            has_taken_off = True
                            qr_code_count += 1
                            time.sleep(5)  # Stabilize after takeoff
                        else:
                            print("Failed to take off")
                    elif qr_code_count == 1 and not moving_right:
                        # Second QR code: Start moving right
                        print("Second QR code detected, moving right...")
                        qr_code_count += 1
                        moving_right = True
                        webbrowser.open(data)
                    elif qr_code_count == 2 and moving_right:
                        # Third QR code: Stop moving, move right again
                        print("Third QR code detected, moving right again...")
                        qr_code_count += 1
                        moving_right = False  # Stop first movement
                        if drone(moveBy(1.0, 0, 0, 0) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success():
                            print("Moved right after third QR code")
                            moving_right = True  # Resume moving right for final scan
                        else:
                            print("Failed to move right")
                        webbrowser.open(data)
                    elif qr_code_count == 3:
                        # Final QR code: Land the drone
                        print("Final QR code detected, landing drone...")
                        qr_code_count += 1
                        moving_right = False
                        if drone(Landing()).wait().success():
                            print("Drone has landed")
                        else:
                            print("Failed to land")
                        webbrowser.open(data)
                        break  # Exit loop after landing

            except Exception as e:
                print(f"Error during QR code detection: {e}")

            # Move right slowly if in moving_right state
            if moving_right and qr_code_count in [2, 3]:
                if drone(moveBy(0.5, 0, 0, 0) >> FlyingStateChanged(state="hovering", _timeout=5)).wait().success():
                    print("Moving right slowly...")
                else:
                    print("Failed to move right")
                time.sleep(1)  # Short delay between movements to keep it slow

            # Display the video feed
            cv2.imshow("Drone QR Code Scanner", img)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) == ord("q"):
                print("Manual exit triggered, landing drone...")
                if has_taken_off:
                    drone(Landing()).wait()
                break

    finally:
        # Cleanup
        if has_taken_off and qr_code_count < 4:
            print("Ensuring drone lands before cleanup...")
            drone(Landing()).wait()
        cap.release()
        cv2.destroyAllWindows()
        pdraw.close()
        drone.disconnect()
        pdraw.wait(PdrawState.Closed, timeout=5)
        pdraw.destroy()

if __name__ == "__main__":
    main(sys.argv[1:])