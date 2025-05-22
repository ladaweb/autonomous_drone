import argparse
import olympe
import os
import re
import sys
import cv2
import webbrowser
from olympe.video.pdraw import Pdraw, PdrawState

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
                    webbrowser.open(data)
                    break
            except Exception as e:
                print(f"Error during QR code detection: {e}")
                continue

            # Display the video feed
            cv2.imshow("Drone QR Code Scanner", img)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) == ord("q"):
                break

    finally:
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        pdraw.close()
        drone.disconnect()
        pdraw.wait(PdrawState.Closed, timeout=5)
        pdraw.destroy()

if __name__ == "__main__":
    main(sys.argv[1:])