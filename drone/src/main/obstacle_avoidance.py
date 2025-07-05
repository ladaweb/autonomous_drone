import argparse  # For parsing command-line arguments
import olympe  # Olympe SDK for controlling Parrot drones
import os  # For accessing environment variables
import re  # For regex operations
import sys  # To access command-line arguments
import cv2  # OpenCV for image processing
import time  # For time delays
from olympe.video.pdraw import Pdraw, PdrawState  # For handling video streaming from the drone
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing  # Drone piloting commands
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged  # Drone state events

# Get drone IP and RTSP port from environment or use default
DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")

# Detect obstacles in left, center, and right zones of the frame
def detect_obstacle_regions(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale
    blur = cv2.GaussianBlur(gray, (5, 5), 0)  # Blur to reduce noise
    edges = cv2.Canny(blur, 50, 150)  # Detect edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # Find contours

    # Define width zones for obstacle checking
    height, width = frame.shape[:2]
    left_zone = (0, int(width * 0.3))
    center_zone = (int(width * 0.35), int(width * 0.65))
    right_zone = (int(width * 0.7), width)

    # Initialize obstacle flags
    obstacle_left = obstacle_center = obstacle_right = False

    # Classify contour positions
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1500:  # Filter out small objects
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2  # Center x of bounding box
            if left_zone[0] <= cx < left_zone[1]:
                obstacle_left = True
            elif center_zone[0] <= cx < center_zone[1]:
                obstacle_center = True
            elif right_zone[0] <= cx < right_zone[1]:
                obstacle_right = True

    return obstacle_left, obstacle_center, obstacle_right  # Return detection results

def main(argv):
    # Parse command-line arguments for video stream URL
    parser = argparse.ArgumentParser(description="Drone obstacle avoidance test")
    parser.add_argument(
        "-u",
        "--url",
        default=f"rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live",
        help="RTSP URL for drone video stream",
    )
    parser.add_argument("-m", "--media-name", default="Front camera")
    args = parser.parse_args(argv)

    # Extract drone IP using regex from URL
    drone_ip = re.search(r"\d+\.\d+\.\d+\.\d+", args.url)
    if not drone_ip:
        print("Invalid drone IP in URL")
        return

    # Connect to drone
    drone = olympe.Drone(drone_ip.group())
    if not drone.connect():
        print("Failed to connect to drone")
        return

    # Initialize Pdraw to handle video streaming
    pdraw = Pdraw()
    pdraw.play(url=args.url, media_name=args.media_name)
    if not pdraw.wait(PdrawState.Playing, timeout=5):
        print("Failed to start video stream")
        drone.disconnect()
        pdraw.destroy()
        return

    # OpenCV video capture for RTSP stream
    cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # Set buffer to reduce lag

    try:
        # Drone takeoff and wait until hovering
        print("Taking off...")
        if not drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait().success():
            print("Takeoff failed")
            return

        print("Drone hovering. Starting obstacle check...")

        while True:
            ret, frame = cap.read()  # Read frame from drone
            if not ret:
                print("Failed to grab frame")
                continue

            # Detect obstacle locations in current frame
            obs_left, obs_center, obs_right = detect_obstacle_regions(frame)

            if obs_center:
                print("Obstacle ahead!")
                if not obs_left:
                    print("Turning left")
                    drone(moveBy(0, -0.3, 0, 0)).wait()  # Move left
                elif not obs_right:
                    print("Turning right")
                    drone(moveBy(0, 0.3, 0, 0)).wait()  # Move right
                else:
                    print("Blocked on all sides. Moving up.")
                    drone(moveBy(0, 0, -0.3, 0)).wait()  # Move up
            else:
                print("No obstacle ahead. Moving forward.")
                drone(moveBy(0.3, 0, 0, 0)).wait()  # Move forward

            # Show the drone camera feed
            cv2.imshow("Drone Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Quit on 'q'
                break

    finally:
        drone(Landing()).wait()  # Land the drone
        cap.release()  # Release video capture
        cv2.destroyAllWindows()  # Close OpenCV windows
        pdraw.close()  # Stop video stream
        drone.disconnect()  # Disconnect from drone
        pdraw.wait(PdrawState.Closed, timeout=5)  # Wait for shutdown
        pdraw.destroy()  # Cleanup pdraw

if __name__ == "__main__":
    main(sys.argv[1:])  # Run main with CLI arguments
