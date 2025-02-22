import os
import queue
import threading
import olympe
import cv2
import numpy as np
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")

class RedObjectDetector:
    def __init__(self):
        self.drone = olympe.Drone(DRONE_IP)
        self.frame_queue = queue.Queue(maxsize=10)  # Limit queue size
        self.running = True

    def start(self):
        # Start the drone, stream video, and begin detection.
        self.drone.connect()
        self.drone.streaming.set_callbacks(raw_cb=self.yuv_frame_cb)
        self.drone.streaming.start()

        # Take off and wait until hovering
        self.drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait()
        print("Takeoff successful.")

        # Move forward by 2 meters to get closer
        self.move_forward(2.0)

        # Start the frame processing thread
        threading.Thread(target=self.process_frames, daemon=True).start()

    def stop(self):
        """Stop streaming, land the drone, and disconnect."""
        self.running = False
        self.drone.streaming.stop()
        self.drone(Landing()).wait()
        self.drone.disconnect()
        cv2.destroyAllWindows()

    def move_forward(self, distance):
        """Moves the drone forward by a given distance in meters."""
        if self.drone.get_state(FlyingStateChanged)["state"] != "hovering":
            print("Drone is not hovering. Cannot execute moveBy command.")
            return False

        move = self.drone(
            moveBy(distance, 0, 0, 0) >> FlyingStateChanged(state="hovering", _timeout=10)
        ).wait()

        if move.success():
            print(f"Drone moved forward by {distance} meters.")
            return True
        else:
            print("Failed to move the drone.")
            return False

    def yuv_frame_cb(self, yuv_frame):
        """Processes video frames from the drone."""
        yuv_frame.ref()
        self.frame_queue.put(yuv_frame.as_ndarray())
        yuv_frame.unref()

    def process_frames(self):
        """Continuously processes frames for red object detection."""
        while self.running:
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)  # Convert YUV to BGR
                self.detect_red_object(bgr_frame)

    def detect_red_object(self, frame):
        """Detects any red object in the frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for all shades of red
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create two masks and combine them
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        full_red_mask = mask1 | mask2  # Combine both masks

        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        full_red_mask = cv2.morphologyEx(full_red_mask, cv2.MORPH_OPEN, kernel)
        full_red_mask = cv2.morphologyEx(full_red_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(full_red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            print("Red object detected!")
            cv2.putText(frame, "Red Object Detected!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Show mask and frame for debugging (optional)
        cv2.imshow('Red Mask', full_red_mask)
        cv2.imshow('Frame', frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    detector = RedObjectDetector()
    try:
        detector.start()
        while True:  # Keep the program running
            pass
    except KeyboardInterrupt:
        detector.stop()
