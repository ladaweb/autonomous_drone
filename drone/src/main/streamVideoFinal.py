#!/usr/bin/env python

import csv
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import shutil
import time
import olympe

from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
from olympe.messages.gimbal import set_target
from olympe.messages.camera import set_camera_mode
from olympe.video.renderer import PdrawRenderer

olympe.log.update_config({"loggers": {"olympe": {"level": "INFO"}}})

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")

class StreamingExample:
    def __init__(self):
        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="drone_")
        print(f"Olympe streaming output dir: {self.tempd}")

        self.frame_queue = queue.Queue()
        self.running = False
        self.renderer = None

    def start(self):
        # Connect to the drone
        assert self.drone.connect(retry=3), "Failed to connect to the drone"

        # Removed set_camera_mode to test if it's causing the issue
        # print("Setting camera mode to recording...")
        # self.drone(set_camera_mode(cam_id=0, value="recording")).wait()

        # Ensure RTSP Stream Works
        if DRONE_RTSP_PORT:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"
            print(f"Using RTSP port: {DRONE_RTSP_PORT}")

        # Save Streaming Video
        self.drone.streaming.set_output_files(
            video=os.path.join(self.tempd, "streaming.mp4"),
            metadata=os.path.join(self.tempd, "streaming_metadata.json"),
        )

        # Set Callbacks
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )

        print("Starting streaming...")
        assert self.drone.streaming.start(), "Failed to start streaming"

        # Initialize the renderer with error handling
        try:
            self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
            print("PdrawRenderer initialized successfully!")
        except Exception as e:
            print(f"Failed to initialize PdrawRenderer: {e}")
            self.renderer = None

        self.running = True
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.processing_thread.start()

    def stop(self):
        print("Stopping streaming...")

        # Stop the frame processing thread
        self.running = False
        self.processing_thread.join()

        # Stop the renderer if it exists
        if self.renderer is not None:
            print("Stopping renderer...")
            self.renderer.stop()
            self.renderer = None  # Ensure it's properly cleared

        # Clear the frame queue
        while not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get_nowait()
                frame.unref()
            except queue.Empty:
                break

        # Stop the video stream
        print("Stopping video stream...")
        self.drone.streaming.stop()

        # Wait for the drone to finalize the video file
        print("Waiting for video file to finalize...")
        time.sleep(5)  # Increased delay to ensure video file is finalized

        # Land the drone
        print("Landing drone...")
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("Drone landed successfully.")

        # Disconnect from the drone
        print("Disconnecting drone...")
        self.drone.disconnect()
        print("Drone disconnected.")

    def yuv_frame_cb(self, yuv_frame):
        print("Received a YUV frame!")  # Debug to confirm frames are being received
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def yuv_frame_processing(self):
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            yuv_frame.unref()

    def flush_cb(self, stream):
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        print("Streaming started successfully!")

    def end_cb(self):
        print("Streaming stopped.")

    def lock_camera(self):
        """Locks the camera orientation to prevent unwanted rotation."""
        self.drone(set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="absolute",
            yaw=0.0,
            pitch_frame_of_reference="absolute",
            pitch=0.0,
            roll_frame_of_reference="absolute",
            roll=0.0
        )).wait()

    def fly(self):
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10))
        ).wait()

        print("Setting stability settings...")
        self.drone(MaxTilt(5.0)).wait()
        self.drone(MaxRotationSpeed(10.0)).wait()
        self.lock_camera()

        print("Moving 4m ahead...")
        self.drone(moveBy(4, 0, 0, 0, _timeout=20)).wait().success()
        self.lock_camera()

        print("Moving 3m right...")
        self.drone(moveBy(0, 3, 0, 0, _timeout=20)).wait().success()
        self.lock_camera()

        print("Turning 90 degrees...")
        self.drone(moveBy(0, 0, 0, 1.57, _timeout=10)).wait().success()
        self.drone(moveBy(0, 0, 0, 1.57, _timeout=10)).wait().success()

        print("Landing...")
        self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
        print("Landed successfully!")

    def replay_video(self):
        mp4_filepath = os.path.join(self.tempd, "streaming.mp4")

        # Check if video file is valid
        if not os.path.exists(mp4_filepath):
            print(f"Video file not found at {mp4_filepath}!")
            return
        if os.path.getsize(mp4_filepath) == 0:
            print("Video file is empty! The recording may have failed.")
            return

        # Check if VLC is installed
        if shutil.which("vlc") is None:
            print("VLC is not installed. Please install VLC or use another player.")
            return

        print("Playing recorded video...")
        try:
            subprocess.run(shlex.split(f"vlc --play-and-exit {mp4_filepath}"), check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to play video: {e}")

def test_streaming():
    streaming_example = StreamingExample()
    streaming_example.start()
    streaming_example.fly()
    streaming_example.stop()

    print("Checking recorded video...")
    streaming_example.replay_video()

if __name__ == "__main__":
    test_streaming()