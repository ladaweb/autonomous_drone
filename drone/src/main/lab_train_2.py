import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.battery import capacity
import time
import threading
import cv2
import keyboard
import os
import numpy as np

# Drone connection settings
DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT", "554")
RTSP_URL = f"rtsp://{DRONE_IP}:{DRONE_RTSP_PORT}/live"

# Global control flags
drone_should_land = False
drone_should_hover = False

# Battery globals
battery_initial = None  # mAh at start
battery_max = None      # max battery capacity in mAh

# QR code tracking
qr_codes_detected = []  # List to store (data, qr_time, scan_number, detection_to_decode_time)

def log_message(message, start_time=None):
    """Log message with timestamp to console and file, optionally including duration."""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    if start_time is not None:
        duration = time.time() - start_time
        log_entry += f" (Duration: {duration:.2f} seconds)"
    print(log_entry)
    with open("drone_mission_log.txt", "a") as f:
        f.write(f"{log_entry}\n")

def battery_callback(batt, scheduler):
    """Callback to track current battery state."""
    global battery_max, battery_initial
    try:
        if battery_max is None:
            battery_max = batt["full_charge"]
            log_message(f"Initial max battery capacity set: {battery_max:.0f} mAh")
        if battery_initial is None:
            battery_initial = batt["remaining"]
            log_message(f"Initial battery level set: {battery_initial:.0f} mAh")
        return batt["remaining"]
    except Exception as e:
        log_message(f"Error in battery_callback: {str(e)}")
        return None

def get_battery_status(drone, retries=3, delay=1):
    """Get current battery mAh and percentage with retry logic."""
    for _ in range(retries):
        batt_state = drone.get_state(capacity)
        if batt_state:
            remaining_mAh = batt_state["remaining"]
            full_charge_mAh = batt_state["full_charge"]
            percentage = (remaining_mAh / full_charge_mAh) * 100
            log_message(f"Battery status: {remaining_mAh:.0f} mAh ({percentage:.1f}%)")
            return remaining_mAh, percentage
        log_message("Retrying battery state retrieval...")
        time.sleep(delay)
    log_message("Unable to retrieve battery state after retries")
    return None, None

def monitor_battery(drone):
    """Continuously monitor battery status in a separate thread."""
    global drone_should_land
    while not drone_should_land:
        remaining_mAh, percentage = get_battery_status(drone)
        if percentage is not None and percentage < 20:
            log_message("Battery critically low (<20%). Landing immediately.")
            drone_should_land = True
        time.sleep(2)

def keyboard_listener():
    """Listen for keyboard inputs to control drone."""
    global drone_should_land, drone_should_hover
    while True:
        if keyboard.is_pressed("q"):
            log_message("Emergency land triggered (q pressed)")
            drone_should_land = True
            drone_should_hover = False  # Reset hover to allow immediate landing
            break
        elif keyboard.is_pressed("s"):
            log_message("Hover triggered (s pressed)")
            drone_should_hover = True
            time.sleep(0.5)  # Debounce to avoid multiple triggers

def safe_move(drone, dx, dy, dz, dpsi):
    """Execute a safe movement with checks for landing/hover and battery."""
    global drone_should_land, drone_should_hover
    # Check battery before moving
    remaining_mAh, percentage = get_battery_status(drone)
    if percentage is not None and percentage < 20:
        log_message("Battery critically low (<20%). Landing immediately.")
        drone_should_land = True

    if drone_should_land:
        log_message("Land requested. Skipping movement.")
        return False  # Return False to indicate movement was skipped

    if drone_should_hover:
        log_message("Hover requested. Holding position...")
        drone(FlyingStateChanged(state="hovering", _timeout=5)).wait()
        while drone_should_hover and not drone_should_land:
            time.sleep(0.5)  # Check frequently to exit hover on land request
        return False  # Return False to indicate no movement was performed

    log_message(f"Executing moveBy: dx={dx}, dy={dy}, dz={dz}, dpsi={dpsi}")
    move = drone(
        moveBy(dx, dy, dz, dpsi)
        >> FlyingStateChanged(state="hovering", _timeout=10)
    ).wait()
    if not move.success():
        log_message("Movement failed!")
        return False
    log_message("Movement completed successfully")
    return True

def scan_qr_for_20_seconds(drone, scan_number):
    """Scan for QR codes using drone's video stream for 20 seconds."""
    global qr_codes_detected
    log_message(f"Connecting to drone video stream for QR scan {scan_number}")
    scan_start_time = time.time()
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    if not cap.isOpened():
        log_message(f"Failed to open video stream for QR scan {scan_number}", scan_start_time)
        return 0

    detector = cv2.QRCodeDetector()
    qr_found = []
    start_time = time.time()
    first_detection_time = None

    while time.time() - start_time < 20:
        if drone_should_land:
            log_message(f"QR scan {scan_number} aborted due to landing request")
            break

        ret, frame = cap.read()
        if not ret:
            log_message(f"Failed to read frame from video stream in scan {scan_number}")
            continue

        data, bbox, _ = detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            # QR code detected (bounding box found)
            if first_detection_time is None:
                first_detection_time = time.time()
                log_message(f"QR code detected (bounding box) in scan {scan_number}")
            # Draw bounding box
            bbox = np.int32(bbox)
            for i in range(len(bbox[0])):
                pt1 = tuple(bbox[0][i])
                pt2 = tuple(bbox[0][(i + 1) % len(bbox[0])])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            if data:
                # QR code decoded
                cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if data and data not in [q[0] for q in qr_found] and data not in [q[0] for q in qr_codes_detected]:
                    detection_to_decode_time = time.time() - first_detection_time if first_detection_time else 0.0
                    log_message(f"QR Code decoded in scan {scan_number}: '{data}' (Detection to decode time: {detection_to_decode_time:.3f} seconds)")
                    qr_found.append((data, time.time() - start_time))
                    qr_codes_detected.append((data, time.time() - start_time, scan_number, detection_to_decode_time))
                    break  # Stop after finding the first QR code

        # Show live drone video feed with QR overlay
        cv2.imshow("Drone QR Debug View", frame)
        if cv2.waitKey(1) & 0xFF == ord("q") or drone_should_land:
            log_message(f"QR scan {scan_number} interrupted (q pressed or landing triggered)")
            break

    cap.release()
    cv2.destroyAllWindows()
    get_battery_status(drone)
    qr_count = len(qr_found)
    if qr_count == 0 and first_detection_time is not None:
        log_message(f"QR code detected but not decoded in scan {scan_number}")
    log_message(f"Finished QR scan {scan_number}. Detected {qr_count} QR code(s)", scan_start_time)
    return qr_count

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

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = 30  # Standard frame rate

    # Initialize video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
    if not out.isOpened():
        log_message("Failed to initialize video writer for 360 scan", scan_start_time)
        cap.release()
        return None, None

    log_message(f"Recording started: {video_filename}")
    return cap, out

def main():
    """Main control loop for drone operation with battery monitoring and logging."""
    global drone_should_land, drone_should_hover, qr_codes_detected
    main_start_time = time.time()
    log_message("Starting main execution")

    print("[MAIN] Connecting to drone...")
    drone = olympe.Drone(DRONE_IP)
    if not drone.connect():
        log_message("Connection to drone failed")
        return
    log_message("Connected to drone")

    # Subscribe to battery updates
    log_message("Subscribing to battery updates")
    drone.subscribe(battery_callback, capacity(_policy="wait"))

    # Check initial battery status
    remaining_mAh, percentage = get_battery_status(drone)
    if percentage is None or percentage < 20:
        log_message("Low battery or unable to read battery (<20%). Aborting mission.")
        drone.disconnect()
        return

    log_message("Starting keyboard listener")
    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()

    log_message("Starting battery monitoring thread")
    battery_thread = threading.Thread(target=monitor_battery, args=(drone,), daemon=True)
    battery_thread.start()

    try:
        log_message("Initiating takeoff")
        takeoff = drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait()
        if not takeoff.success():
            log_message("Takeoff failed")
            drone_should_land = True
        else:
            log_message("Takeoff completed")
        if drone_should_land:
            return

        log_message("Setting max tilt to 5 (slower speed)")
        drone(MaxTilt(5)).wait()
        get_battery_status(drone)
        if drone_should_land:
            return

        # Step 1: Move up 1.5 m (in 3 steps)
        for i in range(3):
            if not safe_move(drone, 0, 0, -0.5, 0):
                if drone_should_land:
                    return
            time.sleep(0.5)
            log_message(f"Moved up 0.5 m (step {i+1}/3)")
            if drone_should_land:
                return

        # Step 2: Move forward 1.12 m
        log_message("Moving forward 1.12 m")
        if not safe_move(drone, 1.12, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        if drone_should_land:
            return

        # Step 3: Rotate 90° left (counter-clockwise)
        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Rotated 90° left")
        if drone_should_land:
            return

        # Step 4: Approach QR area
        if not safe_move(drone, 1.4, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Approached QR area")
        if drone_should_land:
            return

        # Step 5: Scan for 1st QR
        log_message("Scanning for first QR code")
        qr_count = scan_qr_for_20_seconds(drone, 1)
        log_message(f"Total QR codes detected so far: {len(qr_codes_detected)}")
        if drone_should_land:
            return

        # Step 6: Go back (rotate back and return)
        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Rotated back (step 1/2)")
        if drone_should_land:
            return

        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Rotated back (step 2/2)")
        if drone_should_land:
            return

        if not safe_move(drone, 1, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved back 1 m")
        if drone_should_land:
            return

        # Step 7: Turn left
        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Turned left")
        if drone_should_land:
            return

        # Step 8: Go 2 m
        if not safe_move(drone, 1, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved forward 1 m (step 1/2)")
        if drone_should_land:
            return

        if not safe_move(drone, 1.2, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved forward 1.2 m (step 2/2)")
        if drone_should_land:
            return

        # Step 9: Turn right -> approach 2nd QR
        if not safe_move(drone, 0, 0, 0, 1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Turned right for second QR")
        if drone_should_land:
            return

        # Step 10: Scan for 2nd QR
        log_message("Scanning for second QR code")
        qr_count = scan_qr_for_20_seconds(drone, 2)
        log_message(f"Total QR codes detected so far: {len(qr_codes_detected)}")
        if drone_should_land:
            return

        # Step 11: Turn left, 3 m forward -> approach 3rd QR
        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Turned left for third QR")
        if drone_should_land:
            return

        if not safe_move(drone, 1.5, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved forward 1.5 m (step 1/3)")
        if drone_should_land:
            return

        if not safe_move(drone, 1.5, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved forward 1.5 m (step 2/3)")
        if drone_should_land:
            return

        if not safe_move(drone, 1, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved forward 1 m (step 3/3)")
        if drone_should_land:
            return

        if not safe_move(drone, 0.8, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved forward 0.8 m")
        if drone_should_land:
            return

        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Turned left")
        if drone_should_land:
            return

        # Step 12: Approach 3rd QR
        if not safe_move(drone, 1.5, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Approached third QR")
        if drone_should_land:
            return

        # Step 13: Scan for 3rd QR
        log_message("Scanning for third QR code")
        qr_count = scan_qr_for_20_seconds(drone, 3)
        log_message(f"Total QR codes detected so far: {len(qr_codes_detected)}")
        if drone_should_land:
            return

        # Step 14: Go back
        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Rotated back (step 1/3)")
        if drone_should_land:
            return

        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Rotated back (step 2/3)")
        if drone_should_land:
            return

        if not safe_move(drone, 0.9, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Moved back 0.9 m")
        if drone_should_land:
            return

        if not safe_move(drone, 0, 0, 0, -1.5708):
            if drone_should_land:
                return
        time.sleep(0.5)
        log_message("Rotated back (step 3/3)")
        if drone_should_land:
            return

        # Step 15: Move forward 1.5 m and do 360 scan
        log_message("Moving forward 1.5 m for 360 scan")
        if not safe_move(drone, 1.5, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(1)
        if drone_should_land:
            return

        # Start video recording for 360 scan
        video_cap, video_out = record_360_scan(drone)
        if video_cap and video_out:
            log_message("Performing 360-degree scan")
            scan_start_time = time.time()
            for i in range(4):
                if drone_should_land:
                    log_message("360 scan aborted due to landing request")
                    break
                if not safe_move(drone, 0, 0, 0, 1.5708):  # 90 degrees
                    if drone_should_land:
                        break
                log_message(f"Rotated 90 degrees (step {i+1}/4) for 360 scan")
                # Record frames during rotation
                rotation_start = time.time()
                while time.time() - rotation_start < 2:  # 2 seconds per rotation
                    if drone_should_land:
                        log_message("360 scan interrupted due to landing request")
                        break
                    ret, frame = video_cap.read()
                    if ret:
                        video_out.write(frame)
                    get_battery_status(drone)
                if drone_should_land:
                    break

            # Stop video recording
            log_message("Stopping 360 scan recording")
            video_cap.release()
            video_out.release()
            log_message("Video saved to 360_scan.mp4", scan_start_time)
        else:
            log_message("Skipped 360 scan due to video setup failure")
        if drone_should_land:
            return

        # Step 16: Turn right to go to initial position
        if not safe_move(drone, 0, 0, 0, 1.5708):
            if drone_should_land:
                return
        time.sleep(1)
        log_message("Turned right (step 1/2)")
        if drone_should_land:
            return

        if not safe_move(drone, 0, 0, 0, 1.5708):
            if drone_should_land:
                return
        time.sleep(1)
        log_message("Turned right (step 2/2)")
        if drone_should_land:
            return

        # Step 17: Move back 1.5 m
        log_message("Returning back 1.5 m to original position")
        if not safe_move(drone, 1.5, 0, 0, 0):
            if drone_should_land:
                return
        time.sleep(1)
        if drone_should_land:
            return

        # Log total QR codes detected
        log_message(f"Total QR codes detected in mission: {len(qr_codes_detected)}")
        if qr_codes_detected:
            log_message("QR codes detected:")
            for qr_data, qr_time, qr_scan, decode_time in qr_codes_detected:
                log_message(f"  Scan {qr_scan}: '{qr_data}' at {qr_time:.2f} seconds (Detection to decode time: {decode_time:.3f} seconds)")

    finally:
        # Step 18: Land and disconnect
        if drone.get_state(FlyingStateChanged)["state"] != "landed":
            log_message("Initiating landing")
            drone(Landing() >> FlyingStateChanged(state="landed", _timeout=5)).wait()
            log_message("Drone landed")
            get_battery_status(drone)
        log_message("Disconnecting from drone")
        drone.disconnect()
        log_message("Main execution completed", main_start_time)

if __name__ == "__main__":
    main()