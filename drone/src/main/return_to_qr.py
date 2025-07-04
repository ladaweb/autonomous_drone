import olympe
import os
import time
import numpy as np
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")

# Hardcoded pose (from earlier detection)
saved_pose = {
    "yaw_deg": -0.14,
    "altitude_m": 1.22
}


def return_to_saved_qr():
    drone = olympe.Drone(DRONE_IP)
    if not drone.connect():
        print("❌ Failed to connect to drone")
        return

    print("🛫 Taking off...")
    assert drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=10)).wait().success()
    time.sleep(2)

    # Go up to the saved altitude (z is negative up)
    altitude = saved_pose["altitude_m"]
    print(f"📏 Ascending to {altitude:.2f} m...")
    assert drone(
        moveBy(0.0, 0.0, -altitude, 0.0) >> FlyingStateChanged(state="hovering", _timeout=10)
    ).wait().success()

    # Rotate to saved yaw
    yaw_rad = np.deg2rad(saved_pose["yaw_deg"])
    print(f"↪️ Rotating to yaw {saved_pose['yaw_deg']:.2f}°...")
    assert drone(
        moveBy(0.0, 0.0, 0.0, yaw_rad) >> FlyingStateChanged(state="hovering", _timeout=10)
    ).wait().success()

    print("✅ Arrived at saved position. Hovering...")
    time.sleep(5)

    print("🛬 Landing...")
    assert drone(Landing() >> FlyingStateChanged(state="landing", _timeout=10)).wait().success()

    drone.disconnect()
    print("🔌 Disconnected. Return complete.")


if __name__ == "__main__":
    return_to_saved_qr()
