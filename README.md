# Drone Simulation Environment

This file demonstrates how to add a drone using Sphinx and run a simulated environment with Parrot-UE4.

## Prerequisites

- **Sphinx**: Ensure Sphinx is installed and configured.
- **Drone Firmware**: Accessible via the provided URL.
- **Parrot-UE4**: Installed and available in your system’s PATH.
- **Configuration File**: Verify that the file `/home/aims-lab/code/parrot-olympe/research/drone/src/resources/config.yaml` exists and is correctly configured.

## How to Run

### 1. Add the Drone

To register the drone with Sphinx and specify its firmware, run the following command:

```bash
sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::firmware="https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip"
```

### 2. After the drone is added, start the simulation environment by executing:

```bash
parrot-ue4-empty -config-file=/home/aims-lab/code/parrot-olympe/research/drone/src/resources/config.yaml
```
