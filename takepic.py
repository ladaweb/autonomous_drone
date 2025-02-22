import olympe
import requests
import json
import os
import tempfile
from olympe.messages.camera2 import Command, Event
from logging import getLogger
from xml.etree import ElementTree as ET

# Logging setup
olympe.log.update_config({
    "loggers": {
        "olympe": {"level": "INFO"},
        "photo_example": {"level": "INFO", "handlers": ["console"]},
    }
})
logger = getLogger("photo_example")

# Drone IP and API endpoint
DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
MEDIA_API_URL = f"http://{DRONE_IP}/api/v1/media/medias"

# Tags of interest in metadata
XMP_TAGS_OF_INTEREST = (
    "CameraRollDegree",
    "CameraPitchDegree",
    "CameraYawDegree",
    "CaptureTsUs",
    "GPSLatitude",
    "GPSLongitude",
    "GPSAltitude",
)

def setup_photo_burst_mode(drone):
    drone(
        Command.Configure(
            camera_id=0,
            config={
                "camera_mode": "photo",
                "photo_mode": "single",
                "photo_file_format": "jpeg",
                "photo_burst_value": "10_over_1s",
                "photo_dynamic_range": "standard",
                "photo_resolution": "12_mega_pixels",
                "photo_format": "rectilinear",
                "photo_signature": "none",
            },
            _timeout=3.0,
        )
    ).wait()

def take_photo_burst(drone):
    drone(Event.Photo(type="stop", stop_reason="capture_done", _timeout=3.0) &
          Command.StartPhoto(camera_id=0)).wait()
    logger.info("Photo burst captured!")

def list_available_medias():
    try:
        response = requests.get(MEDIA_API_URL, timeout=10)
        if response.status_code == 200:
            media_list = response.json()
            logger.info("Available Media on Drone:")
            print(json.dumps(media_list, indent=4))
        else:
            logger.error(f"Failed to list media: {response.status_code} - {response.reason}")
    except requests.exceptions.RequestException as e:
        logger.error(f"Error accessing the media API: {e}")

def download_and_parse_metadata(drone, media_dir):
    drone.media.download_dir = tempfile.mkdtemp(prefix="olympe_media_")
    for media in os.listdir(media_dir):
        media_path = os.path.join(media_dir, media)
        logger.info(f"Processing media file: {media_path}")
        with open(media_path, "rb") as image_file:
            image_data = image_file.read()
            start = image_data.find(b"<x:xmpmeta")
            end = image_data.find(b"</x:xmpmeta>") + 12
            if start == -1 or end == -1:
                logger.warning(f"No metadata found in {media}")
                continue
            xmp_data = ET.fromstring(image_data[start:end])
            for item in xmp_data[0][0]:
                tag = item.tag.split("}")[-1]
                if tag in XMP_TAGS_OF_INTEREST:
                    logger.info(f"{tag}: {item.text}")

def main():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    try:
        setup_photo_burst_mode(drone)
        take_photo_burst(drone)
        list_available_medias()

        # Download and parse metadata
        media_dir = drone.media.download_dir
        if media_dir:
            download_and_parse_metadata(drone, media_dir)
        else:
            logger.error("No media directory found for downloads.")

    finally:
        drone.disconnect()

if __name__ == "__main__":
    main()
