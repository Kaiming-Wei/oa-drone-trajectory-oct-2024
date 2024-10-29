"""Data models for the camera and user specification."""
from dataclasses import dataclass

@dataclass
class Camera:
    """
    Data model for a simple pinhole camera.
    
    References: 
    - https://github.com/colmap/colmap/blob/3f75f71310fdec803ab06be84a16cee5032d8e0d/src/colmap/sensor/models.h#L220
    - https://en.wikipedia.org/wiki/Pinhole_camera_model
    """

    # We want to model the following camera parameters in Python:
    # - focal length along x axis (in pixels)
    # - focal length along y axis (in pixels)
    # - optical center of the image along the x axis (in pixels)
    # - optical center of the image along the y axis (in pixels)
    # - Size of the sensor along the x axis (in mm)
    # - Size of the sensor along the y axis (in mm)
    # - Number of pixels in the image along the x axis
    # - Number of pixels in the image along the y axis

    fx : float
    fy : float
    cx : float
    cy : float
    sensor_size_x_mm : float
    sensor_size_y_mm : float
    image_size_x_px : int
    image_size_y_px : int


@dataclass
class DatasetSpec:
    """
    Data model for specifications of an image dataset.
    """

    # - Overlap: the ratio (in 0 to 1) of scene shared between two consecutive images.
    # - Sidelap: the ratio (in 0 to 1) of scene shared between two images in adjacent rows.
    # - Height: the height of the scan above the ground (in meters).
    # - Scan_dimension_x: the horizontal size of the rectangle to be scanned
    # - Scan_dimension_y: the vertical size of the rectangle to be scanned
    # - exposure_time_ms: the exposure time for each image (in milliseconds).
    overlap : float
    sidelap : float
    height : float
    scan_dimension_x : float
    scan_dimension_y : float
    exposure_time_ms : float


@dataclass
class Waypoint:
    """
    Waypoints are positions where the drone should fly to and capture a photo.
    """
    pass