import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_focal_length_in_mm, compute_image_footprint_on_surface, compute_ground_sampling_distance


def compute_distance_between_images(camera: Camera, dataset_spec: DatasetSpec, camera_angle : float = 0) -> np.ndarray:
    """Compute the distance between images in the horizontal and vertical directions for specified overlap and sidelap.

    Args:
        by default this is Nadir view, camera_angle = 0
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        float: The distance between images in the horizontal direction.
        float: The distance between images in the vertical direction.
    """


    """
    If the camera is angled (non-nadir view), 
    the footprint of the image on the ground will be skewed, 
    And we need to account for this tilt when calculating the distances.
    An angled view will change the effective coverage area of each image,
    meaning we need to adjust the footprint width and height based on the tilt angle.
    The effective height along the tilted line of sight is calculated by: effective_height = height / cos(theta)
    """
    height = dataset_spec.height
    # footprint_x, footprint_y = compute_image_footprint_on_surface(camera, height)
    # print(footprint_x, footprint_y)
    # check if the camera is non-nadir
    if camera_angle != 0:
        camera_angle = np.radians(camera_angle)
        height /= np.cos(camera_angle)

    footprint_x, footprint_y = compute_image_footprint_on_surface(camera, height)

    overlap = dataset_spec.overlap
    sidelap = dataset_spec.sidelap
    distance_x = (1-overlap)*footprint_x
    distance_y = (1-sidelap)*footprint_y

    return np.array([distance_x, distance_y], dtype=np.float32)

def compute_distance_between_images_method2(camera: Camera, dataset_spec: DatasetSpec, camera_angle : float = 0) -> np.ndarray:
    # Convert focal length into mm
    fx, fy = compute_focal_length_in_mm(camera)

    # Calculate FOV in radians for x and y axes
    fov_x = 2 * np.arctan(camera.sensor_size_x_mm / (2 * fx))
    fov_y = 2 * np.arctan(camera.sensor_size_y_mm / (2 * fy))

    # Calculate footprint size
    height = dataset_spec.height
    camera_angle = np.radians(camera_angle)

    top = height * np.tan(camera_angle + fov_y / 2)
    bottom = height * np.tan(camera_angle - fov_y / 2)
    right = height * np.tan(camera_angle + fov_x / 2)
    left = height * np.tan(camera_angle - fov_x / 2)

    footprint_x = right - left
    footprint_y = top - bottom

    overlap = dataset_spec.overlap
    sidelap = dataset_spec.sidelap
    distance_x = (1-overlap)*footprint_x
    distance_y = (1-sidelap)*footprint_y

    return np.array([distance_x, distance_y], dtype=np.float32)

def compute_speed_during_photo_capture(camera: Camera, dataset_spec: DatasetSpec, allowed_movement_px: float = 1) -> float:
    """Compute the speed of drone during an active photo capture to prevent more than 1px of motion blur.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.
        allowed_movement_px (float, optional): The maximum allowed movement in pixels. Defaults to 1 px.

    Returns:
        float: The speed at which the drone should move during photo capture.
    """
    # Calculate Ground Sampling Distance (GSD) in meters/pixel
    height = dataset_spec.height
    gsd = compute_ground_sampling_distance(camera, height)

    allowed_movement_meters = allowed_movement_px * gsd

    # Calculate speed using camera's exposure time (in seconds)
    exposure_time = dataset_spec.exposure_time_ms / 1000  # Ensure exposure time is provided in seconds
    max_speed = allowed_movement_meters / exposure_time

    return max_speed


def generate_photo_plan_on_grid(camera: Camera, dataset_spec: DatasetSpec) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        List[Waypoint]: scan plan as a list of waypoints.

    """
    raise NotImplementedError()
