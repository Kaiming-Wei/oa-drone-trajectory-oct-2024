import typing as T
import math

import numpy as np

from src.data_model import Camera, DatasetSpec, Waypoint
from src.camera_utils import compute_image_footprint_on_surface, compute_ground_sampling_distance


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
    footprint_x, footprint_y = compute_image_footprint_on_surface(camera, dataset_spec.height)
    overlap = dataset_spec.overlap
    sidelap = dataset_spec.sidelap
    distance_x = (1-overlap)*footprint_x
    distance_y = (1-sidelap)*footprint_y

    # check if the camera is Non-Nadir view
    if camera_angle != 0:
        camera_angle = np.radians(camera_angle)
        distance_x /= np.cos(camera_angle)
        distance_y /= np.cos(camera_angle)


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
    raise NotImplementedError()


def generate_photo_plan_on_grid(camera: Camera, dataset_spec: DatasetSpec) -> T.List[Waypoint]:
    """Generate the complete photo plan as a list of waypoints in a lawn-mower pattern.

    Args:
        camera (Camera): Camera model used for image capture.
        dataset_spec (DatasetSpec): user specification for the dataset.

    Returns:
        List[Waypoint]: scan plan as a list of waypoints.

    """
    raise NotImplementedError()
