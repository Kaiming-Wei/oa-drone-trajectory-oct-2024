import typing as T

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
    x_dist, y_dist = compute_distance_between_images(camera, dataset_spec)

    scan_dimension_x, scan_dimension_y = dataset_spec.scan_dimension_x, dataset_spec.scan_dimension_y


    num_images_x = int(np.ceil(scan_dimension_x / x_dist))
    num_images_y = int(np.ceil(scan_dimension_y / y_dist))

    waypoints = []
    z = dataset_spec.height
    speed = compute_speed_during_photo_capture(camera, dataset_spec)

    for j in range(num_images_y):
        row_waypoints = []
        for i in range(num_images_x):
            x = i * x_dist
            y = j * y_dist
            row_waypoints.append(Waypoint(x, y, z, speed))
        
        # Reverse row order for alternating rows to create the lawnmower pattern
        if j % 2 == 1:
            row_waypoints.reverse()
        
        waypoints.extend(row_waypoints)
    
    return waypoints


def approximate_time(camera: Camera, dataset_spec: DatasetSpec, waypoints: list, max_speed: float, a_max: float):
    imag_dist = compute_distance_between_images(camera, dataset_spec)[0]

    photo_speed = waypoints[0].speed
    time_to_next_waypoint = 0       # time take from end of one waypoint to the next begining of the waypoint

    # distance for drone to accelerate from photo speed to max speed
    dist_to_max_speed = (max_speed - photo_speed) ** 2 / (2 * a_max)

    achievable_speed = max_speed


    # Triangle
    if imag_dist <= 2 * dist_to_max_speed:
        distance = imag_dist / 2

        # find the speed at the half way of the distance, we need another half distance to deccelerate
        # I used one of the kinematic equation: v_f ** 2 = v_i ** 2 + 2*a*d
        # to find highest speed the drone can reach
        achievable_speed = np.sqrt(photo_speed**2 + 2 * a_max * distance)
        time_to_next_waypoint = 2 * (achievable_speed - photo_speed) / a_max

    else:   # Trapezoid
        accelerate_time = (max_speed - photo_speed) / a_max     # time for drone to accelerate to it's max speed
        dist_at_max_speed = imag_dist - 2 * dist_to_max_speed   # distance drone fly at the max speed
        time_at_max_speed = dist_at_max_speed / max_speed       # time maintain in the max speed

        time_to_next_waypoint = 2 * accelerate_time + time_at_max_speed
    
    # Time to take a single photo in seconds
    time_photo = dataset_spec.exposure_time_ms / 1000.0

    # Total time to capture all photos and move between waypoints
    n = len(waypoints)
    total_time = n * time_photo + (n - 1) * time_to_next_waypoint


    return total_time, achievable_speed  