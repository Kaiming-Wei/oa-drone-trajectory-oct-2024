"""Utility functions for the camera model.
"""
import numpy as np

from src.data_model import Camera

def compute_focal_length_in_mm(camera: Camera) -> np.ndarray:
    """Computes the focal length in mm for the given camera

    Args:
        camera (Camera): the camera model.

    Returns:
        np.ndarray: [fx, fy] in mm.
    """
    # Note(Ayush): Solution provided by project leader.
    pixel_to_mm_x = camera.sensor_size_x_mm / camera.image_size_x_px
    pixel_to_mm_y = camera.sensor_size_y_mm / camera.image_size_y_px

    return np.array([camera.fx * pixel_to_mm_x, camera.fy * pixel_to_mm_y])

def project_world_point_to_image(camera: Camera, point: np.ndarray) -> np.ndarray:
    """Project a 3D world point into the image coordinates.

    Args:
        camera (Camera): the camera model
        point (np.ndarray): the 3D world point

    Returns:
        np.ndarray: [u, v] pixel coordinates corresponding to the point.
    """
    fx = camera.fx
    fy = camera.fy
    cx = camera.cx
    cy = camera.cy

    X, Y, Z = point

    x = (X / Z) * fx
    y = (Y / Z) * fy

    u = x + cx
    v = y + cy

    return np.array([u, v], dtype=np.float32)



def compute_image_footprint_on_surface(camera: Camera, distance_from_surface: float) -> np.ndarray:
    """Compute the footprint of the image captured by the camera at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).

    Returns:
        np.ndarray: [footprint_x, footprint_y] in meters.
    """
    fx = camera.fx  # focal width
    fy = camera.fy  # focal length
    image_width = camera.image_size_x_px
    image_length = camera.image_size_y_px 

    footprint_width = (distance_from_surface / fx) * image_width
    footprint_length = (distance_from_surface / fy) * image_length

    return np.array([footprint_width, footprint_length], dtype=np.float32)

def compute_ground_sampling_distance(camera: Camera, distance_from_surface: float) -> float:
    """Compute the ground sampling distance (GSD) at a given distance from the surface.

    Args:
        camera (Camera): the camera model.
        distance_from_surface (float): distance from the surface (in m).
    
    Returns:
        float: the GSD in meters (smaller among x and y directions).
    """
    # GSD = footprint / number of pixels
    footprint_x,footprint_y = compute_image_footprint_on_surface(camera, distance_from_surface)
    GSD_x = footprint_x / camera.image_size_x_px
    GSD_y = footprint_y / camera.image_size_y_px

    return min(GSD_x,GSD_y)


def reproject_image_point_to_world(camera: Camera, pixel_point: np.ndarray, depth: float) -> np.ndarray:
    """
    Reproject a 2D pixel location back to a 3D world point given the camera model and depth.

    Args:
        camera (Camera): the camera model.
        pixel_point (np.ndarray): the (x, y) coordinates in the image plane.
        depth (float): the depth (distance from the camera to the point) in meters.

    Returns:
        np.ndarray: the corresponding 3D world coordinates [X, Y, Z].
    """
    # Extract pixel coordinates
    u, v = pixel_point

    # Reproject to normalized camera coordinates
    x = (u - camera.cx) * depth / camera.fx
    y = (v - camera.cy) * depth / camera.fy
    z = depth  # The depth corresponds to the Z-coordinate in world space

    # Return the 3D coordinates
    return np.array([x, y, z], dtype=int)
