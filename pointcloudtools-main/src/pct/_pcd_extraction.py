import math
from typing import Any, Tuple

import numpy as np

# TODO fix this somehow
np.float = np.float64

import ros_numpy as rnp

from pct._processing_state import ProcessState


def process_laser_msg(msg: Any, state: ProcessState) -> None:
    """
    ---
    Processes `geometry_msgs.PointCloud2` message
    ---
    Then fields from `fields` are extracted and processed.\n
    Message with used fields:\n
    - `x (float64)`
    - `y (float64)`
    - `z (float64)`
    - `intensity`
    """

    # Search for GPS first
    if not state.gps.captured:
        return

    # Do not process laser msg if standing (passed dist less than 1 cm)
    if state.gps.passed_dist_m < 0.01:  # TODO may vary
        return

    # Do not process msg if moving backwards
    if state.gps.passed_dist_m - state.gps.prev_passed_dist_m < 0:
        return

    # Get points with intensity
    state.laser.cur_data = \
        rnp.point_cloud2.pointcloud2_to_array(msg, squeeze=True)
    
    # Return if msg is empty
    if len(state.laser.cur_data) == 0:
        return

    # Get xyz points from PointCloud2 array
    state.laser.cur_arr = get_xyz_from_arr(state.laser.cur_data)

    # Correct xyz points with IMU
    state.laser.cur_arr = state.imu.cur_rot_mat.dot(state.laser.cur_arr.T).T

    # Rotate point cloud becasuse of upside-down lidar orientation TODO need this?
    state.laser.cur_arr = rotate_points_by_180_z(state.laser.cur_arr)

    # Shift points position by change in distance
    # state.laser.cur_arr += [0, state.gps.alt_diff_m, state.gps.passed_dist_m]
    state.laser.cur_arr += [
        0,
        0 if state.gps.alt_diff_m is None else state.gps.alt_diff_m,
        0 if state.gps.passed_dist_m is None else state.gps.passed_dist_m]

    # Append new points set to main cloud
    state.laser.point_cloud_arr.extend(state.laser.cur_arr)


def process_imu_msg(msg: Any, state: ProcessState) -> None:
    """
    ---
    Processes `geometry_msgs.Quaternion` message
    ---
    Message with 4 fields:\n
    - `x (float64)`
    - `y (float64)`
    - `z (float64)`
    - `w (float64)`
    """

    # Search for GPS first
    if not state.gps.captured:
        return

    # Get drone orientation (geometry_msgs.Quaternion)
    msg_quat = msg.orientation
    # Get angles from quat
    roll_x, pitch_y, yaw_z = euler_from_quaternion(
        msg_quat.x,
        msg_quat.y,
        msg_quat.z,
        msg_quat.w,
    )

    # Capture initial platform angles
    # and correct further position based on them
    if state.imu.start_vals is None:
        state.imu.start_vals = (roll_x, pitch_y, yaw_z)

    # Swap roll and pitch
    # Because laser scanner is aligned horizontally
    state.imu.cur_rot_mat = euler_to_rot_mat(
        roll=pitch_y - state.imu.start_vals[1],
        pitch=roll_x - state.imu.start_vals[0],
        yaw=yaw_z - state.imu.start_vals[2],
    )


def process_gps_msg(msg: Any, state: ProcessState) -> None:
    """
    ---
    Processes `sensor_msgs.NavSatFix` message
    ---
    Message with used fields:\n
    ...
    - `latitude (float64)`
    - `longitude (float64)`
    - `altitude (float64)` \n
    ...
    """

    # Remember starting position
    if not state.gps.captured:
        state.gps.start_lat = msg.latitude
        state.gps.start_lon = msg.longitude
        state.gps.start_alt = msg.altitude

    # Remember previous passed dist
    if state.gps.passed_dist_m is not None:
        state.gps.prev_passed_dist_m = state.gps.passed_dist_m

    # Update current position
    state.gps._cur_lat = msg.latitude
    state.gps._cur_lon = msg.longitude
    state.gps._cur_alt = msg.altitude

    state.gps.captured = True


def euler_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[float, float, float]:
    """
    Converts `x`, `y`, `z`, `w` values to `roll`, `pitch`, `yaw`
    """

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


def get_xyz_from_arr(arr: np.ndarray) -> np.ndarray:
    # Removing nans
    mask = np.isfinite(arr['x']) & np.isfinite(arr['y']) & np.isfinite(arr['z'])
    arr = arr[mask]

    # TODO
    xyz = np.zeros(arr.shape + (3,), dtype=np.float64)
    xyz[..., 0] = arr["x"]
    xyz[..., 1] = arr["y"]
    xyz[..., 2] = arr["z"]

    return xyz


def euler_to_rot_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1],
    ])
    ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)],
    ])
    rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)],
    ])
    # R = RzRyRx
    rotMat = np.dot(rz_yaw, np.dot(ry_pitch, rx_roll))

    return rotMat


def rotate_points_by_180_z(points: np.ndarray) -> np.ndarray:
    rz = np.array([
        [ np.cos(3.14159), -np.sin(3.14159), 0  ],
        [ np.sin(3.14159), np.cos(3.14159) , 0  ],
        [ 0              , 0               , 1  ],
    ])

    return rz.dot(points.T).T
