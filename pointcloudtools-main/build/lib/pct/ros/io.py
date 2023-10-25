from typing import List
import pathlib

import numpy as np
import open3d as o3d
import rosbag

from pct.utils import (
    swap_points_axes,
    color_ply_by_height,
    filter_point_cloud,
)
from pct._settings import (
    LASER_TOPIC,
    IMU_TOPIC,
    GPS_TOPIC,
)
from pct._processing_state import ProcessState
from pct._pcd_extraction import (
    process_laser_msg,
    process_imu_msg,
    process_gps_msg,
)


def xtract_pcd_from_bags(
    bags: List[pathlib.Path],
) -> o3d.geometry.PointCloud:
    """
    Bags should come in sorted order.\n
    Cloud from bags loads fully in RAM.
    """

    TOPIC_TO_HANDLER = {
        LASER_TOPIC: process_laser_msg,
        IMU_TOPIC: process_imu_msg,
        GPS_TOPIC: process_gps_msg,
    }

    # Get data from bags
    state: ProcessState = ProcessState.empty()
    for file in bags:
        bag = rosbag.Bag(file)
        for topic, msg, _ in \
            bag.read_messages(topics=TOPIC_TO_HANDLER.keys()):
            TOPIC_TO_HANDLER[topic](msg, state)
    
    # TODO check if poinst in state is empty, raise error
    if len(state.laser.point_cloud_arr) == 0:
        print(f"Point cloud arr length: {len(state.laser.point_cloud_arr)}")
        raise RuntimeError("No points were extracted from bags")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(state.laser.point_cloud_arr)
    return pcd


def create_point_cloud_from_bags(
    bags: List[pathlib.Path],
    in_dir: pathlib.Path,
    with_name: str,
    **kwargs,
) -> pathlib.Path:
    """
    Bags should come in sorted order.\n
    `kwargs: downsmpl_vox, min_filter`
    """

    pcd = xtract_pcd_from_bags(bags=bags)
    pcd = filter_point_cloud(pcd)
    pts = np.asarray(pcd.points)
    # TODO mb also save without swapping axes
    pts = swap_points_axes(pts, ax="y", with_ax="z")
    pcd.points = o3d.utility.Vector3dVector(pts)
    cloudpath = (in_dir / with_name).with_suffix(".ply")
    o3d.io.write_point_cloud(str(cloudpath), pcd)
    pcd = color_ply_by_height(cloud=cloudpath, height_axis="z", in_place=True)
    return pcd
