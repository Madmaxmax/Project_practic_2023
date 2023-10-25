
import os
import pandas as pd
import pathlib
import functools
from typing import Union, Tuple, Dict

import numpy as np
import open3d as o3d


def convert_ply_to_xyz(
    cloud: pathlib.Path,
) -> pathlib.Path:
    """
    Converts `.ply` cloud to `.xyz` cloud.\n
    Returns path to converted cloud.
    """
    # TODO add all properties from ply to xyz

    pcd = o3d.io.read_point_cloud(str(cloud), format="ply")
    directory = cloud.parent
    filename = cloud.stem
    xyz_cloud = (directory / filename).with_suffix(".xyz")
    o3d.io.write_point_cloud(str(xyz_cloud), pcd)
    return xyz_cloud


def convert_xyz_to_ply(
    cloud: pathlib.Path,
) -> pathlib.Path:
    raise NotImplementedError()


def convert_ply_to_las() -> ...:
    raise NotImplementedError()


def filter_point_cloud(
    pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """
    Donwsamples point cloud object and removes points radius otliers
    """

    DOWNSAMPLE_VOXEL_SZ = 0.5
    pcd = pcd.voxel_down_sample(voxel_size=DOWNSAMPLE_VOXEL_SZ)

    NEIGHBORS = 10
    RADIUS = 4.5
    pcd, _ = pcd.remove_radius_outlier(nb_points=NEIGHBORS, radius=RADIUS)
    return pcd


def swap_points_axes(
    pts: np.ndarray,
    ax: Union[int, str],
    with_ax: Union[int, str],
) -> np.ndarray:
    """Returns swapped points with swapped axes"""

    ax_to_idx = {
        "x": 0,
        "y": 1,
        "z": 2,
        0: 0,
        1: 1,
        2: 2,
    }
    ax_1 = ax_to_idx[ax]
    ax_2 = ax_to_idx[with_ax]

    pts[:, [ax_1, ax_2]] = pts[:, [ax_2, ax_1]]
    return pts


def swap_point_cloud_file_axes(
    cloud: pathlib.Path,
    ax: Union[int, str],
    with_ax: Union[int, str],
    in_place: bool = False,
) -> pathlib.Path:
    """
    Reads point cloud, swaps two axes between each other, 
    saves new point cloud with swapped axes.\n
    Returns path to new point cloud.\n
    `in_place` will overwrite the given file.
    """

    idx_to_ax = {
        0: "x",
        1: "y",
        2: "z",
        "x": "x",
        "y": "y",
        "z": "z",
    }
    ax_to_idx = {
        "x": 0,
        "y": 1,
        "z": 2,
        0: 0,
        1: 1,
        2: 2,
    }
    ax_1 = ax_to_idx[ax]
    ax_2 = ax_to_idx[with_ax]

    pcd = o3d.io.read_point_cloud(str(cloud))

    pts = np.asarray(pcd.points)
    pts = swap_points_axes(pts, ax=ax_1, with_ax=ax_2)
    pcd.points = o3d.utility.Vector3dVector(pts)

    if in_place:
        swapped_cloud_path = cloud
    else:
        ext = cloud.suffix
        swapped_cloud_path = \
            f"{os.path.splitext(cloud)[0]}_swapped_ax_{idx_to_ax[ax]}_with_{idx_to_ax[with_ax]}{ext}"

    o3d.io.write_point_cloud(str(swapped_cloud_path), pcd)
    return pathlib.Path(swapped_cloud_path)


def rotate_pts_by_axis(
    pts: np.ndarray,
    ax: Union[int, str],
    rad: float,
) -> np.ndarray:
    """
    Rotates `pts` points along `ax` axis by `rad` radians.\n
    Returns rotated points.
    """

    if ax == "x":
        ax = 0
    if ax == "y":
        ax = 1
    if ax == "z":
        ax = 2

    ax_map = {
        0: functools.partial(_get_rot_mat, rad, 0, 0), 
        1: functools.partial(_get_rot_mat, 0, rad, 0),
        2: functools.partial(_get_rot_mat, 0, 0, rad),
    }
    rot_mat = ax_map[ax]()
    rotated_pts = rot_mat.dot(pts.T).T
    return rotated_pts


def rotate_point_cloud_by_axis(
    cloud: pathlib.Path,
    ax: Union[int, str],
    rad: float,
    in_place: bool = False,
) -> pathlib.Path:
    """
    Reads, rotates point cloud along `ax` axis by `rad` radians 
    and saves new rotated point cloud.\n
    `in_place` will overwrite the given file.\n
    Returns path to rotated point cloud.\n
    `+` sign rotation units -> clockwise rotation.
    """

    pcd = o3d.io.read_point_cloud(str(cloud))

    pts = np.asarray(pcd.points)
    rotated_pts = rotate_pts_by_axis(pts, ax=ax, rad=rad)
    pcd.points = o3d.utility.Vector3dVector(rotated_pts)

    if in_place:
        rotated_cloud_path = cloud
    else:
        ext = cloud.suffix
        rotated_cloud_path = f"{os.path.splitext(cloud)[0]}_rotated_{ax}_{rad}{ext}"

    o3d.io.write_point_cloud(str(rotated_cloud_path), pcd)
    return pathlib.Path(rotated_cloud_path)


# TODO remake
def _class_xyz_ground_by_height(cloud: pathlib.Path) -> pathlib.Path:
    """
    Classifies `.xyz` cloud ground points. Rough classification within 1m from lowest point.\n
    Returns path to new classified cloud.
    """

    outfile = pathlib.Path(f"{cloud}_out")
    df = pd.read_csv(cloud, sep=" ", names=["x", "y", "z"])
    zcol = df.iloc[:, 2]
    border = zcol.min() + 1
    ground_pts = []

    for zval in zcol:
        if zval <= border:
            ground_pts.append("GROUND")
        else:
            ground_pts.append("")

    df["class"] = ground_pts
    df.to_csv(outfile, index=False, sep=" ", lineterminator="\n")
    return outfile


def _div_points_into_hozrizon_segments(
    pts: np.ndarray,
    alt_ax: int = 2,
) -> Tuple[np.ndarray]:
    """Altitude in Z axis"""

    # TODO make number of slices arbitrary

    x_col, y_col = pts[:, 0], pts[:, 1]

    n_y = 50  # Step factor in y axis
    n_x = 30  # Step factor in x axis

    max_x_pt_val, min_x_pt_val = x_col.max(), x_col.min()
    max_y_pt_val, min_y_pt_val = y_col.max(), y_col.min()

    xstep = (max_x_pt_val - min_x_pt_val) / n_x  # Step left/right
    ystep = (max_y_pt_val - min_y_pt_val) / n_y  # Step forward

    # Empty points containers to fill
    pts_lowest_to_1_m = np.empty((0, 3))
    pts_1_to_3_m = np.empty((0, 3))
    pts_3_to_4_m = np.empty((0, 3))
    pts_4_to_6_m = np.empty((0, 3))
    pts_6_to_10_m = np.empty((0, 3))
    pts_10_m_to_end = np.empty((0, 3))

    # TODO optimize concatenation with append, then convert to np.array

    # Traverse cloud in segments in case ground has elevations and cavities
    for y in np.arange(min_y_pt_val, max_y_pt_val, ystep):
        for x in np.arange(min_x_pt_val, max_x_pt_val, xstep):
            # Select a segment of points from cloud within borders:
            segment = pts[
                (x_col > x)  # lower x border
                & (x_col < x + xstep)  # upper x border
                & (y_col > y)  # lower y border
                & (y_col < y + ystep)  # upper y border
            ]
            if segment.size == 0:
                continue
            segment_z_col = segment[:, alt_ax]
            # Height minimum inside current segment
            loc_z_min = segment_z_col.min()

            # Points starting from lowest point to 1m
            ceil_1_m = loc_z_min + 1
            cur_seg = segment[
                (segment_z_col >= loc_z_min)
                & (segment_z_col < ceil_1_m)]
            pts_lowest_to_1_m = np.concatenate((pts_lowest_to_1_m, cur_seg))

            # Points from 1 to 3 m
            ceil_3_m = loc_z_min + 3
            cur_seg = segment[
                (segment_z_col >= ceil_1_m)
                & (segment_z_col < ceil_3_m)]
            pts_1_to_3_m = np.concatenate((pts_1_to_3_m, cur_seg))

            # Points from 3 to 4 m
            ceil_4_m = loc_z_min + 4
            cur_seg = segment[
                (segment_z_col >= ceil_3_m)
                & (segment_z_col < ceil_4_m)]
            pts_3_to_4_m = np.concatenate((pts_3_to_4_m, cur_seg))

            # Points from 4 to 6 m
            ceil_6_m = loc_z_min + 6
            cur_seg = segment[
                (segment_z_col >= ceil_4_m)
                & (segment_z_col < ceil_6_m)]
            pts_4_to_6_m = np.concatenate((pts_4_to_6_m, cur_seg))

            # Points from 6 to 10 m
            ceil_10_m = loc_z_min + 10
            cur_seg = segment[
                (segment_z_col >= ceil_6_m)
                & (segment_z_col < ceil_10_m)]
            pts_6_to_10_m = np.concatenate((pts_6_to_10_m, cur_seg))

            # Points from 10 m to end
            cur_seg = segment[
                (segment_z_col >= ceil_10_m)]
            pts_10_m_to_end = np.concatenate((pts_10_m_to_end, cur_seg))

    return (
        pts_lowest_to_1_m,
        pts_1_to_3_m,
        pts_3_to_4_m,
        pts_4_to_6_m,
        pts_6_to_10_m,
        pts_10_m_to_end,
    )


def color_ply_by_height(
    cloud: pathlib.Path,
    height_axis: Union[str, int],
    in_place: bool = False,
) -> pathlib.Path:
    """
    Colors `.ply` cloud in XY segments.\n
    Returns path to colored cloud.\n
    `in_place` will overwrite the given file.
    """

    ax_m = {
        "x": 0,
        "y": 1,
        "z": 2,
        0: 0,
        1: 1,
        2: 2,
    }
    height_axis = ax_m[height_axis]

    COLORS = {
        "var_min_to_1": [255, 200, 200],  # Ground
        "1_to_3": [0, 200, 0],
        "3_to_4": [204, 0, 204],
        "4_to_6": [0, 0, 204],
        "6_to_10": [204, 102, 0],
        "10_to_var_max": [0, 191, 255],
    }
    
    pcd = o3d.io.read_point_cloud(str(cloud))
    pts = np.asarray(pcd.points)

    # Points horizontal slices
    slices = _div_points_into_hozrizon_segments(pts, alt_ax=height_axis)
    lowest_to_1_m, pts_1_to_3_m, pts_3_to_4_m, \
        pts_4_to_6_m, pts_6_to_10_m, \
        pts_10_m_to_end = slices

    colors_min_to_1_m = np.array([COLORS["var_min_to_1"]] * len(lowest_to_1_m))
    colors_1_to_3_m = np.array([COLORS["1_to_3"]] * len(pts_1_to_3_m))
    colors_3_to_4_m = np.array([COLORS["3_to_4"]] * len(pts_3_to_4_m))
    colors_4_to_6_m = np.array([COLORS["4_to_6"]] * len(pts_4_to_6_m))
    colors_6_to_10_m = np.array([COLORS["6_to_10"]] * len(pts_6_to_10_m))
    colors_10_m_to_end = np.array([COLORS["10_to_var_max"]] * len(pts_10_m_to_end))

    # Fill empty slices with empty 3d arrays
    if colors_min_to_1_m.size == 0:
        colors_min_to_1_m = np.empty((0, 3))
    if colors_1_to_3_m.size == 0:
        colors_1_to_3_m = np.empty((0, 3))
    if colors_3_to_4_m.size == 0:
        colors_3_to_4_m = np.empty((0, 3))
    if colors_4_to_6_m.size == 0:
        colors_4_to_6_m = np.empty((0, 3))
    if colors_6_to_10_m.size == 0:
        colors_6_to_10_m = np.empty((0, 3))
    if colors_10_m_to_end.size == 0:
        colors_10_m_to_end = np.empty((0, 3))

    colors_arr = np.concatenate((
        colors_min_to_1_m,
        colors_1_to_3_m,
        colors_3_to_4_m,
        colors_4_to_6_m,
        colors_6_to_10_m,
        colors_10_m_to_end,
    ), axis=0)

    pcd.points = o3d.utility.Vector3dVector(np.concatenate(slices))
    # Colors are float64 values in range from 0 to 1
    pcd.colors = o3d.utility.Vector3dVector(colors_arr.astype(np.float64) / 255)

    if in_place:
        outfile = cloud
    else:
        directory = cloud.parent
        filename = cloud.stem
        outfile = (directory / f"{filename}_colored").with_suffix(".ply")

    o3d.io.write_point_cloud(str(outfile), pcd)
    return outfile


def _color_box(
    *,
    points: np.ndarray,
    colors: np.ndarray,
    left_m: float,
    right_m: float,
    front_m: float,
    back_m: float,
    color_in: np.ndarray[np.float64],
) -> np.ndarray[np.float64]:
    """"""

    new_colors = []
    for point, color in zip(points, colors):
        pt_x = point[0]
        pt_y = point[1]

        if pt_x > left_m and pt_x < right_m \
            and pt_y > front_m and pt_y < back_m:
            new_colors.append(color_in)
        else:
            new_colors.append(color)
    
    return np.array(new_colors)


def _get_color_gradient_in_rng(
    from_: Union[float, int],
    to: Union[float, int],
) -> Dict[Tuple[float, float], np.ndarray]:
    """Returns dictionary with levels as keys and color as value"""

    range_ = abs(from_) + abs(to)
    level_size = range_ / 255
    rng_to_color = {}
    low_border = from_
    i = 1
    while low_border < to + level_size:
        rng_to_color[(low_border, low_border + level_size)] = np.array([255, 255, 255 - i])
        low_border += level_size
        i += 1

    return rng_to_color


def _color_box_gradient(
    *,
    points: np.ndarray,
    colors: np.ndarray,
    left_m: float,
    right_m: float,
    front_m: float,
    back_m: float,
    color_in: np.ndarray[np.float64],
) -> np.ndarray[np.float64]:
    """
    `left_m` and `right_m` in X axis, `front_m` and `back_m` in Y axis.\n
    If one of them is `None` - colors maximum to corresponding side.
    """

    x_points = points[:, 0]
    y_points = points[:, 1]
    box = points[
        (x_points > left_m)
        & (x_points < right_m)
        & (y_points > front_m)
        & (y_points < back_m)
    ]
    box_alt_col = box[:, 2]
    rng_to_color = _get_color_gradient_in_rng(
        from_=box_alt_col.min(),
        to=box_alt_col.max(),
    )

    new_colors = []
    for point, color in zip(points, colors):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]

        # Point in box - replace color
        if pt_x > left_m and pt_x < right_m and pt_y > front_m and pt_y < back_m:

            # Search for point height range, assign corresponding color
            for rng, color_grad in rng_to_color.items():
                if pt_z >= rng[0] and pt_z <= rng[1]:
                    new_colors.append(color_grad / 255)
                    break

        else:
            # Leave default color if point not in the box
            new_colors.append(color)

    return np.array(new_colors)


def color_points_box_segment(
    *,
    points: np.ndarray,
    colors: np.ndarray = np.empty((0, 3)),
    left_m: Union[float, int, None],
    right_m: Union[float, int, None],
    front_m: Union[float, int, None],
    back_m: Union[float, int, None],
    color_in: np.ndarray = np.asarray([255, 255, 0]),
    with_gradient: bool = False,
) -> np.ndarray[np.float64]:
    """
    Expects xyz orientation points with z as height axis.\n
    Colors a corridor along `y` axis with unlimited height in `color_in`.\n
    `color_in` is yellow by default.\n
    Returns new colors array.        ...

    """

    if left_m == 0 and right_m == 0:
        raise ValueError("Given left and right distances are both zero")

    if front_m == 0 and left_m == 0:
        raise ValueError("Given front and back distances are both zero")

    if left_m is None:
        left_m = points[:, 0].min()  # min x
    if right_m is None:
        right_m = points[:, 0].max()  # max x
    if front_m is None:
        front_m = points[:, 1].max()  # max y
    if back_m is None:
        back_m = points[:, 1].min()  # min y

    color_in = color_in.astype(np.float64) / 255

    if colors.size != 0 and colors.size != points.size:
        raise ValueError("Mismatched points and colors array sizes")

    if colors.size == 0:
        colors = colors * points.size

    if with_gradient:
        return _color_box_gradient(
            points=points,
            colors=colors,
            left_m=left_m,
            right_m=right_m,
            front_m=front_m,
            back_m=back_m,
            color_in=color_in,
        )
    else:
        return _color_box(
            points=points,
            colors=colors,
            left_m=left_m,
            right_m=right_m,
            front_m=front_m,
            back_m=back_m,
            color_in=color_in,
        )


def color_ply_box(
    cloud: pathlib.Path,
    *,
    left_m: float,
    right_m: float,
    front_m: float,
    back_m: float,
    in_place: bool = False,
    color: np.ndarray = np.asarray([255, 255, 0]),
    with_gradient: bool = False,
) -> pathlib.Path:
    """
    Expects xyz orientation cloud, with z as height axis.\n
    Colors a corridor along `y` axis with unlimited height in `color`.\n
    `color` is yellow by default.
    """

    pcd = o3d.io.read_point_cloud(str(cloud))
    pts = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    new_colors = color_points_box_segment(
        points=pts,
        colors=colors,
        left_m=left_m,
        right_m=right_m,
        front_m=front_m,
        back_m=back_m,
        color_in=color,
        with_gradient=with_gradient,
    )
    pcd.colors = o3d.utility.Vector3dVector(new_colors)
    if in_place:
        outfile = cloud
    else:
        directory = cloud.parent
        filename = cloud.stem
        outfile = (directory / f"{filename}_box_colored").with_suffix(".ply")

    o3d.io.write_point_cloud(str(outfile), pcd)
    return outfile


def _get_rot_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """`roll` - x, `pitch` - y, `yaw` - z"""

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
