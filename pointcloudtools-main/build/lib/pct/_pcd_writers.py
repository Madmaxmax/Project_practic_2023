"""Writers for writing in specific point cloud formats"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
import pathlib

import numpy as np
import open3d as o3d


class ABCWriter(ABC):
    @abstractmethod
    def write(
        self,
        fname: str,
        dir_: pathlib.Path,
        np_points: np.ndarray,
        **kwargs,
    ) -> pathlib.Path:
        ...


@dataclass
class PlyWriter(ABCWriter):
    def write(
        self,
        fname: str,
        dir_: pathlib.Path,
        np_points: np.ndarray,
        **kwargs,
    ) -> pathlib.Path:
        """"""

        # TODO add colors
        pth = dir_ / f"{fname}.ply"
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np_points)
        o3d.io.write_point_cloud(str(pth), **kwargs)
        return pth


@dataclass
class XYZWriter(ABCWriter):
    def write(
        self,
        fname: str,
        dir_: pathlib.Path,
        np_points: np.ndarray,
    ) -> pathlib.Path:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)
        xyz_cloud = (dir_ / fname).with_suffix(".xyz")
        o3d.io.write_point_cloud(str(xyz_cloud), pcd)
        return xyz_cloud


@dataclass
class PcdWriter(ABCWriter):
    def write(self, np_points: np.ndarray) -> None:
        ...


@dataclass
class LasWriter(ABCWriter):
    def write(self, np_points: np.ndarray) -> None:
        ...


@dataclass
class LazWriter(ABCWriter):
    def write(self, np_points: np.ndarray) -> None:
        ...
