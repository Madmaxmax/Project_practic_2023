from entities import *
import pathlib
from pct.utils import convert_ply_to_xyz


def start(q: Query):
    ply_cloud = pathlib.Path(q.source_dir[0])
    xyz_cloud = convert_ply_to_xyz(ply_cloud)
    return q.make_response()

