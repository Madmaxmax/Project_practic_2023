from entities import *
import pathlib
from pct.utils import color_ply_by_height


def start(q: Query):
    point_cloud = pathlib.Path(q.source_dir[0])
    colored_cloud = color_ply_by_height(
        cloud=point_cloud,
        height_axis=q.params["height_axis"]
    )
    return q.make_response()
