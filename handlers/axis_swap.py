from entities import *
import pathlib
from pct.utils import swap_point_cloud_file_axes


def start(q: Query):
    point_cloud = pathlib.Path(q.source_dir[0])
    swapped_cloud = swap_point_cloud_file_axes(
        cloud=point_cloud,
        ax=q.params["ax"],
        with_ax=q.params["with_ax"]
    )
    return q.make_response()
