from entities import *
import pathlib
from pct.utils import rotate_point_cloud_by_axis


def start(q: Query):
    point_cloud = pathlib.Path(q.source_dir[0])
    rotated_cloud = rotate_point_cloud_by_axis(
        cloud=point_cloud,
        ax=q.params['ax'],
        rad=q.params['rad']
    )
    return q.make_response()

