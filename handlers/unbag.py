from entities import *
import pathlib
from pct.ros.io import create_point_cloud_from_bags


def start(q: Query):
    bags = q.source_dir
    bags = [pathlib.Path(bag) for bag in bags]
    file_name = "cloud"
    q.output_dir += file_name + ".ply"
    print(q.output_dir)
    point_cloud = create_point_cloud_from_bags(
        bags=bags,
        in_dir=pathlib.Path(q.output_dir),
        with_name="file_name"
    )
    return q.make_response()

