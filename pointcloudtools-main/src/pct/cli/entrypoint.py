import pathlib

import click

ROS_OK = True
try:
    from pct.ros.io import create_point_cloud_from_bags
except ImportError:
    ROS_OK = False




def _filter_out_bagfiles(dir_: pathlib.Path):
    """
    Filters only bag files from dir,
     returns sorted list of `pathlib.Path` objects to bag files
    """

    def bag_idx(file_path: pathlib.Path) -> int:
        return int(file_path.stem.split("_")[-1])

    res = []
    for file in dir_.iterdir():
        if file.suffix == ".bag":
            res.append(file.absolute())

    return sorted(res, key=bag_idx)


@click.group()
def main() -> None:
    ...


@click.command("from-bags")
@click.argument("bags_dir", type=click.Path(exists=True), metavar="Путь к директории с .bag-файлами")
@click.option("-s", "--save-name", type=str, default=None, help="Имя сохраняемого файла")
def create_point_cloud_file_from_bags(bags_dir: str, save_name: str) -> None:
    if not ROS_OK:
        print("ROS module isn't installed. Reinstall lib with ROS or All option")
        return

    bags_d = pathlib.Path(bags_dir)
    bags = _filter_out_bagfiles(bags_d)
    cloud = create_point_cloud_from_bags(
        bags=bags,
        in_dir=pathlib.Path("./"),
        with_name=bags_d.name if save_name is None else save_name,
    )


main.add_command(create_point_cloud_file_from_bags)
