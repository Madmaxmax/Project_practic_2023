# Установка
В одной директории с `pyproject.toml`:
```
pip install ./
```
Если требуется создавать облака точек из ROS `.bag` файлов -> дополнительно:
```
./install.sh
```

---

# Пример использования API

### Создание облака точек из `.bag` файлов
```python
import pathlib

from pct.ros.io import create_point_cloud_from_bags

bags = [
    "/tmp/bags/fragment_2023-04-19-16-16-26_0.bag",
    "/tmp/bags/fragment_2023-04-19-16-16-56_1.bag",
    "/tmp/bags/fragment_2023-04-19-16-17-26_2.bag",
    "/tmp/bags/fragment_2023-04-19-16-17-56_3.bag",
    "/tmp/bags/fragment_2023-04-19-16-18-26_4.bag",
]
# Для корректной работы требуются объекты типа Path
bags = [pathlib.Path(bag) for bag in bags]

point_cloud = create_point_cloud_from_bags(
    bags=bags,
    in_dir=pathlib.Path("./"),
    with_name="cloud",
)
# В результате будет создано окрашенное по высоте облако точек ./cloud.ply
```

### Смена осей местами в существующем файле облака точек
```python
import pathlib

from pct.utils import swap_point_cloud_file_axes

point_cloud = pathlib.Path("./cloud.ply")
# Смена осей Y и Z местами
swapped_cloud = swap_point_cloud_file_axes(
    cloud=point_cloud,
    ax="y",  # Может быть "z" или индекс оси
    with_ax="z",  # Может быть "y" или индекс оси
)
# В результате будет создано облако точек ./cloud_swapped_ax_y_with_z.ply
# Создается в одной директории с облаком у которого меняли оси
```

### Вращение облака точек вокруг заданной оси
```python
import pathlib

from pct.utils import rotate_point_cloud_by_axis

point_cloud = pathlib.Path("./cloud.ply")
# Вращение облака точек вокруг оси y на 0.261799 радиан
rotated_cloud = rotate_point_cloud_by_axis(
    cloud=point_cloud,
    ax="y",
    rad=0.261799,
)
# В результате будет создано облако точек ./cloud_rotated_y_0.261799.ply
# Создается в одной директории с вращаемым облаком
```

### Конвертация формата `.ply` в формат  `.xyz`
```python
import pathlib

from pct.utils import convert_ply_to_xyz


ply_cloud = pathlib.Path("./cloud.ply")
xyz_cloud = convert_ply_to_xyz(ply_cloud)
# В результате будет создано облако точек ./cloud.xyz
# Создается в одной директории с конвертируемым облаком
```

### Окрашивание облака точек по высоте
```python
import pathlib

from pct.utils import color_ply_by_height


point_cloud = "./cloud.ply"
colored_cloud = color_ply_by_height(
    cloud=point_cloud,
    height_axis="z",
)
# В результате будет создано облако точек ./cloud_colored.ply
# Создается в одной директории с окрашиваемым облаком
```
