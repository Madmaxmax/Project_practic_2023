from pct._pcd_writers import (
    PlyWriter,
    PcdWriter,
    LasWriter,
    LazWriter,
)


FILE_FORMAT_TO_WRITER = {
    "ply": PlyWriter,
    "pcd": PcdWriter,
    "las": LasWriter,
    "laz": LazWriter,
}

LASER_TOPIC = "/velodyne_points"
IMU_TOPIC = "/mavros/imu/data"
GPS_TOPIC = "/mavros/global_position/global"
