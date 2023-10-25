from typing import Union, List, Tuple

from attrs import define
import numpy as np
import haversine


@define
class GPSState:
    captured: bool

    start_lat: Union[float, None]
    start_lon: Union[float, None]
    start_alt: Union[float, None]
    
    _cur_lat: Union[float, None]
    _cur_lon: Union[float, None]
    _cur_alt: Union[float, None]

    prev_passed_dist_m: Union[float, None]

    @classmethod
    def empty(cls) -> object:
        return cls(
            captured=False,
            start_lat=None,
            start_lon=None,
            start_alt=None,
            cur_lat=None,
            cur_lon=None,
            cur_alt=None,
            prev_passed_dist_m=None,
        )

    @property
    def passed_dist_m(self) -> Union[float, None]:
        if not self.captured:
            return None
        return haversine.haversine(
            (self.start_lat, self.start_lon),
            (self._cur_lat, self._cur_lon),
            unit=haversine.Unit.METERS,
        )

    # TODO meters?
    @property
    def alt_diff_m(self) -> Union[float, None]:
        if not self.captured:
            return None
        return self._cur_alt - self.start_alt    

    @property
    def prev_passed_dist_m(self) -> Union[float, None]:
        if not self.captured:
            return None
        return self.prev_passed_dist_m


@define
class LaserState:
    point_cloud_arr: List[np.ndarray]
    cur_data: np.ndarray
    cur_arr: np.ndarray
    rot_mat: np.ndarray

    @classmethod
    def empty(cls) -> object:
        return cls(
            point_cloud_arr=[],
            cur_data=None,
            cur_arr=None,
            rot_mat=None,
        )


@define
class ImuState:
    start_vals: Union[Tuple[float, float, float], None]
    cur_rot_mat: Union[np.ndarray, None]

    @classmethod
    def empty(cls) -> object:
        return cls(
            start_vals=None,
            cur_rot_mat=None,
        )


@define
class ProcessState:
    gps: GPSState
    laser: LaserState
    imu: ImuState

    @classmethod
    def empty(cls) -> object:
        return cls(
            gps=GPSState.empty(),
            laser=LaserState.empty(),
            imu=ImuState.empty(),
        )
