from collections import namedtuple
from serial_interface import PublishedValue
import math
import numpy as np


class Setting:
    _changed: bool = False
    _value: float | int = 0

    def set(self, val: float | int):
        self._value = val
        self._changed = True

    def get(self) -> float | int:
        return self._value

    def was_changed(self) -> bool:
        return self._changed

    def acknowledge_change(self):
        self._changed = False


SettingContainer = namedtuple(
    "SettingContainer", ["value", "getter", "setter", "widget"]
)
LidarData = namedtuple(
    "LidarData", ["timestamp", "angle", "distance", "x_coord", "y_coord"]
)

def to_lidar_data(vals: PublishedValue) -> list[LidarData]:
    out = list()
    for v in vals:
        angle = v.value[0]
        distance = v.value[1]
        data = None
        if angle != -1:
            x = distance * np.cos(math.radians(angle))
            y = distance * np.sin(math.radians(angle))
            data = LidarData(
                timestamp=v.timestamp,
                angle=angle,
                distance=distance,
                x_coord=x,
                y_coord=y,
            )
        else:
            data = LidarData(
                timestamp=v.timestamp,
                angle=angle,
                distance=distance,
                x_coord=0,
                y_coord=0,
            )
        out.append(data)
    return out


def append_rle_encoded(vals: PublishedValue, tgt_list: list[PublishedValue]):
    """Checks all values in vals against the last value of tgt_list and the next
    value of vals.
    While each value is approximately equal, instead of the value being added
    to target list, the timestamp of the last value is shifted to match the
    new one. Doing this means that only the two samples to the left and right
    edge of a signal edge are added to the list, thereby saving memory.

    P => Penultimate value in tgt_list
    L => Last value in tgt_list
    C => Current value of vals
    N => Next value of vals

    Case 1: All signals are similar. Set timestamp of L to timestamp of C
        v <- loop is here
    P L C N
    _______

    Case 2: No value change between last, current and penultimate. Set
    timestamp of L to timestamp of C
        v <- loop is here
    P L C N
    _____/-
    
    Case 3: Value change between last and current. Last value is already part
    of tgt_list and the current needs to be added to record the signal edge
        v <- loop is here
    P L C N
    ___/---

    Case 4: No value change between last and current, but we can't shift the
    timestamp of the last value because we would loose the signal edge.
    Add current value to tgt_list
        v <- loop is here
    P L C N
    _/-----
    
    Args:
        vals (si.PublishedValue): List of values to check and maybe add
        tgt_list (list[si.PublishedValue]): List to add values to
    """
    for idx, v in enumerate(vals):  # Leave out last element for the
        if isinstance(v.value, list):
            # This function can only deal with the 
            tgt_list.append(v)

        penultimateVal: PublishedValue = tgt_list[-2] if len(tgt_list) > 1 else PublishedValue(0, float("inf"))
        lastVal: PublishedValue = tgt_list[-1] if len(tgt_list) > 0 else PublishedValue(0, float("inf"))
        
        penultimate_to_last_similar: bool = penultimateVal.values_approx_equal(lastVal)
        last_to_current_similar: bool = lastVal.values_approx_equal(v)
        if penultimate_to_last_similar and last_to_current_similar:
            # Case 1 and 2
            lastVal.timestamp = v.timestamp
        else:
            # Case 3 and 4
            tgt_list.append(v)
            