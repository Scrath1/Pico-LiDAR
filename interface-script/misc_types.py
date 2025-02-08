from collections import namedtuple

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
        
SettingContainer = namedtuple('SettingContainer', ["value", "getter", "setter", "widget"])
LidarData = namedtuple('LidarData', ['timestamp', 'angle', 'distance', 'x_coord', 'y_coord'])