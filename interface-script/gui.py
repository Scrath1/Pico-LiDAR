import sys
from PyQt6.QtWidgets import QApplication, QWidget, QMainWindow, QCheckBox, QDoubleSpinBox, QSpinBox
from PyQt6.QtCore import Qt, QTimer
from window import Ui_MainWindow
import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import serial_interface as si
import queue
from collections import namedtuple
import numpy as np
import math

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
        
SettingContainer = namedtuple('SettingContainer', ["value", "getter", "widget"])

class MainWindow(QMainWindow, Ui_MainWindow):
    # UI elements
    _plot_update_timer = QTimer()
    _plot_update_interval_ms = 1000
    
    # Setting value holders
    # Widget connections are initialized later
    settings: dict[SettingContainer]
    _target_rpm = Setting()
    _points_per_rev = Setting()
    _vl53l0x_scantime_budget_us = Setting()
    _scan_angle_offset = Setting()
    
    LidarData = namedtuple('LidarData', ['timestamp', 'angle', 'distance', 'x_coord', 'y_coord'])
    
    # Plot value holders and corresponding queues for receiving data
    _measured_rpm: list[si.PublishedValue] = list()
    _measured_rpm_queue = queue.Queue()
    _target_rpm: list[si.PublishedValue] = list()
    _target_rpm_queue = queue.Queue()
    _pwm: list[si.PublishedValue] = list()
    _pwm_queue = queue.Queue()
    _vl53l0x: list[LidarData] = list()
    _vl53l0x_queue = queue.Queue()
    _hc_sr04: list[LidarData] = list()
    _hc_sr04_queue = queue.Queue()
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.settings = {
            "Kp": SettingContainer(value=Setting(), getter=si.get_kp, widget=self.ui.doubleSpinBox_kp),
            "Ki": SettingContainer(value=Setting(), getter=si.get_ki, widget=self.ui.doubleSpinBox_ki),
            "Kd": SettingContainer(value=Setting(), getter=si.get_kd, widget=self.ui.doubleSpinBox_kd),
            "targetRPM": SettingContainer(value=Setting(), getter=si.get_target_rpm, widget=self.ui.spinBox_tgt_rpm),
            "motorOn": SettingContainer(value=Setting(), getter=si.get_motor_state, widget=self.ui.checkBox_motor_on),
            "PointsPerRev": SettingContainer(value=Setting(), getter=si.get_datapoints_per_rev, widget=self.ui.spinBox_points_per_rev),
            "Vl53L0X_Budget": SettingContainer(value=Setting(), getter=si.get_vl53l0x_budget, widget=self.ui.spinBox_vl53l0x_budget),
            "angleOffset": SettingContainer(value=Setting(), getter=si.get_angle_offset, widget=self.ui.spinBox_angle_offset)
        }
        self.connect_signals_slots()
        self.subscribe_data_sources()
        self._plot_update_timer.timeout.connect(self.plot_handler_update)
        self._plot_update_timer.start(self._plot_update_interval_ms)
        self.btn_handler_read_from_device()
    
    def connect_signals_slots(self):
        self.ui.btn_read_from_device.pressed.connect(self.btn_handler_read_from_device)
        self.ui.btn_send_to_device.pressed.connect(self.btn_handler_send_to_device)
        self.ui.btn_reset_device.pressed.connect(self.btn_handler_reset_device)
        self.ui.btn_rpm_plot_reset.pressed.connect(self.btn_handler_rpm_plot_reset)
        self.ui.btn_lidar_plot_reset.pressed.connect(self.btn_handler_lidar_plot_reset)
        self.ui.checkBox_motor_on.clicked.connect(self.checkbox_handler_motor_on)
        self.ui.doubleSpinBox_kp.valueChanged.connect(self.spinbox_handler_kp)
        self.ui.doubleSpinBox_ki.valueChanged.connect(self.spinbox_handler_ki)
        self.ui.doubleSpinBox_kd.valueChanged.connect(self.spinbox_handler_kd)
        self.ui.spinBox_tgt_rpm.valueChanged.connect(self.spinbox_handler_tgt_rpm)
        self.ui.spinBox_points_per_rev.valueChanged.connect(self.spinbox_handler_points_per_rev)
        self.ui.spinBox_vl53l0x_budget.valueChanged.connect(self.spinbox_handler_vl53l0x_budget)
    
    def subscribe_data_sources(self):
        self._measured_rpm_queue = si.subscribe("measuredRPM", 200)
        self._target_rpm_queue = si.subscribe("targetRPM", 200)
        self._pwm_queue = si.subscribe("PWM", 200)
        self._vl53l0x_queue = si.subscribe("VL53L0X", 200)
        self._hc_sr04_queue = si.subscribe("HC-SR04", 200)
    
    def btn_handler_read_from_device(self):
        for k, s in self.settings.items():
            val = s.getter()
            s.value.set(val)
            s.value.acknowledge_change()
            if isinstance(s.widget, QSpinBox) or isinstance(s.widget, QDoubleSpinBox):
                s.widget.setValue(val)
            elif isinstance(s.widget, QCheckBox):
                s.widget.setChecked(bool(val))
    
    def btn_handler_send_to_device(self):
        print("send to device")

    def btn_handler_reset_device(self):
        si.reset_mcu()
        
    def btn_handler_rpm_plot_reset(self):
        print("reset RPM plot")
        
    def btn_handler_lidar_plot_reset(self):
        print("reset lidar plot")
        
    def checkbox_handler_motor_on(self):
        if self.ui.checkBox_motor_on.checkState() == Qt.CheckState.Checked:
            si.start_motor()
            print("Motor on")
        else:
            si.stop_motor()
            print("Motor off")
            
    def spinbox_handler_kp(self):
        print("spinbox kp")
        
    def spinbox_handler_ki(self):
        print("spinbox ki")
        
    def spinbox_handler_kd(self):
        print("spinbox kd")
        
    def spinbox_handler_tgt_rpm(self):
        print("spinbox target rpm")
        
    def spinbox_handler_points_per_rev(self):
        print("spinbox ppr")
        
    def spinbox_handler_vl53l0x_budget(self):
        print("spinbox vl53l0x budget")
        
    def spinbox_handler_angle_offset(self):
        print("spinbox angle offset")
        
    def pull_queue_data(self):
        def to_lidar_data(vals: si.PublishedValue) -> list[MainWindow.LidarData]:
            out = list()
            for v in vals:
                angle = v.value[0]
                distance = v.value[1]
                data = MainWindow.LidarData(timestamp=v.timestamp, angle=angle, distance=distance, x_coord=0, y_coord=0)
                if angle != -1:
                    data.x_coord = distance * np.cos(math.radians(angle))
                    data.y_coord = distance * np.sin(math.radians(angle))
                out.append(data)
            return out
        vals = si.get_published_values(self._measured_rpm_queue)
        self._measured_rpm += vals
        vals = si.get_published_values(self._target_rpm_queue)
        self._target_rpm += vals
        vals = si.get_published_values(self._pwm_queue)
        self._pwm += vals
        vals = si.get_published_values(self._vl53l0x_queue)
        self._vl53l0x += to_lidar_data(vals)
        vals = si.get_published_values(self._hc_sr04_queue)
        self._hc_sr04 += to_lidar_data(vals)
        return
        
    def plot_handler_update(self):
        self.pull_queue_data()
        print("Plot update")

def run_gui():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())