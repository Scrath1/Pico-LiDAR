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
        
SettingContainer = namedtuple('SettingContainer', ["value", "getter", "setter", "widget"])

class MainWindow(QMainWindow, Ui_MainWindow):
    # UI elements
    _plot_update_timer = QTimer()
    _plot_update_interval_ms = 250
    
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
            "Kp": SettingContainer(value=Setting(), getter=si.get_kp, setter=si.set_kp, widget=self.ui.doubleSpinBox_kp),
            "Ki": SettingContainer(value=Setting(), getter=si.get_ki, setter=si.set_ki, widget=self.ui.doubleSpinBox_ki),
            "Kd": SettingContainer(value=Setting(), getter=si.get_kd, setter=si.set_kd, widget=self.ui.doubleSpinBox_kd),
            "targetRPM": SettingContainer(value=Setting(), getter=si.get_target_rpm, setter=si.set_target_rpm, widget=self.ui.spinBox_tgt_rpm),
            "motorOn": SettingContainer(value=Setting(), getter=si.get_motor_state, setter=None, widget=self.ui.checkBox_motor_on), # Setter handled using Qt checkbox handler
            "PointsPerRev": SettingContainer(value=Setting(), getter=si.get_datapoints_per_rev, setter=si.set_datapoints_per_rev , widget=self.ui.spinBox_points_per_rev),
            "Vl53L0X_Budget": SettingContainer(value=Setting(), getter=si.get_vl53l0x_budget, setter=si.set_vl53l0x_budget, widget=self.ui.spinBox_vl53l0x_budget),
            "angleOffset": SettingContainer(value=Setting(), getter=si.get_angle_offset, setter=si.set_angle_offset, widget=self.ui.spinBox_angle_offset)
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
        for k, s in self.settings.items():
            if(s.setter != None):
                if s.value.was_changed():
                    s.setter(s.value.get())
                    s.value.acknowledge_change()

    def btn_handler_reset_device(self):
        si.reset_mcu()
        
    def btn_handler_rpm_plot_reset(self):
        self._measured_rpm.clear()
        self._target_rpm.clear()
        self._pwm.clear()
        
    def btn_handler_lidar_plot_reset(self):
        self._hc_sr04.clear()
        self._vl53l0x.clear()
        
    def checkbox_handler_motor_on(self):
        if self.ui.checkBox_motor_on.checkState() == Qt.CheckState.Checked:
            si.start_motor()
            print("Motor on")
        else:
            si.stop_motor()
            print("Motor off")
            
    def spinbox_handler_kp(self):
        w = self.settings["Kp"].widget
        self.settings["Kp"].value.set(w.value())
        
    def spinbox_handler_ki(self):
        w = self.settings["Ki"].widget
        self.settings["Ki"].value.set(w.value())
        
    def spinbox_handler_kd(self):
        w = self.settings["Kd"].widget
        self.settings["Kd"].value.set(w.value())
        
    def spinbox_handler_tgt_rpm(self):
        w = self.settings["targetRPM"].widget
        self.settings["targetRPM"].value.set(w.value())
        
    def spinbox_handler_points_per_rev(self):
        w = self.settings["PointsPerRev"].widget
        self.settings["PointsPerRev"].value.set(w.value())
        
    def spinbox_handler_vl53l0x_budget(self):
        w = self.settings["Vl53L0X_Budget"].widget
        self.settings["Vl53L0X_Budget"].value.set(w.value())
        
    def spinbox_handler_angle_offset(self):
        w = self.settings["angleOffset"].widget
        self.settings["angleOffset"].value.set(w.value())
        
    def pull_queue_data(self):
        def to_lidar_data(vals: si.PublishedValue) -> list[MainWindow.LidarData]:
            out = list()
            for v in vals:
                angle = v.value[0]
                distance = v.value[1]
                data = None
                if angle != -1:
                    x = distance * np.cos(math.radians(angle))
                    y = distance * np.sin(math.radians(angle))
                    data = MainWindow.LidarData(timestamp=v.timestamp, angle=angle, distance=distance, x_coord=x, y_coord=y)
                else:
                    data = MainWindow.LidarData(timestamp=v.timestamp, angle=angle, distance=distance, x_coord=0, y_coord=0)
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
        self.draw_rpm_plot()
        self.draw_lidar_plot()
        
    def draw_rpm_plot(self):
        fig = self.ui.plot_rpm.fig
        fig.clear()
        measured_rpm_timestamps = [v.timestamp for v in self._measured_rpm]
        measured_rpm_values = [v.value for v in self._measured_rpm]
        target_rpm_timestamps = [v.timestamp for v in self._target_rpm]
        target_rpm_values = [v.value for v in self._target_rpm]
        # define plot limits for the rpm and pwm plots
        limits = [{0, max(measured_rpm_values + target_rpm_values + [60]) * 2},{0, 100}]
        
        if len(self._measured_rpm) > 0 and len(self._target_rpm) > 0:
            rpm_plot = fig.add_subplot()
            color_mRPM = "C0"
            color_tRPM = "C1"
            rpm_plot.set_ylim(limits[0])
            rpm_plot.set_ylim(limits[0])
            rpm_plot.plot(measured_rpm_timestamps, measured_rpm_values, label="measured RPM", color=color_mRPM)
            rpm_plot.plot(target_rpm_timestamps, target_rpm_values, label="target RPM", color=color_tRPM)
            rpm_plot.set_xlabel("Timestamp (ms)")
            rpm_plot.set_ylabel("RPM")
            rpm_plot.legend()
            rpm_plot.grid()

        if len(self._pwm) > 0:
            pwm_timestamps = [v.timestamp for v in self._pwm]
            pwm_values = [v.value for v in self._pwm]
            pwm_plot = fig.add_subplot(frame_on=False)
            pwm_plot.set_ylim(limits[1])
            color_pwm = "C2"
            pwm_plot.plot(pwm_timestamps, pwm_values, label="PWM", color=color_pwm)
            pwm_plot.set_ylabel("PWM Duty Cycle", color=color_pwm)
            pwm_plot.tick_params(axis='y', colors=color_pwm)
            pwm_plot.yaxis.set_label_position('right')
            pwm_plot.yaxis.set_ticks_position('right')
            pwm_plot.set_xticks([])
        self.ui.plot_rpm.draw()
        
    def draw_lidar_plot(self):
        # ToDo: Add slider for selecting limits
        # ToDo: Filter out too old data
        # ToDo: Fix window scaling for fullscreen support
        vl53l0x_x = [v.x_coord for v in self._vl53l0x]
        vl53l0x_y = [v.y_coord for v in self._vl53l0x]
        hc_sr04_x = [v.x_coord for v in self._hc_sr04]
        hc_sr04_y = [v.y_coord for v in self._hc_sr04]
        
        fig = self.ui.plot_lidar.fig
        fig.clear()
        lidar_plot = fig.add_subplot()
        lidar_plot.grid()
        lidar_plot.axhline(y=0, color='r', linestyle='-')
        lidar_plot.axvline(x=0, color='r', linestyle='-')
        
        if len(vl53l0x_x) > 1 or len(hc_sr04_x) > 1:
            lidar_plot.set_xlabel("X Position (mm)")
            lidar_plot.set_ylabel("Y Position (mm)")
            lidar_plot.scatter(vl53l0x_x, vl53l0x_y, color="r", label="VL53L0X")
            lidar_plot.scatter(hc_sr04_x, hc_sr04_y, color="b", label="HC-SR04")
            lidar_plot.legend()
        else:
            lidar_plot.text(0.5, 0.5, "No data")
        
        self.ui.plot_lidar.draw()

def run_gui():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())