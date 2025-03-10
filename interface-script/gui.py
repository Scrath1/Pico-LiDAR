import sys
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QCheckBox,
    QDoubleSpinBox,
    QSpinBox,
)
from PyQt6.QtCore import Qt, QTimer
from window import Ui_MainWindow
import matplotlib
import numpy as np
import math

matplotlib.use("QtAgg")
import queue
import argparse
from misc import Setting, LidarData, append_rle_encoded, to_lidar_data
import interface as it


class MainWindow(QMainWindow, Ui_MainWindow):
    # UI elements
    _plot_update_timer = QTimer()
    _plot_update_interval_ms = 100

    # Setting value holders
    # Widget connections are initialized later
    settings: dict[Setting]

    # Plot value holders and corresponding queues for receiving data
    _measured_rpm: list[it.PublishedValue] = list()
    _measured_rpm_queue = queue.Queue()
    _target_rpm: list[it.PublishedValue] = list()
    _target_rpm_queue = queue.Queue()
    _pwm: list[it.PublishedValue] = list()
    _pwm_queue = queue.Queue()
    _vl53l0x: list[LidarData] = list()
    _vl53l0x_queue = queue.Queue()
    _hc_sr04: list[LidarData] = list()
    _hc_sr04_queue = queue.Queue()

    _lidar_plot_range: int = 0
    _lidar_data_max_age_ms: int = 0
    _rpm_plot_range: int = 0

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.settings = {
            "Kp": Setting(
                getter=it.get_kp,
                setter=it.set_kp,
                widget=self.ui.doubleSpinBox_kp,
            ),
            "Ki": Setting(
                getter=it.get_ki,
                setter=it.set_ki,
                widget=self.ui.doubleSpinBox_ki,
            ),
            "Kd": Setting(
                getter=it.get_kd,
                setter=it.set_kd,
                widget=self.ui.doubleSpinBox_kd,
            ),
            "targetRPM": Setting(
                getter=it.get_target_rpm,
                setter=it.set_target_rpm,
                widget=self.ui.spinBox_tgt_rpm,
            ),
            "motorOn": Setting(
                getter=it.get_motor_state,
                setter=None,
                widget=self.ui.checkBox_motor_on,
            ),  # Setter handled using Qt checkbox handler
            "PointsPerRev": Setting(
                getter=it.get_datapoints_per_rev,
                setter=it.set_datapoints_per_rev,
                widget=self.ui.spinBox_points_per_rev,
            ),
            "Vl53L0X_Budget": Setting(
                getter=it.get_vl53l0x_budget,
                setter=it.set_vl53l0x_budget,
                widget=self.ui.spinBox_vl53l0x_budget,
            ),
            "angleOffset": Setting(
                getter=it.get_angle_offset,
                setter=it.set_angle_offset,
                widget=self.ui.spinBox_angle_offset,
            ),
        }
        # Disable keyboard tracking for spinboxes to allow for enter key to confirm
        # changes rather than applying a change on every typed character
        for k, s in self.settings.items():
            if isinstance(s.widget, QSpinBox) or isinstance(s.widget, QDoubleSpinBox):
                s.widget.setKeyboardTracking(False)
        self.connect_signals_slots()
        self.subscribe_data_sources()
        self._plot_update_timer.timeout.connect(self.plot_handler_update)
        self._plot_update_timer.start(self._plot_update_interval_ms)
        self._lidar_plot_range = self.ui.slider_range_lidar_plot.value()
        self.ui.display_lidar_plot_zoom.display(self._lidar_plot_range)
        self._rpm_plot_range = self.ui.slider_range_rpm_plot.value()
        self.ui.display_rpm_plot_zoom.display(self._rpm_plot_range)
        self.btn_handler_read_from_device()

    def connect_signals_slots(self):
        self.ui.btn_read_from_device.pressed.connect(self.btn_handler_read_from_device)
        self.ui.btn_reset_device.pressed.connect(self.btn_handler_reset_device)
        self.ui.btn_rpm_plot_reset.pressed.connect(self.btn_handler_rpm_plot_reset)
        self.ui.btn_lidar_plot_reset.pressed.connect(self.btn_handler_lidar_plot_reset)
        self.ui.checkBox_motor_on.clicked.connect(self.checkbox_handler_motor_on)
        self.ui.doubleSpinBox_kp.valueChanged.connect(self.spinbox_handler_kp)
        self.ui.doubleSpinBox_ki.valueChanged.connect(self.spinbox_handler_ki)
        self.ui.doubleSpinBox_kd.valueChanged.connect(self.spinbox_handler_kd)
        self.ui.spinBox_tgt_rpm.valueChanged.connect(self.spinbox_handler_tgt_rpm)
        self.ui.spinBox_points_per_rev.valueChanged.connect(
            self.spinbox_handler_points_per_rev
        )
        self.ui.spinBox_vl53l0x_budget.valueChanged.connect(
            self.spinbox_handler_vl53l0x_budget
        )
        self.ui.slider_range_rpm_plot.valueChanged.connect(
            self.slider_handler_rpm_plot_range
        )
        self.ui.slider_range_lidar_plot.valueChanged.connect(
            self.slider_handler_lidar_plot_range
        )

    def set_lidar_data_max_age_ms(self, val: int):
        self._lidar_data_max_age_ms = val

    def subscribe_data_sources(self):
        self._measured_rpm_queue = it.subscribe("measuredRPM", 200)
        self._target_rpm_queue = it.subscribe("targetRPM", 200)
        self._pwm_queue = it.subscribe("PWM", 200)
        self._vl53l0x_queue = it.subscribe("VL53L0X", 200)
        self._hc_sr04_queue = it.subscribe("HC-SR04", 200)

    def btn_handler_read_from_device(self):
        for k, s in self.settings.items():
            val = s.getter()
            s.value = val
            if isinstance(s.widget, QSpinBox) or isinstance(s.widget, QDoubleSpinBox):
                s.widget.setValue(val)
            elif isinstance(s.widget, QCheckBox):
                s.widget.setChecked(bool(val))

    def btn_handler_reset_device(self):
        it.reset_mcu()

    def btn_handler_rpm_plot_reset(self):
        self._measured_rpm.clear()
        self._target_rpm.clear()
        self._pwm.clear()

    def slider_handler_rpm_plot_range(self, value):
        self._rpm_plot_range = value

    def btn_handler_lidar_plot_reset(self):
        self._hc_sr04.clear()
        self._vl53l0x.clear()

    def slider_handler_lidar_plot_range(self, value):
        self._lidar_plot_range = value

    def checkbox_handler_motor_on(self):
        if self.ui.checkBox_motor_on.checkState() == Qt.CheckState.Checked:
            it.start_motor()
        else:
            it.stop_motor()

    def spinbox_handler_kp(self):
        w = self.settings["Kp"].widget
        self.settings["Kp"].value = w.value()
        it.set_kp(w.value())

    def spinbox_handler_ki(self):
        w = self.settings["Ki"].widget
        self.settings["Ki"].value = w.value()
        it.set_ki(w.value())

    def spinbox_handler_kd(self):
        w = self.settings["Kd"].widget
        self.settings["Kd"].value = w.value()
        it.set_kd(w.value())

    def spinbox_handler_tgt_rpm(self):
        w = self.settings["targetRPM"].widget
        self.settings["targetRPM"].value = w.value()
        it.set_target_rpm(w.value())

    def spinbox_handler_points_per_rev(self):
        w = self.settings["PointsPerRev"].widget
        self.settings["PointsPerRev"].value = w.value()
        it.set_datapoints_per_rev(w.value())

    def spinbox_handler_vl53l0x_budget(self):
        w = self.settings["Vl53L0X_Budget"].widget
        self.settings["Vl53L0X_Budget"].value = w.value()
        it.set_vl53l0x_budget(w.value())

    def spinbox_handler_angle_offset(self):
        w = self.settings["angleOffset"].widget
        self.settings["angleOffset"].value = w.value()
        it.set_angle_offset(w.value())

    def pull_queue_data(self):
        vals = it.get_published_values(self._measured_rpm_queue)
        append_rle_encoded(vals, self._measured_rpm)
        vals = it.get_published_values(self._target_rpm_queue)
        append_rle_encoded(vals, self._target_rpm)
        vals = it.get_published_values(self._pwm_queue)
        append_rle_encoded(vals, self._pwm)
        vals = it.get_published_values(self._vl53l0x_queue)
        self._vl53l0x += to_lidar_data(vals)
        vals = it.get_published_values(self._hc_sr04_queue)
        self._hc_sr04 += to_lidar_data(vals)

    def plot_handler_update(self):
        self.pull_queue_data()
        if(len(self._hc_sr04) > 0):
            self.ui.display_hc_sr04.display(self._hc_sr04[-1].distance)
        else:
            self.ui.display_hc_sr04.display(-1)
        if(len(self._vl53l0x) > 0):
            self.ui.display_vl53l0x.display(self._vl53l0x[-1].distance)
        else:
            self.ui.display_vl53l0x.display(-1)
        self.draw_rpm_plot()
        self.draw_lidar_plot()

    def draw_rpm_plot(self):
        fig = self.ui.plot_rpm.fig
        fig.clear()
        measured_rpm_timestamps = [v.timestamp for v in self._measured_rpm]
        measured_rpm_values = [v.value for v in self._measured_rpm]
        target_rpm_timestamps = [v.timestamp for v in self._target_rpm]
        target_rpm_values = [v.value for v in self._target_rpm]
        # define plot y limits for the rpm and pwm plots
        ylimits_rpm = [0, max(measured_rpm_values + target_rpm_values + [60]) * 1.5]
        ylimits_pwm = [0, 100]
        xlimits = [0]
        if len(measured_rpm_timestamps) > 0:
            xlimits = [
                measured_rpm_timestamps[-1] + self._rpm_plot_range,
                measured_rpm_timestamps[-1],
            ]

        if len(self._measured_rpm) > 0 and len(self._target_rpm) > 0:
            rpm_plot = fig.add_subplot()
            color_mRPM = "C0"
            color_tRPM = "C1"
            rpm_plot.set_ylim(bottom=ylimits_rpm[0], top=ylimits_rpm[1])
            if self._rpm_plot_range == 0:
                # Enable autoscaling on x-axis
                rpm_plot.autoscale(axis="x")
            else:
                # Set limit based on age
                rpm_plot.set_xlim(left=xlimits[0], right=xlimits[1])
            rpm_plot.plot(
                measured_rpm_timestamps,
                measured_rpm_values,
                label="measured RPM",
                color=color_mRPM,
            )
            rpm_plot.plot(
                target_rpm_timestamps,
                target_rpm_values,
                label="target RPM",
                color=color_tRPM,
                linestyle='dashed'
            )
            rpm_plot.set_xlabel("Timestamp (ms)")
            rpm_plot.set_ylabel("RPM")
            rpm_plot.legend()
            rpm_plot.grid()

        if len(self._pwm) > 0:
            pwm_timestamps = [v.timestamp for v in self._pwm]
            pwm_values = [v.value for v in self._pwm]
            pwm_plot = fig.add_subplot(frame_on=False)
            pwm_plot.set_ylim(bottom=ylimits_pwm[0], top=ylimits_pwm[1])
            if self._rpm_plot_range == 0:
                # Enable autoscaling on x-axis
                pwm_plot.autoscale(axis="x")
            else:
                # Set limit based on age
                pwm_plot.set_xlim(left=xlimits[0], right=xlimits[1])
            color_pwm = "C2"
            pwm_plot.plot(pwm_timestamps, pwm_values, label="PWM", color=color_pwm)
            pwm_plot.set_ylabel("PWM Duty Cycle", color=color_pwm)
            pwm_plot.tick_params(axis="y", colors=color_pwm)
            pwm_plot.yaxis.set_label_position("right")
            pwm_plot.yaxis.set_ticks_position("right")
            pwm_plot.set_xticks([])

        fig.tight_layout()
        self.ui.plot_rpm.draw()

    def draw_lidar_plot(self):
        # Discard data that is too old
        cutoff_timestamp_ms = it.get_timestamp_ms() - self._lidar_data_max_age_ms
        self._vl53l0x = [v for v in self._vl53l0x if v.timestamp > cutoff_timestamp_ms]
        self._hc_sr04 = [v for v in self._hc_sr04 if v.timestamp > cutoff_timestamp_ms]

        vl53l0x_a = [math.radians(v.angle) for v in self._vl53l0x if v.angle != -1]
        vl53l0x_d = [v.distance for v in self._vl53l0x if v.angle != -1]
        hc_sr04_a = [math.radians(v.angle) for v in self._hc_sr04 if v.angle != -1]
        hc_sr04_d = [v.distance for v in self._hc_sr04 if v.angle != -1]

        fig = self.ui.plot_lidar.fig
        fig.clear()
        lidar_plot = fig.add_subplot(projection = 'polar')
        lidar_plot.grid(True)
        lidar_plot.set_ylim(0, self._lidar_plot_range)
        # Make 0 degree face north and invert angle direction
        lidar_plot.set_theta_zero_location("N")
        lidar_plot.set_theta_direction(-1)

        # Add spaced distance markers
        rticks = np.linspace(0, self._lidar_plot_range, num=5)
        lidar_plot.set_rticks(rticks)
        if len(vl53l0x_a) > 1 or len(hc_sr04_a) > 1:
            lidar_plot.scatter(vl53l0x_a, vl53l0x_d, color="r", label="VL53L0X")
            lidar_plot.scatter(hc_sr04_a, hc_sr04_d, color="b", label="HC-SR04")
            lidar_plot.legend()

        self.ui.plot_lidar.draw()


app = None
def run_gui(args: argparse.Namespace):
    global app
    app = QApplication(sys.argv)
    window = MainWindow()
    if(args.age):
        window.set_lidar_data_max_age_ms(args.age)
    window.show()
    return app.exec()


def close_gui():
    global app
    if app != None:
        app.exit()
