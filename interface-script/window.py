# Form implementation generated from reading ui file '.\window.ui'
#
# Created by: PyQt6 UI code generator 6.4.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1154, 837)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.controls = QtWidgets.QVBoxLayout()
        self.controls.setObjectName("controls")
        self.groupBox_general_control = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.groupBox_general_control.setObjectName("groupBox_general_control")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_general_control)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.layout_general_control = QtWidgets.QFormLayout()
        self.layout_general_control.setObjectName("layout_general_control")
        self.btn_read_from_device = QtWidgets.QPushButton(parent=self.groupBox_general_control)
        self.btn_read_from_device.setObjectName("btn_read_from_device")
        self.layout_general_control.setWidget(0, QtWidgets.QFormLayout.ItemRole.SpanningRole, self.btn_read_from_device)
        self.btn_send_to_device = QtWidgets.QPushButton(parent=self.groupBox_general_control)
        self.btn_send_to_device.setObjectName("btn_send_to_device")
        self.layout_general_control.setWidget(1, QtWidgets.QFormLayout.ItemRole.SpanningRole, self.btn_send_to_device)
        self.btn_reset_device = QtWidgets.QPushButton(parent=self.groupBox_general_control)
        self.btn_reset_device.setMinimumSize(QtCore.QSize(75, 0))
        self.btn_reset_device.setLayoutDirection(QtCore.Qt.LayoutDirection.LeftToRight)
        self.btn_reset_device.setObjectName("btn_reset_device")
        self.layout_general_control.setWidget(2, QtWidgets.QFormLayout.ItemRole.SpanningRole, self.btn_reset_device)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.layout_general_control.setItem(3, QtWidgets.QFormLayout.ItemRole.LabelRole, spacerItem)
        self.verticalLayout_3.addLayout(self.layout_general_control)
        self.controls.addWidget(self.groupBox_general_control)
        self.groupBox_motor_control = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.groupBox_motor_control.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Preferred, QtWidgets.QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_motor_control.sizePolicy().hasHeightForWidth())
        self.groupBox_motor_control.setSizePolicy(sizePolicy)
        self.groupBox_motor_control.setFlat(False)
        self.groupBox_motor_control.setCheckable(False)
        self.groupBox_motor_control.setObjectName("groupBox_motor_control")
        self.gridLayoutWidget = QtWidgets.QWidget(parent=self.groupBox_motor_control)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 20, 136, 176))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.layout_motor_control = QtWidgets.QFormLayout(self.gridLayoutWidget)
        self.layout_motor_control.setContentsMargins(0, 0, 0, 0)
        self.layout_motor_control.setObjectName("layout_motor_control")
        self.label_Kp = QtWidgets.QLabel(parent=self.gridLayoutWidget)
        self.label_Kp.setObjectName("label_Kp")
        self.layout_motor_control.setWidget(0, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_Kp)
        self.doubleSpinBox_kp = QtWidgets.QDoubleSpinBox(parent=self.gridLayoutWidget)
        self.doubleSpinBox_kp.setSingleStep(0.05)
        self.doubleSpinBox_kp.setObjectName("doubleSpinBox_kp")
        self.layout_motor_control.setWidget(0, QtWidgets.QFormLayout.ItemRole.FieldRole, self.doubleSpinBox_kp)
        self.label_Ki = QtWidgets.QLabel(parent=self.gridLayoutWidget)
        self.label_Ki.setObjectName("label_Ki")
        self.layout_motor_control.setWidget(1, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_Ki)
        self.doubleSpinBox_ki = QtWidgets.QDoubleSpinBox(parent=self.gridLayoutWidget)
        self.doubleSpinBox_ki.setSingleStep(0.05)
        self.doubleSpinBox_ki.setObjectName("doubleSpinBox_ki")
        self.layout_motor_control.setWidget(1, QtWidgets.QFormLayout.ItemRole.FieldRole, self.doubleSpinBox_ki)
        self.label_Kd = QtWidgets.QLabel(parent=self.gridLayoutWidget)
        self.label_Kd.setObjectName("label_Kd")
        self.layout_motor_control.setWidget(2, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_Kd)
        self.doubleSpinBox_kd = QtWidgets.QDoubleSpinBox(parent=self.gridLayoutWidget)
        self.doubleSpinBox_kd.setSingleStep(0.05)
        self.doubleSpinBox_kd.setObjectName("doubleSpinBox_kd")
        self.layout_motor_control.setWidget(2, QtWidgets.QFormLayout.ItemRole.FieldRole, self.doubleSpinBox_kd)
        self.label_target_rpm = QtWidgets.QLabel(parent=self.gridLayoutWidget)
        self.label_target_rpm.setObjectName("label_target_rpm")
        self.layout_motor_control.setWidget(3, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_target_rpm)
        self.spinBox_tgt_rpm = QtWidgets.QSpinBox(parent=self.gridLayoutWidget)
        self.spinBox_tgt_rpm.setMaximum(10000)
        self.spinBox_tgt_rpm.setObjectName("spinBox_tgt_rpm")
        self.layout_motor_control.setWidget(3, QtWidgets.QFormLayout.ItemRole.FieldRole, self.spinBox_tgt_rpm)
        self.checkBox_motor_on = QtWidgets.QCheckBox(parent=self.gridLayoutWidget)
        self.checkBox_motor_on.setObjectName("checkBox_motor_on")
        self.layout_motor_control.setWidget(4, QtWidgets.QFormLayout.ItemRole.LabelRole, self.checkBox_motor_on)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.layout_motor_control.setItem(5, QtWidgets.QFormLayout.ItemRole.LabelRole, spacerItem1)
        self.controls.addWidget(self.groupBox_motor_control)
        self.groupBox_sensor_control = QtWidgets.QGroupBox(parent=self.centralwidget)
        self.groupBox_sensor_control.setObjectName("groupBox_sensor_control")
        self.formLayout_3 = QtWidgets.QFormLayout(self.groupBox_sensor_control)
        self.formLayout_3.setObjectName("formLayout_3")
        self.layout_sensor_control = QtWidgets.QFormLayout()
        self.layout_sensor_control.setObjectName("layout_sensor_control")
        self.spinBox_vl53l0x_budget = QtWidgets.QSpinBox(parent=self.groupBox_sensor_control)
        self.spinBox_vl53l0x_budget.setMaximum(300000)
        self.spinBox_vl53l0x_budget.setObjectName("spinBox_vl53l0x_budget")
        self.layout_sensor_control.setWidget(1, QtWidgets.QFormLayout.ItemRole.FieldRole, self.spinBox_vl53l0x_budget)
        self.spinBox_angle_offset = QtWidgets.QSpinBox(parent=self.groupBox_sensor_control)
        self.spinBox_angle_offset.setMinimum(-359)
        self.spinBox_angle_offset.setMaximum(359)
        self.spinBox_angle_offset.setObjectName("spinBox_angle_offset")
        self.layout_sensor_control.setWidget(2, QtWidgets.QFormLayout.ItemRole.FieldRole, self.spinBox_angle_offset)
        self.spinBox_points_per_rev = QtWidgets.QSpinBox(parent=self.groupBox_sensor_control)
        self.spinBox_points_per_rev.setObjectName("spinBox_points_per_rev")
        self.layout_sensor_control.setWidget(0, QtWidgets.QFormLayout.ItemRole.FieldRole, self.spinBox_points_per_rev)
        self.label_points_per_rev = QtWidgets.QLabel(parent=self.groupBox_sensor_control)
        self.label_points_per_rev.setObjectName("label_points_per_rev")
        self.layout_sensor_control.setWidget(0, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_points_per_rev)
        self.label_vl53l0x_budget = QtWidgets.QLabel(parent=self.groupBox_sensor_control)
        self.label_vl53l0x_budget.setObjectName("label_vl53l0x_budget")
        self.layout_sensor_control.setWidget(1, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_vl53l0x_budget)
        self.label_angle_offset = QtWidgets.QLabel(parent=self.groupBox_sensor_control)
        self.label_angle_offset.setObjectName("label_angle_offset")
        self.layout_sensor_control.setWidget(2, QtWidgets.QFormLayout.ItemRole.LabelRole, self.label_angle_offset)
        self.formLayout_3.setLayout(0, QtWidgets.QFormLayout.ItemRole.SpanningRole, self.layout_sensor_control)
        self.controls.addWidget(self.groupBox_sensor_control)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding)
        self.controls.addItem(spacerItem2)
        self.controls.setStretch(0, 1)
        self.controls.setStretch(1, 1)
        self.controls.setStretch(2, 1)
        self.horizontalLayout_2.addLayout(self.controls)
        self.line = QtWidgets.QFrame(parent=self.centralwidget)
        self.line.setFrameShape(QtWidgets.QFrame.Shape.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line.setObjectName("line")
        self.horizontalLayout_2.addWidget(self.line)
        self.graphs = QtWidgets.QVBoxLayout()
        self.graphs.setObjectName("graphs")
        self.plot_rpm = MplCanvas(parent=self.centralwidget)
        self.plot_rpm.setObjectName("plot_rpm")
        self.graphs.addWidget(self.plot_rpm)
        self.rpm_plot_control = QtWidgets.QGridLayout()
        self.rpm_plot_control.setObjectName("rpm_plot_control")
        self.slider_range_rpm_plot = QtWidgets.QSlider(parent=self.centralwidget)
        self.slider_range_rpm_plot.setMinimum(-20000)
        self.slider_range_rpm_plot.setMaximum(0)
        self.slider_range_rpm_plot.setSliderPosition(0)
        self.slider_range_rpm_plot.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.slider_range_rpm_plot.setInvertedAppearance(True)
        self.slider_range_rpm_plot.setInvertedControls(False)
        self.slider_range_rpm_plot.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        self.slider_range_rpm_plot.setTickInterval(1000)
        self.slider_range_rpm_plot.setObjectName("slider_range_rpm_plot")
        self.rpm_plot_control.addWidget(self.slider_range_rpm_plot, 1, 1, 1, 1)
        self.btn_rpm_plot_reset = QtWidgets.QPushButton(parent=self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Maximum, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_rpm_plot_reset.sizePolicy().hasHeightForWidth())
        self.btn_rpm_plot_reset.setSizePolicy(sizePolicy)
        self.btn_rpm_plot_reset.setLayoutDirection(QtCore.Qt.LayoutDirection.LeftToRight)
        self.btn_rpm_plot_reset.setObjectName("btn_rpm_plot_reset")
        self.rpm_plot_control.addWidget(self.btn_rpm_plot_reset, 1, 2, 1, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        self.label = QtWidgets.QLabel(parent=self.centralwidget)
        self.label.setObjectName("label")
        self.rpm_plot_control.addWidget(self.label, 0, 1, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.display_rpm_plot_zoom = QtWidgets.QLCDNumber(parent=self.centralwidget)
        self.display_rpm_plot_zoom.setObjectName("display_rpm_plot_zoom")
        self.rpm_plot_control.addWidget(self.display_rpm_plot_zoom, 1, 0, 1, 1)
        self.graphs.addLayout(self.rpm_plot_control)
        self.line_2 = QtWidgets.QFrame(parent=self.centralwidget)
        self.line_2.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.line_2.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line_2.setObjectName("line_2")
        self.graphs.addWidget(self.line_2)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.graphs.addItem(spacerItem3)
        self.plot_lidar = MplCanvas(parent=self.centralwidget)
        self.plot_lidar.setObjectName("plot_lidar")
        self.graphs.addWidget(self.plot_lidar)
        self.lidar_plot_control = QtWidgets.QGridLayout()
        self.lidar_plot_control.setObjectName("lidar_plot_control")
        self.display_lidar_plot_zoom = QtWidgets.QLCDNumber(parent=self.centralwidget)
        self.display_lidar_plot_zoom.setObjectName("display_lidar_plot_zoom")
        self.lidar_plot_control.addWidget(self.display_lidar_plot_zoom, 1, 0, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.btn_lidar_plot_reset = QtWidgets.QPushButton(parent=self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btn_lidar_plot_reset.sizePolicy().hasHeightForWidth())
        self.btn_lidar_plot_reset.setSizePolicy(sizePolicy)
        self.btn_lidar_plot_reset.setObjectName("btn_lidar_plot_reset")
        self.lidar_plot_control.addWidget(self.btn_lidar_plot_reset, 1, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(parent=self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.lidar_plot_control.addWidget(self.label_2, 0, 1, 1, 1, QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.slider_range_lidar_plot = QtWidgets.QSlider(parent=self.centralwidget)
        self.slider_range_lidar_plot.setMinimum(100)
        self.slider_range_lidar_plot.setMaximum(10000)
        self.slider_range_lidar_plot.setSliderPosition(10000)
        self.slider_range_lidar_plot.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.slider_range_lidar_plot.setInvertedAppearance(False)
        self.slider_range_lidar_plot.setInvertedControls(False)
        self.slider_range_lidar_plot.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        self.slider_range_lidar_plot.setTickInterval(1000)
        self.slider_range_lidar_plot.setObjectName("slider_range_lidar_plot")
        self.lidar_plot_control.addWidget(self.slider_range_lidar_plot, 1, 1, 1, 1)
        self.graphs.addLayout(self.lidar_plot_control)
        self.graphs.setStretch(0, 2)
        self.graphs.setStretch(2, 1)
        self.graphs.setStretch(4, 5)
        self.horizontalLayout_2.addLayout(self.graphs)
        self.horizontalLayout_2.setStretch(0, 1)
        self.horizontalLayout_2.setStretch(2, 2)
        self.gridLayout.addLayout(self.horizontalLayout_2, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(parent=MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1154, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.slider_range_lidar_plot.sliderMoved['int'].connect(self.display_lidar_plot_zoom.display) # type: ignore
        self.slider_range_rpm_plot.sliderMoved['int'].connect(self.display_rpm_plot_zoom.display) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        MainWindow.setTabOrder(self.doubleSpinBox_kp, self.doubleSpinBox_ki)
        MainWindow.setTabOrder(self.doubleSpinBox_ki, self.doubleSpinBox_kd)
        MainWindow.setTabOrder(self.doubleSpinBox_kd, self.spinBox_tgt_rpm)
        MainWindow.setTabOrder(self.spinBox_tgt_rpm, self.checkBox_motor_on)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Pico LiDAR"))
        self.groupBox_general_control.setTitle(_translate("MainWindow", "General"))
        self.btn_read_from_device.setText(_translate("MainWindow", "Read from device"))
        self.btn_send_to_device.setText(_translate("MainWindow", "Send to device"))
        self.btn_reset_device.setText(_translate("MainWindow", "Reset device"))
        self.groupBox_motor_control.setTitle(_translate("MainWindow", "Motor control"))
        self.label_Kp.setText(_translate("MainWindow", "Kp"))
        self.label_Ki.setText(_translate("MainWindow", "Ki"))
        self.label_Kd.setText(_translate("MainWindow", "Kd"))
        self.label_target_rpm.setText(_translate("MainWindow", "Target RPM"))
        self.checkBox_motor_on.setText(_translate("MainWindow", "Motor on"))
        self.groupBox_sensor_control.setTitle(_translate("MainWindow", "Sensor control"))
        self.label_points_per_rev.setText(_translate("MainWindow", "<html><head/><body><p>Points per Revolution</p></body></html>"))
        self.label_vl53l0x_budget.setText(_translate("MainWindow", "VL53L0X Scan Time Budget (us)"))
        self.label_angle_offset.setText(_translate("MainWindow", "Scan Angle Offset (degree)"))
        self.btn_rpm_plot_reset.setText(_translate("MainWindow", "Reset RPM Graph"))
        self.label.setText(_translate("MainWindow", "Max Age (ms), 0 to show everything"))
        self.btn_lidar_plot_reset.setText(_translate("MainWindow", "Reset LiDAR Graph"))
        self.label_2.setText(_translate("MainWindow", "Display Range (mm)"))
from mplcanvas import MplCanvas
