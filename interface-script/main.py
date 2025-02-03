import sys
import argparse
from pid_gui import run_gui, exit_gui
import serial_interface as si
import time
import matplotlib.pyplot as plt
import queue
import numpy as np
import math
import threading

parser = argparse.ArgumentParser(
    prog='pico-lidar companion program'
)

vl53l0x_queue = queue.Queue()
vl53l0x_values = list()
VL53L0X_MAX_VALID_DISTANCE = 3000
fig_id = 0
max_data_age_ms = 0

def _get_queued_values(q: queue) -> list:
    out = list()
    while True:
        try:
            ts, vals = q.get_nowait()
            out.append((ts, vals))
        except queue.Empty:
            return out

def _create_lidar_plot():
    plt.figure(num=fig_id)
    plt.grid(True)

def _update_lidar_plot():
    global vl53l0x_values
    vl53l0x_ts_vals = _get_queued_values(vl53l0x_queue)
    # First process the values. Filter out all values with unknown angle
    vl53l0x_ts_vals = [v for v in vl53l0x_ts_vals if v[1][0]!=-1]
    # Next filter out all values with an unrealistic measurement and add new data
    # to existing value list
    vl53l0x_values += [v for v in vl53l0x_ts_vals if v[1][1] < VL53L0X_MAX_VALID_DISTANCE]
    
    # Remove all data which is older by max_data_age_s than the newest data points.
    # If age is 0 or less, skip this step
    if max_data_age_ms > 0 and len(vl53l0x_values) > 0:
        cutoff_age_ms = vl53l0x_values[-1][0] - max_data_age_ms
        vl53l0x_values = [v for v in vl53l0x_values if v[0] >= cutoff_age_ms]
    
    fig = plt.figure(num=fig_id)
    fig.clear()
    lidar_plot = fig.add_subplot()
    lidar_plot.grid()
    lidar_plot.axhline(y=0, color='r', linestyle='-')
    lidar_plot.axvline(x=0, color='r', linestyle='-')

    if len(vl53l0x_values) < 1:
        lidar_plot.text(0.5, 0.5, "No data")
    else:
        # calculate x and y coordinates for each [angle, distance] value
        angle_distance_list = [v[1] for v in vl53l0x_values]
        x_coords = [d * np.cos(math.radians(a)) for [a,d] in angle_distance_list]
        y_coords = [d * np.sin(math.radians(a)) for [a,d] in angle_distance_list]
        lidar_plot.set_xlabel("X Position (mm)")
        lidar_plot.set_ylabel("Y Position (mm)")
        lidar_plot.scatter(x_coords, y_coords)
    plt.show(block=False)
    plt.pause(0.001) # needed to update the gui

def _send_args(args):
    if(args.rpm):
        si.set_target_rpm(args.arpm)
    if(args.kp):
        si.set_kp(args.kp)
    if(args.ki):
        si.set_ki(args.ki)
    if(args.kd):
        si.set_kd(args.kd)

def main():
    serialArgs = parser.add_argument_group("Serial", "Configuration of the serial interface")
    serialArgs.add_argument('-p', '--port', action='store', type=str, help="Serial port", required=True)
    serialArgs.add_argument('-b', '--baud', action='store', type=int, default=115200, help="Baudrate")
    settingArgs = parser.add_argument_group("Parameters")
    settingArgs.add_argument('-kp', action='store', type=float, help="Overwrite PID K_p parameter")
    settingArgs.add_argument('-ki', action='store', type=float, help="Overwrite PID K_i parameter")
    settingArgs.add_argument('-kd', action='store', type=float, help="Overwrite PID K_d parameter")
    settingArgs.add_argument('-rpm', action='store', type=int, help="LiDAR dome rotation speed")
    settingArgs.add_argument('-g', "--pid-gui", action='store_true', default=False, help="Open GUI for tuning PID parameters")
    plotArgs = parser.add_argument_group("Plotting")
    plotArgs.add_argument('-a', '--age', action='store', type=int, default=1000, help="Maximum age in ms of points in lidar plot. Set to 0 to keep all measurements")

    args = parser.parse_args(sys.argv[1:])
    global max_data_age_ms
    max_data_age_ms = args.age

    if not si.init_serial(args.port, args.baud):
        print("Failed to open serial port. Maybe it is already in use?")
        exit()

    _send_args(args)
    gui_thread = threading.Thread(target=run_gui)
    if(args.pid_gui):
        gui_thread.start()
    else:
        si.start_motor()

    global vl53l0x_queue
    vl53l0x_queue = si.subscribe("VL53L0X")
    _create_lidar_plot()

    while plt.fignum_exists(fig_id):
        _update_lidar_plot()
        time.sleep(0.1)
    
    exit_gui()
    gui_thread.join()
    # Before exiting, stop motor
    si.stop_motor()
    # sleep is necessary so stop message can be sent before program exits
    time.sleep(0.05)

if __name__=="__main__":
    main()