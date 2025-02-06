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
VL53L0X_MAX_VALID_DISTANCE_MM = 500
hc_sr04_queue = queue.Queue()
hc_sr04_values = list()
HC_SR04_MAX_VALID_DISTANCE_MM = 4000
fig_id = 0
max_data_age_ms = 0

def read_args_file(file: str) -> list[str]:
    out = list()
    with open(file, 'r') as file:
        for line in file:
            s = line.split()
            out += s
    return out

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
    global vl53l0x_values, hc_sr04_values
    vl53l0x_ts_vals = _get_queued_values(vl53l0x_queue)
    hc_sr04_ts_vals = _get_queued_values(hc_sr04_queue)
    # First process the values. Filter out all values with unknown angle
    vl53l0x_ts_vals = [v for v in vl53l0x_ts_vals if v[1][0]!=-1]
    hc_sr04_ts_vals = [v for v in hc_sr04_ts_vals if v[1][0]!=-1]
    # Next filter out all values with an unrealistic measurement and add new data
    # to existing value list
    vl53l0x_values += [v for v in vl53l0x_ts_vals if v[1][1] < VL53L0X_MAX_VALID_DISTANCE_MM]
    hc_sr04_values += [v for v in hc_sr04_ts_vals if v[1][1] < HC_SR04_MAX_VALID_DISTANCE_MM]
    # Remove all data which is older by max_data_age_s than the newest data points.
    # If age is 0 or less, skip this step
    if max_data_age_ms > 0 and len(vl53l0x_values) > 0:
        cutoff_age_ms = vl53l0x_values[-1][0] - max_data_age_ms
        vl53l0x_values = [v for v in vl53l0x_values if v[0] >= cutoff_age_ms]
    if max_data_age_ms > 0 and len(hc_sr04_values) > 0:
        cutoff_age_ms = hc_sr04_values[-1][0] - max_data_age_ms
        hc_sr04_values = [v for v in hc_sr04_values if v[0] >= cutoff_age_ms]
    
    fig = plt.figure(num=fig_id)
    fig.clear()
    lidar_plot = fig.add_subplot()
    lidar_plot.grid()
    lidar_plot.axhline(y=0, color='r', linestyle='-')
    lidar_plot.axvline(x=0, color='r', linestyle='-')

    if len(vl53l0x_values) < 1 and len(hc_sr04_values) < 1:
        lidar_plot.text(0.5, 0.5, "No data")
    else:
        # calculate x and y coordinates for each [angle, distance] value
        angle_distance_list = [v[1] for v in vl53l0x_values]
        vl53l0x_x_coords = [d * np.cos(math.radians(a)) for [a,d] in angle_distance_list]
        vl53l0x_y_coords = [d * np.sin(math.radians(a)) for [a,d] in angle_distance_list]
        angle_distance_list = [v[1] for v in hc_sr04_values]
        hc_sr04_x_coords = [d * np.cos(math.radians(a)) for [a,d] in angle_distance_list]
        hc_sr04_y_coords = [d * np.sin(math.radians(a)) for [a,d] in angle_distance_list]
        lidar_plot.set_xlabel("X Position (mm)")
        lidar_plot.set_ylabel("Y Position (mm)")
        lidar_plot.scatter(vl53l0x_x_coords, vl53l0x_y_coords, color="r", label="VL53L0X")
        lidar_plot.scatter(hc_sr04_x_coords, hc_sr04_y_coords, color="b", label="HC-SR04")
        lidar_plot.legend()
    plt.show(block=False)
    plt.pause(0.001) # needed to update the gui

def _send_args(args):
    if(args.kp):
        si.set_kp(args.kp)
    if(args.ki):
        si.set_ki(args.ki)
    if(args.kd):
        si.set_kd(args.kd)
    if(args.rpm):
        si.set_target_rpm(args.arpm)
    if(args.offset_angle):
        si.set_angle_offset(args.offset_angle)
    if(args.scanpoints):
        si.set_datapoints_per_rev(args.scanpoints)
    if(args.timebudget_vl53l0x):
        si.set_vl53l0x_budget(args.timebudget_vl53l0x)

def main():
    parser.add_argument('-r', '--reset', action='store_true', help="Sends a reset command before anything else happens")
    parser.add_argument('-f', '--file', action='store', type=str, help="Reads arguments from file instead of the CLI")
    serialArgs = parser.add_argument_group("Serial", "Configuration of the serial interface")
    serialArgs.add_argument('-p', '--port', action='store', type=str, help="Serial port", required=True)
    serialArgs.add_argument('-b', '--baud', action='store', type=int, default=115200, help="Baudrate")
    settingArgs = parser.add_argument_group("Parameters")
    settingArgs.add_argument('-kp', action='store', type=float, help="Overwrite PID K_p parameter")
    settingArgs.add_argument('-ki', action='store', type=float, help="Overwrite PID K_i parameter")
    settingArgs.add_argument('-kd', action='store', type=float, help="Overwrite PID K_d parameter")
    settingArgs.add_argument('-rpm', action='store', type=int, help="LiDAR dome rotation speed")
    settingArgs.add_argument('-o', '--offset-angle', action='store', type=int, help="Offsets the angle of all measurements")
    settingArgs.add_argument('-s', '--scanpoints', action='store', type=int, help="Number of datapoints to measure per rotation")
    settingArgs.add_argument('-tv', '--timebudget-vl53l0x', action='store', type=int, help="Time in us the VL53L0X has to get a measurement. This impacts accuracy")
    settingArgs.add_argument('-g', "--pid-gui", action='store_true', default=False, help="Open GUI for tuning PID parameters")
    plotArgs = parser.add_argument_group("Plotting")
    plotArgs.add_argument('-a', '--age', action='store', type=int, default=1000, help="Maximum age in ms of points in lidar plot. Set to 0 to keep all measurements")

    # Iterate over args to check if a file argument was passed
    args = None
    for idx, x in enumerate(sys.argv):
        if x=='-f' or x=='--file' and idx+1 < len(sys.argv):
            args = parser.parse_args(read_args_file(sys.argv[idx+1]))
    # If no file argument was found and parsed, read the CLI arguments instead
    if args == None:
        args = parser.parse_args(sys.argv[1:])
    
    global max_data_age_ms
    max_data_age_ms = args.age

    if not si.init_serial(args.port, args.baud):
        print("Failed to open serial port. Maybe it is already in use?")
        exit()
    
    if(args.reset):
        print("Resetting MCU")
        si.reset_mcu()
        time.sleep(0.1)

    _send_args(args)
    gui_thread = threading.Thread(target=run_gui)
    if(args.pid_gui):
        gui_thread.start()
    else:
        si.start_motor()

    global vl53l0x_queue, hc_sr04_queue
    vl53l0x_queue = si.subscribe("VL53L0X")
    hc_sr04_queue = si.subscribe("HC-SR04")
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