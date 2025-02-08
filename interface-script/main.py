import sys
import argparse
import serial_interface as si
import time
import matplotlib.pyplot as plt
import queue
import numpy as np
import math
import threading
from gui import run_gui

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
    settingArgs.add_argument('-ng', "--no-gui", action='store_true', default=False, help="Open GUI for tuning PID parameters")
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
    
    if(args.no_gui):
        si.start_motor()
        while(True):
            # Loop forever until the process is cancelled
            continue
    else:
        run_gui()

    # Before exiting, stop motor
    si.stop_motor()
    # sleep is necessary so stop message can be sent before program exits
    time.sleep(0.05)

if __name__=="__main__":
    main()