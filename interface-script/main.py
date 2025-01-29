import sys
import argparse
from pid_gui import run_gui
import serial_interface as si
import time
import matplotlib.pyplot as plt
import queue
import numpy as np
import math

parser = argparse.ArgumentParser(
    prog='pico-lidar companion program'
)

MAX_DATA_POINTS = 200

vl53l0x_queue = queue.Queue()
vl53l0x_values = list()
VL53L0X_MAX_VALID_DISTANCE = 3000
fig_id = 0

def _get_queued_values(q: queue) -> list:
    out = list()
    while True:
        try:
            ts, vals = q.get_nowait()
            out.append(vals)
        except queue.Empty:
            return out

def _create_lidar_plot():
    plt.figure(num=fig_id)
    plt.grid(True)
    
def _update_lidar_plot():
    global vl53l0x_values
    vl53l0x_values += _get_queued_values(vl53l0x_queue)
    # First process the values. Filter out all values with unknown angle
    vl53l0x_values = [v for v in vl53l0x_values if v[0]!=-1]
    # Next filter out all values with an unrealistic measurement
    vl53l0x_values = [v for v in vl53l0x_values if v[1] < VL53L0X_MAX_VALID_DISTANCE]
    while len(vl53l0x_values) > MAX_DATA_POINTS:
        vl53l0x_values.pop(0)
    if len(vl53l0x_values) < 1:
        return
    # calculate x and y coordinates for each [angle, distance] value
    x_coords = [d * np.cos(math.radians(a)) for [a,d] in vl53l0x_values]
    y_coords = [d * np.sin(math.radians(a)) for [a,d] in vl53l0x_values]
    
    plt.figure(num=fig_id).clear()
    plt.scatter(x_coords, y_coords)
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

    args = parser.parse_args(sys.argv[1:])

    if not si.init_serial(args.port, args.baud):
        print("Failed to open serial port. Maybe it is already in use?")
        exit()

    _send_args(args)
    if(args.pid_gui):
        run_gui()
    else:
        si.start_motor()
        global vl53l0x_queue
        vl53l0x_queue = si.subscribe("VL53L0X")
        _create_lidar_plot()
        
        while plt.fignum_exists(fig_id):
            _update_lidar_plot()
            time.sleep(0.1)
        si.stop_motor()
        # sleep is necessary so stop message can be sent before program exits
        time.sleep(0.05)

if __name__=="__main__":
    main()