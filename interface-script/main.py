import sys
import argparse
import time
import signal
from gui import run_gui, close_gui
import interface as it

running: bool = True

parser = argparse.ArgumentParser(
    prog='pico-lidar companion program'
)

def sigint_handler(sig, frame):
    global running
    print("Exiting program")
    running = False
    close_gui()

def read_args_file(file: str) -> list[str]:
    out = list()
    with open(file, 'r') as file:
        for line in file:
            s = line.split()
            out += s
    return out

def _send_args(args):
    if(args.kp):
        it.set_kp(args.kp)
    if(args.ki):
        it.set_ki(args.ki)
    if(args.kd):
        it.set_kd(args.kd)
    if(args.rpm):
        it.set_target_rpm(args.arpm)
    if(args.offset_angle):
        it.set_angle_offset(args.offset_angle)
    if(args.scanpoints):
        it.set_datapoints_per_rev(args.scanpoints)
    if(args.timebudget_vl53l0x):
        it.set_vl53l0x_budget(args.timebudget_vl53l0x)

def main():
    parser.add_argument('-f', '--file', action='store', type=str, help="Reads arguments from file instead of the CLI")
    comArgs = parser.add_argument_group("Communication Parameters")
    comArgs.add_argument('-b', '--baud', action='store', type=int, default=115200, help="Baudrate in case of a serial connection")
    comInterfaceArgs = comArgs.add_mutually_exclusive_group(required=True)
    comInterfaceArgs.add_argument('-p', '--port', action='store', type=str, help="Serial port. Mutually exclusive to --destination")
    comInterfaceArgs.add_argument('-d', '--destination', action='store', type=str, help="Hostname or IP-Address of destination. Mutually exclusive to --port")
    mcuArgs = parser.add_argument_group("MCU Parameters")
    mcuArgs.add_argument('-r', '--reset', action='store_true', help="Sends a reset command before anything else happens")
    mcuArgs.add_argument('-kp', action='store', type=float, help="Overwrite PID K_p parameter")
    mcuArgs.add_argument('-ki', action='store', type=float, help="Overwrite PID K_i parameter")
    mcuArgs.add_argument('-kd', action='store', type=float, help="Overwrite PID K_d parameter")
    mcuArgs.add_argument('-rpm', action='store', type=int, help="LiDAR dome rotation speed")
    mcuArgs.add_argument('-o', '--offset-angle', action='store', type=int, help="Offsets the angle of all measurements")
    mcuArgs.add_argument('-s', '--scanpoints', action='store', type=int, help="Number of datapoints to measure per rotation")
    mcuArgs.add_argument('-tv', '--timebudget-vl53l0x', action='store', type=int, help="Time in us the VL53L0X has to get a measurement. This impacts accuracy")
    mcuArgs.add_argument('-ng', "--no-gui", action='store_true', default=False, help="Disable GUI")
    plotArgs = parser.add_argument_group("GUI Plotting")
    plotArgs.add_argument('-a', '--age', action='store', type=int, default=1000, help="Maximum age in ms of points in lidar plot. Set to 0 to keep all measurements")

    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, sigint_handler)

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

    if not it.init_interface(args):
        exit()
    
    if(args.reset):
        it.reset_mcu()
        time.sleep(0.1)

    _send_args(args)
    
    return_code = 0
    if(args.no_gui):
        it.start_motor()
        global running
        while(running):
            # Loop forever until the process is cancelled
            continue
    else:
        return_code = run_gui(args)

    # Before exiting, stop motor
    it.stop_motor()
    # sleep is necessary so stop message can be sent before program exits
    time.sleep(0.05)
    sys.exit(return_code)

if __name__=="__main__":
    main()