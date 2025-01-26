import sys
import argparse
from pid_gui import run_gui
import serial_interface as si
import time

parser = argparse.ArgumentParser(
    prog='pico-lidar companion program'
)

def send_args(args):
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

    # send_args(args)
    # si.init_serial("COM7", args.baud)
    # run_gui()

    si.init_serial(args.port, args.baud)
    send_args(args)
    if(args.pid_gui):
        run_gui()
    else:
        si.start_motor()
        time.sleep(10)
        si.stop_motor()
        time.sleep(0.05)

if __name__=="__main__":
    main()