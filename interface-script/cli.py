import sys
import argparse
import serial
# import commands
import time
from enum import Enum

class CmdInstruction(Enum):
    NONE = 0
    SET = 1
    GET = 2

serial_device: serial.Serial = serial.Serial()
parser = argparse.ArgumentParser(
    prog='pico-lidar companion program'
)

def main():
    serialArgs = parser.add_argument_group("Serial", "Configuration of the serial interface")
    serialArgs.add_argument('-p', '--port', action='store', type=str, help="Serial port", required=True)
    serialArgs.add_argument('-b', '--baud', action='store', type=int, default=115200, help="Baudrate")
    settingArgs = parser.add_argument_group("Parameters")
    settingArgs.add_argument('-kp', action='store', type=float, help="Overwrite PID K_p parameter")
    settingArgs.add_argument('-ki', action='store', type=float, help="Overwrite PID K_i parameter")
    settingArgs.add_argument('-kd', action='store', type=float, help="Overwrite PID K_d parameter")
    settingArgs.add_argument('-rpm', action='store', type=int, default=300, help="LiDAR dome rotation speed")
    args = parser.parse_args(sys.argv[1:])
    print(args)
    
    serial_device.port = args.port
    serial_device.baudrate = args.baud
    serial_device.timeout = 1
    serial_device.open()
    frame0: bytearray = b"\x01\x00\x00\x00\x05\x00\x00\x00\x01\n"
    frame1: bytearray = b"\x01\x00\x00\x00\x05\x00\x00\x00\x00\n"
    serial_device.write(frame0)
    print(serial_device.readlines())
    time.sleep(3)
    serial_device.write(frame1)
    print(serial_device.readlines())
    # while(True):
    #     line: str = serial_device.readline()
    #     print(line)
    
if __name__=="__main__":
    main()