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
    
class ParameterId(Enum):
    NONE = 0,
    KP = 1
    KI = 2
    KD = 3
    TARGET_RPM = 4
    ENABLE_MOTOR = 5
    
CMD_DELIMITER = b'\n'
    
def buildCmdFrame(cmd: CmdInstruction, tgt: ParameterId, value: float | int) -> bytearray:
    bytes = bytearray()
    bytes.append(cmd.value)
    bytes.extend(int(tgt.value).to_bytes(4))
    bytes.extend(value.to_bytes(4))
    bytes.extend(CMD_DELIMITER)
    return bytes

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
    serial_device.write(buildCmdFrame(CmdInstruction.SET, ParameterId.ENABLE_MOTOR, 1))
    print(serial_device.readlines())
    time.sleep(3)
    serial_device.write(buildCmdFrame(CmdInstruction.SET, ParameterId.ENABLE_MOTOR, 0))
    print(serial_device.readlines())
    
if __name__=="__main__":
    main()