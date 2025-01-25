import serial
from enum import Enum
import threading
import queue
import struct
import time

class _CmdInstruction(Enum):
    NONE = 0
    SET = 1
    GET = 2
    
class _ParameterId(Enum):
    NONE = 0,
    KP = 1
    KI = 2
    KD = 3
    TARGET_RPM = 4
    ENABLE_MOTOR = 5
    
CMD_DELIMITER = b'\n'
    
def _buildCmdFrame(cmd: _CmdInstruction, tgt: _ParameterId, value: float | int) -> bytearray:
    bytes = bytearray()
    bytes.append(cmd.value)
    bytes.extend(int(tgt.value).to_bytes(4))
    if isinstance(value, int):
        bytes.extend(value.to_bytes(4))
    elif isinstance(value, float):
        b = bytearray(struct.pack("!f", value))
        if len(b) == 4:
            bytes.extend(b)
        else:
            bytes.extend(int(0).to_bytes(4))
    else:
        raise TypeError(f"Invalid value parameter {type(value)}")
    bytes.extend(CMD_DELIMITER)
    return bytes

serial_device: serial.Serial = serial.Serial()
cmdFrameQueue = queue.Queue()

def _enqueueCmdFrame(b: bytearray):
    cmdFrameQueue.put(b)

def _readSerialThread():
    while True:
        if(serial_device.is_open):
            print(serial_device.readline())
        
def _writeSerialThread():
    while True:
        if(serial_device.is_open):
            cmd = cmdFrameQueue.get()
            serial_device.write(cmd)
            cmdFrameQueue.task_done()
            time.sleep(0.01) # sleep 10ms between commands

def init_serial(port: str, baud: int):
    serial_device.port = port
    serial_device.baudrate = baud
    serial_device.open()
    
    serialRxThread = threading.Thread(target=_readSerialThread)
    serialRxThread.daemon = True # Marks thread to exit automatically when main thread exits
    serialRxThread.start()
    
    serialTxThread = threading.Thread(target=_writeSerialThread)
    serialTxThread.daemon = True
    serialTxThread.start()
    
def start_motor():
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, 1))
    
def stop_motor():
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, 0))
    
def set_target_rpm(val: int):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.TARGET_RPM, val))

def set_kp(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KP, val))
    
def set_ki(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KI, val))

def set_kd(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KD, val))