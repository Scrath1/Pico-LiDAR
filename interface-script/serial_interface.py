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

_serial_device: serial.Serial = serial.Serial()
_cmd_frame_queue = queue.Queue()
_cmd_reply_queue = queue.Queue()

def _buildCmdFrame(cmd: _CmdInstruction, tgt: _ParameterId, value: float | int | None = None) -> bytearray:
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
    bytes.extend(CMD_DELIMITER)
    return bytes

def _enqueueCmdFrame(b: bytearray):
    _cmd_frame_queue.put(b)

def _readSerialThread():
    receiving_reply = False
    reply_buffer: bytearray = bytearray()
    while True:
        if(_serial_device.is_open):
            c = _serial_device.read()
            if ord(c) == 2: # ASCII symbol STX found
                receiving_reply = True
            elif ord(c) == 3:
                receiving_reply = False
                _cmd_reply_queue.put(reply_buffer)
                reply_buffer = bytearray()
            elif receiving_reply:
                reply_buffer.extend(c)
            else:
                print(f"{c.decode()}", end='')

def _writeSerialThread():
    while True:
        if(_serial_device.is_open):
            cmd = _cmd_frame_queue.get()
            _serial_device.write(cmd)
            _cmd_frame_queue.task_done()
            time.sleep(0.01) # sleep 10ms between commands

def init_serial(port: str, baud: int):
    _serial_device.port = port
    _serial_device.baudrate = baud
    _serial_device.open()
    
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

def get_motor_state() -> bool:
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.GET, _ParameterId.ENABLE_MOTOR))
    reply: bytearray = _cmd_reply_queue.get()
    return bool.from_bytes(reply)

def set_target_rpm(val: int):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.TARGET_RPM, val))

def set_kp(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KP, val))
    
def set_ki(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KI, val))

def set_kd(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KD, val))