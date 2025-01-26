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
    
parameter_keys: dict = {_ParameterId.KP: 'K_p',
                        _ParameterId.KI: 'K_i',
                        _ParameterId.KD: 'K_d',
                        _ParameterId.TARGET_RPM: 'targetRPM',
                        _ParameterId.ENABLE_MOTOR: 'enableMotor'}
    
CMD_DELIMITER = b'\n'

_serial_device: serial.Serial = serial.Serial()
_cmd_frame_queue = queue.Queue()
_data_points = dict(list())
_data_points_list_len_limit = 200

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

# Source: https://stackoverflow.com/questions/379906/how-do-i-parse-a-string-to-a-float-or-int
def _parse_num(candidate):
    """Parse string to number if possible
    It work equally well with negative and positive numbers, integers and floats.

    Args:
        candidate (str): string to convert

    Returns:
        float | int | None: float or int if possible otherwise None
    """
    try:
        float_value = float(candidate)
    except ValueError:
        return None

    # Optional part if you prefer int to float when decimal part is 0
    if float_value.is_integer():
        return int(float_value)
    # end of the optional part

    return float_value

def _parse_data_point(msg: str, tgt_dict: dict, list_size_limit: int):
    """Parse a data point string of the format 'key: value\n'
    and add it to a dict of lists
    
    Args:
        msg (str): string to parse
        tgt_dict (dict): dictionary containing lists of values for the
            specific keys
    Returns:
        None
    """
    value_delim_pos: int = msg.find(":")
    if value_delim_pos == -1:
        return
    key: str = msg[:value_delim_pos]
    value: int | float = _parse_num(msg[value_delim_pos+1:])
    if key in tgt_dict:
        # Check if list length is at or over size limit
        if len(tgt_dict[key]) >= list_size_limit:
            tgt_dict[key].pop(0)
        tgt_dict[key].append(value)
    else:
        tgt_dict[key] = [value]

def _readSerialThread():
    receiving_data_point = False
    data_point_msg: str = str()
    while True:
        if(_serial_device.is_open):
            c = _serial_device.read().decode()
            if c == ">": # Data point start delimiter
                receiving_data_point = True
            elif c == "\n" and receiving_data_point: # Data point end delimiter
                receiving_data_point = False
                # Parse data point into dict
                # Expected string format: key: value\n
                # The initial '>' char is not stored
                _parse_data_point(data_point_msg, _data_points, _data_points_list_len_limit)
                # reset data_point_msg
                data_point_msg = ""
            elif receiving_data_point:
                # add char to data_point_msg while reicing data point
                data_point_msg += c
            else:
                # Char is probably part of a log message. Just print it
                print(f"{c}", end='')

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

def _get_list_size_in_dict(key: str, dict):
    if key in dict:
        return len(dict[key])
    else:
        return -1
    
def _request_and_wait(id: _ParameterId, timeout = 0.2):
    key = parameter_keys[id]
    list_size = _get_list_size_in_dict(key, _data_points)
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.GET, id))
    
    accumulated_sleep = 0
    while list_size == _get_list_size_in_dict(key, _data_points):
        if accumulated_sleep < timeout:
            time.sleep(0.01)
            accumulated_sleep += 0.01
        else:
            return 0
    return _data_points[key][-1] # Return latest value from list

def start_motor():
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, 1))
    
def stop_motor():
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, 0))

def get_motor_state() -> bool:
    val = _request_and_wait(_ParameterId.ENABLE_MOTOR)
    return bool(val)

def set_target_rpm(val: int):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.TARGET_RPM, val))

def get_target_rpm() -> int:
    val = _request_and_wait(_ParameterId.TARGET_RPM)
    return int(val)

def set_kp(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KP, val))
    
def get_kp() -> float:
    return float(_request_and_wait(_ParameterId.KP))
    
def set_ki(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KI, val))
    
def get_ki() -> float:
    return float(_request_and_wait(_ParameterId.KI))

def set_kd(val: float):
    _enqueueCmdFrame(_buildCmdFrame(_CmdInstruction.SET, _ParameterId.KD, val))
    
def get_kd() -> float:
    return float(_request_and_wait(_ParameterId.KD))