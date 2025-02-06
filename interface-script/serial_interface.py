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
    VL53L0X_TIME_BUDGET = 6
    DATAPOINTS_PER_REV = 7
    ANGLE_OFFSET = 8
    SERIAL_ERROR_COUNTER = 9
    RESET = 10
    
CMD_START_DELIM = 0b01010101
    
parameter_keys: dict = {_ParameterId.NONE: 'None',
                        _ParameterId.KP: 'K_p',
                        _ParameterId.KI: 'K_i',
                        _ParameterId.KD: 'K_d',
                        _ParameterId.TARGET_RPM: 'targetRPM',
                        _ParameterId.ENABLE_MOTOR: 'enableMotor',
                        _ParameterId.VL53L0X_TIME_BUDGET: 'VL53L0X_budget',
                        _ParameterId.DATAPOINTS_PER_REV: 'scanpoints',
                        _ParameterId.ANGLE_OFFSET: 'angleOffset',
                        _ParameterId.SERIAL_ERROR_COUNTER: 'serialErrorCounter',
                        _ParameterId.RESET: 'Reset'}

_serial_device: serial.Serial = serial.Serial(write_timeout = 0, timeout=1)
_cmd_frame_queue = queue.Queue()
_subscribers = dict(list())
_program_start_time = time.time()

# Used for the getter functions to receive data from the serial read thread
# _ParameterId is used as the key
_internal_queues = dict()

def subscribe(key: str, max_size: int = 0) -> queue:
    """Subscribe to data messages with a specific key
    When a message is received, it's parsed value is put into the
    queue returned by this function as tuple (timestamp, value)
    
    Args:
        key (str): The data point key for which messages are
            wanted.
        max_size (int): Maximum size of queue. If the queue is full and new
            data is available, the oldest data point is automatically removed
    
    """
    q = queue.Queue(maxsize = max_size)
    if key in _subscribers:
        _subscribers[key].append(q)
    else:
        _subscribers[key] = [q]
    return q

def _parse_key_values(msg: str) -> tuple[str, float | list[float]]:
    """Attempts to parse a string of format 'key:value0:value1:...:value_n|*'
        The string is parsed into a tu0ple with the first index being the key
        followed by either a float or a list of floats depending on the
        number of values in the string.
        String parts after the '|' symbol in the msg string are ignored.
    Returns
        Tuple containing (key, value|values)
        or (<empty str>, 0) in case of failure
    """
    vals = list()
    substrs = msg.split(":")
    if len(substrs) < 2:
        return ('', 0)
    key = substrs[0]
    for v in substrs[1:]:
        end_pos = v.find("|")
        if end_pos == -1:
            end_pos = len(v)
        try:
            vals.append(float(v[:end_pos]))
        except ValueError:
            return ('', 0)
    if len(vals) == 1:
        return (key, vals[0])
    else:
        return (key, vals)

def get_timestamp_ms() -> int:
    global _program_start_time
    return round((time.time() - _program_start_time) * 1000)

def _publish(key: str, value: float | list[float]) -> bool:
    if key not in _subscribers:
        return False
    for q in _subscribers[key]:
        if q.full():
            q.get_nowait()
        q.put((get_timestamp_ms(), value))
    return True

def _build_cmd_frame(cmd: _CmdInstruction, tgt: _ParameterId, value: float | int = 0) -> bytearray:
    bytes = bytearray()
    bytes.append(CMD_START_DELIM)
    bytes.append(cmd.value)
    bytes.extend(int(tgt.value).to_bytes(4))
    if isinstance(value, int):
        if value >= 0:
            bytes.extend(value.to_bytes(4))
        else:
            bytes.extend(value.to_bytes(4, signed=True))
    elif isinstance(value, float):
        b = bytearray(struct.pack("!f", value))
        if len(b) == 4:
            bytes.extend(b)
        else:
            bytes.extend(int(0).to_bytes(4))
    return bytes

def _enqueueCmdFrame(b: bytearray):
    _cmd_frame_queue.put(b)

def _readSerialThread():
    receiving_data_point = False
    data_point_msg: str = str()
    while True:
        if(_serial_device.is_open):
            c = _serial_device.read().decode()
            
            # read timeout check
            if len(c) == 0:
                try:
                    # Try reopening connection
                    _serial_device.close()
                    _serial_device.open()
                except serial.SerialException:
                    time.sleep(0.1)
            else:
                if c == ">": # Data point start delimiter
                    receiving_data_point = True
                elif c == "\n" and receiving_data_point: # Data point end delimiter
                    receiving_data_point = False
                    # Parse data point into dict
                    # Expected string format: key: value\n
                    # The initial '>' char is not stored
                    k,v = _parse_key_values(data_point_msg)
                    if len(k) > 0:
                        # Publish to subscriber queues
                        _publish(k,v)
                    # reset data_point_msg
                    data_point_msg = ""
                elif receiving_data_point:
                    # add char to data_point_msg while reicing data point
                    data_point_msg += c
                else:
                    # Char is probably part of a log message. Just print it
                    print(f"{c}", end='')
        else:
            try:
                _serial_device.open()
            except serial.SerialException:
                time.sleep(0.5)

def _writeSerialThread():
    while True:
        if(_serial_device.is_open):
            cmd = _cmd_frame_queue.get()
            write_successful = False
            while not write_successful:
                try:
                    _serial_device.write(cmd)
                    write_successful = True
                except serial.PortNotOpenError:
                    time.sleep(1)
            _cmd_frame_queue.task_done()
            time.sleep(0.01) # sleep 10ms between commands

def init_serial(port: str, baud: int) -> bool:
    global _internal_queues
    for id in _ParameterId:
        key = parameter_keys[id]
        _internal_queues[id] = subscribe(key, 5)

    _serial_device.port = port
    _serial_device.baudrate = baud
    try:
        _serial_device.open()
    except serial.SerialException:
        return False
    
    
    serialRxThread = threading.Thread(target=_readSerialThread)
    serialRxThread.daemon = True # Marks thread to exit automatically when main thread exits
    serialRxThread.start()
    
    serialTxThread = threading.Thread(target=_writeSerialThread)
    serialTxThread.daemon = True
    serialTxThread.start()
    return True

def _request_and_wait(id: _ParameterId, timeout: float = 0.2) -> float:
    cur_timestamp = get_timestamp_ms()
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.GET, id))
    while True:
        try:
            # Search for a result that is newer than the request time
            ts, val = _internal_queues[id].get(timeout = timeout)
            if ts > cur_timestamp:
                return val
        except queue.Empty:
            # If no result was found after the queue timed out once, return 0
            return 0

def get_target_rpm() -> int:
    return int(_request_and_wait(_ParameterId.TARGET_RPM))

def set_kp(val: float):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.KP, val))
    
def get_kp() -> float:
    return float(_request_and_wait(_ParameterId.KP))
    
def set_ki(val: float):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.KI, val))
    
def get_ki() -> float:
    return float(_request_and_wait(_ParameterId.KI))

def set_kd(val: float):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.KD, val))
    
def get_kd() -> float:
    return float(_request_and_wait(_ParameterId.KD))

def set_target_rpm(val: int | float):
    intval: int = 0
    if isinstance(val, int):
        intval = val
    else:
        intval = int(val)
    cmd_frame = _build_cmd_frame(_CmdInstruction.SET, _ParameterId.TARGET_RPM, intval)
    _enqueueCmdFrame(cmd_frame)

def start_motor():
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, 1))

def stop_motor():
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, 0))

def get_motor_state() -> bool:
    return bool(_request_and_wait(_ParameterId.ENABLE_MOTOR))

def set_vl53l0x_budget(val_us: int):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.VL53L0X_TIME_BUDGET, val_us))
    
def get_vl53l0x_budget() -> int:
    return int(_request_and_wait(_ParameterId.VL53L0X_TIME_BUDGET))

def set_datapoints_per_rev(val: int):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.DATAPOINTS_PER_REV, val))
    
def get_datapoints_per_rev() -> int:
    return int(_request_and_wait(_ParameterId.DATAPOINTS_PER_REV))

def set_angle_offset(val: int):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.ANGLE_OFFSET, val))
    
def get_angle_offset() -> int:
    return int(_request_and_wait(_ParameterId.ANGLE_OFFSET))

def get_serial_error_counter() -> int:
    return int(_request_and_wait(_ParameterId.SERIAL_ERROR_COUNTER))

def reset_mcu():
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.RESET, int(1)))