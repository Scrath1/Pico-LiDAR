from dataclasses import dataclass
import math
from enum import Enum
import queue
import struct
import time
import serial_interface as si
import esp_link as esp
import threading

class _CmdInstruction(Enum):
    NONE = 0
    SET = 1
    GET = 2
    RESET = 3
    
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

CMD_START_DELIM = 0b01010101

@dataclass
class PublishedValue:
    timestamp: int
    value: float | list[float]

    def values_approx_equal(self, other) -> bool:
        if isinstance(self.value, float) and isinstance(other.value, float):
            # 2 float values are easy to compare
            return math.isclose(self.value, other.value)
        elif isinstance(self.value, list) and isinstance(other.value, list):
            # In case of 2 lists, check that the length of both is equal first
            if len(self.value) == len(other.value):
                # Iterate over every item in both lists. While each
                # item at i is equal to the corresponding in other, don't return
                for i in range(0, len(self.value)):
                    if not math.isclose(self.value[i], other.value[i]):
                        return False
                # If there were no unequal items, return true
                return True
            else:
                # Lists have different lengths and must be unequal
                return False
        else:
            # self.value and other.value have different types
            return False

parameter_keys: dict = {_ParameterId.NONE: 'None',
                        _ParameterId.KP: 'K_p',
                        _ParameterId.KI: 'K_i',
                        _ParameterId.KD: 'K_d',
                        _ParameterId.TARGET_RPM: 'targetRPM',
                        _ParameterId.ENABLE_MOTOR: 'enableMotor',
                        _ParameterId.VL53L0X_TIME_BUDGET: 'VL53L0X_budget',
                        _ParameterId.DATAPOINTS_PER_REV: 'scanpoints',
                        _ParameterId.ANGLE_OFFSET: 'angleOffset',
                        _ParameterId.SERIAL_ERROR_COUNTER: 'serialErrorCounter'}

_tx_queue = queue.Queue()
_rx_queue = queue.Queue()
_subscribers = dict(list())
_program_start_time = time.time()

# Used for the getter functions to receive data from the serial read thread
# _ParameterId is used as the key
_internal_queues = dict()

def init_interface(args) -> bool:
    global _internal_queues
    global _tx_queue, _rx_queue
    for id in _ParameterId:
        key = parameter_keys[id]
        _internal_queues[id] = subscribe(key, 5)
    if args.port:
        if not si.init_serial(args.port, args.baud, _tx_queue, _rx_queue):
            return False
    elif args.destination:
        if not esp.init_socket(args.destination, 23, _tx_queue, _rx_queue):
            return False
    processingThread = threading.Thread(target=process)
    processingThread.daemon = True
    processingThread.start()
    return True

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

def process():
    global _rx_queue
    data_point_msg = ""
    receiving_data_point = False
    while True:
        c = _rx_queue.get()
        _rx_queue.task_done()

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
    pv = PublishedValue(timestamp=get_timestamp_ms(), value=value)
    if key not in _subscribers:
        return False
    for q in _subscribers[key]:
        if q.full():
            q.get_nowait()
        q.put(pv)
    return True

def get_published_values(q: queue.Queue) -> list[PublishedValue]:
    out = list()
    while q.qsize() > 0:
        pv: PublishedValue = q.get()
        out.append(pv)
    return out

def _build_cmd_frame(cmd: _CmdInstruction, tgt: _ParameterId, value: float | int = 0) -> bytearray:
    bytes = bytearray()
    bytes.append(CMD_START_DELIM)
    bytes.append(cmd.value)
    if(tgt == _ParameterId.NONE):
        # For some reason the NONE Parameter value is treated as a tuple and is therefore causing problems
        # with the integer cast
        bytes.extend(int(0).to_bytes(4))
    else:
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
    _tx_queue.put(b)

def _request_and_wait(id: _ParameterId, timeout: float = 0.2) -> float:
    cur_timestamp = get_timestamp_ms()
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.GET, id))
    while True:
        try:
            # Search for a result that is newer than the request time
            pv: PublishedValue = _internal_queues[id].get(timeout = timeout)
            if pv.timestamp > cur_timestamp:
                return pv.value
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

def set_motor_state(val: bool):
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.SET, _ParameterId.ENABLE_MOTOR, int(val)))

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
    _enqueueCmdFrame(_build_cmd_frame(_CmdInstruction.RESET, _ParameterId.NONE, int(0)))