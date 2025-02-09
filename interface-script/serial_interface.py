import serial
import threading
import time
import queue

_serial_device: serial.Serial = serial.Serial(write_timeout = 0, timeout=1)

run: bool = True

def init_serial(port: str, baud: int, tx_queue: queue.Queue, rx_queue: queue.Queue) -> bool:
    _serial_device.port = port
    _serial_device.baudrate = baud
    try:
        _serial_device.open()
    except serial.SerialException:
        print("Failed to open serial port. Maybe it is already in use?")
        return False
    
    serialRxThread = threading.Thread(target=rx_thread, args=[rx_queue])
    serialRxThread.daemon = True # Marks thread to exit automatically when main thread exits
    serialRxThread.start()
    
    serialTxThread = threading.Thread(target=tx_thread, args=[tx_queue])
    serialTxThread.daemon = True
    serialTxThread.start()
    return True

def rx_thread(rx_queue: queue.Queue):
    while True:
        if(_serial_device.is_open):
            c = ''
            try:
                c = _serial_device.read().decode()
            except serial.SerialException:
                # Try reopening connection
                _serial_device.close()
                _serial_device.open()
                continue
            # read timeout check
            if len(c) == 0:
                try:
                    # Try reopening connection
                    _serial_device.close()
                    _serial_device.open()
                except serial.SerialException:
                    time.sleep(0.1)
            else:
                rx_queue.put(c)

def tx_thread(tx_queue: queue.Queue):
    while True:
        if(_serial_device.is_open):
            d = tx_queue.get()
            write_successful = False
            while not write_successful:
                try:
                    _serial_device.write(d)
                    write_successful = True
                except serial.PortNotOpenError:
                    time.sleep(1)
            tx_queue.task_done()