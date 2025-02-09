import socket
import queue
import time
import threading

_socket = socket.socket()
_host: str = ""
_port: int = 0

def init_socket(hostname: str, port,  tx_queue: queue.Queue, rx_queue: queue.Queue) -> bool:
    global _socket, _hostname, _port
    _host = hostname
    _port = port
    _socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        _socket.connect((_host, _port))
    except TimeoutError:
        print("Failed to connect to host. Is the hostname correct?")
        return False
    rxThread = threading.Thread(target=rx_thread, args=[rx_queue])
    rxThread.daemon = True # Marks thread to exit automatically when main thread exits
    rxThread.start()
    
    txThread = threading.Thread(target=tx_thread, args=[tx_queue])
    txThread.daemon = True
    txThread.start()
    return True

def rx_thread(rx_queue: queue.Queue):
    global _socket, _host, _port
    while True:
        data = _socket.recv(1024)
        if len(data) == 0:
            # reconnect socket
            _socket.connect((_host, _port))
        for c in data:
            rx_queue.put(c)

def tx_thread(tx_queue: queue.Queue):
    global _socket
    while True:
        data = tx_queue.get()
        success: bool = False
        while not success:
            try:
                _socket.sendall(data)
                success = True
            except:
                # Wait for socket to be reopened
                time.sleep(0.5)
        tx_queue.task_done()
