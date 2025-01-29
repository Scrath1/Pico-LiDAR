import tkinter as tk
from tkinter import ttk
import queue
import serial_interface as si
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

max_data_points = 200

target_rpm_min = 50

measured_rpm_queue = queue.Queue()
measured_rpm = list()
target_rpm_queue = queue.Queue()
target_rpm = list()

canvas: FigureCanvasTkAgg
fig = Figure(figsize=(5,5), dpi=100)

motor_spinboxes: dict

def _create_gui() -> tk.Tk:
    global motor_spinboxes
    def set_spinbox_values(kp: float, ki: float, kd: float, rpm: int):
        global motor_spinboxes
        motor_spinboxes["Kp"][0].set(kp)
        motor_spinboxes["Ki"][0].set(ki)
        motor_spinboxes["Kd"][0].set(kd)
        motor_spinboxes["Target RPM"][0].set(rpm)
        
    def on_motor_enable_btn_press():
        if bool(ui_mtr_btn_var.get()):
            # print("Starting motor")
            si.start_motor()
        else:
            # print("Stopping Motor")
            si.stop_motor()
            
    def on_update_btn_press():
        ui_mtr_btn_var.set(si.get_motor_state())
        set_spinbox_values(si.get_kp(), si.get_ki(), si.get_kd(), si.get_target_rpm())

    window = tk.Tk()
    window.title("PID Tuning GUI")
    
    motor_control_frame = ttk.Labelframe(window, text="Motor control")
    motor_control_frame.pack()
    # motor_control_frame.grid(column=0, row=0, padx=20, pady=20)

    motor_spinboxes = {"Kp": [ttk.Spinbox(motor_control_frame, from_=0, to=10, increment=0.01), si.set_kp, 0],
                   "Ki": [ttk.Spinbox(motor_control_frame, from_=0, to=10, increment=0.01), si.set_ki, 0],
                   "Kd": [ttk.Spinbox(motor_control_frame, from_=0, to=10, increment=0.01), si.set_kd, 0],
                   "Target RPM": [ttk.Spinbox(motor_control_frame, from_=target_rpm_min, to=1000, increment=10), si.set_target_rpm, target_rpm_min]}

    row = 0
    for k in motor_spinboxes:
        label = ttk.Label(motor_control_frame, text=k, name="label_" + k)
        label.grid(column = 0, row = row)
        motor_spinboxes[k][0].grid(column = 1, row = row)
        row += 1
        
    ui_update_btn = ttk.Button(motor_control_frame, text="Read from device", command=on_update_btn_press)
    ui_update_btn.grid(column=0, row=row)
    
    ui_send_vals_btn = ttk.Button(motor_control_frame, text="Send to device", command=_check_spinbox_changes)
    ui_send_vals_btn.grid(column=1, row=row)
    row += 1
    
    ui_mtr_btn_var = tk.IntVar()
    ui_mtr_btn = ttk.Checkbutton(motor_control_frame, text="Motor on", command=on_motor_enable_btn_press, onvalue=True, offvalue=False, variable=ui_mtr_btn_var)
    ui_mtr_btn_var.set(0)
    ui_mtr_btn.grid(column=0, row=row)

    set_spinbox_values(0,0,0, target_rpm_min)
    
    # Try to get some initial values
    on_update_btn_press()
    return window

def _embed_plots(window: tk.Tk):
    global fig, canvas
    def on_clear_data_btn_press():
        global measured_rpm, target_rpm
        measured_rpm = list()
        target_rpm = list()
    
    ui_clear_data_btn = ttk.Button(window, text="Clear plot", command=on_clear_data_btn_press)
    ui_clear_data_btn.pack()
    
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().pack()
    
def _get_queued_values(q: queue.Queue, tgt_list: list):
    while q.qsize() > 0:
        ts, val = q.get()
        if len(tgt_list) >= max_data_points:
            tgt_list.pop(0)
        tgt_list.append((ts, val))

def _update_plots():
    global measured_rpm_queue, target_rpm_queue, fig, canvas
    # get data
    _get_queued_values(measured_rpm_queue, measured_rpm)
    _get_queued_values(target_rpm_queue, target_rpm)
    
    fig.clear()
    rpm_plot = fig.add_subplot()
    measured_rpm_timestamps = [v[0] for v in measured_rpm]
    measured_rpm_values = [v[1] for v in measured_rpm]
    target_rpm_timestamps = [v[0] for v in target_rpm]
    target_rpm_values = [v[1] for v in target_rpm]
    if len(measured_rpm) > 0:
        rpm_plot.plot(measured_rpm_timestamps, measured_rpm_values, label="measured RPM")
        rpm_plot.plot(target_rpm_timestamps, target_rpm_values, label="target RPM")
        rpm_plot.set_xlabel("Timestamp (ms)")
        rpm_plot.set_ylabel("RPM")
        rpm_plot.legend()
        rpm_plot.grid()
    canvas.draw()
    
def are_almost_equal(a: float, b: float, tolerance=1e-9):
    return abs(a - b) < tolerance
    
def _check_spinbox_changes(dry_run: bool = False):
    # dry run parameter allows updating the spin boxes once for initialization
    # without unnecessarily sending data to MCU
    global motor_spinboxes
    for k in motor_spinboxes:
        val = float(motor_spinboxes[k][0].get())
        if not are_almost_equal(val, motor_spinboxes[k][2]):
            motor_spinboxes[k][2] = val
            if not dry_run:
                setter_func = motor_spinboxes[k][1]
                setter_func(val)

def run_gui():
    running: bool = True
    def on_closing():
        nonlocal running
        running = False
        window.destroy()

    global measured_rpm_queue, target_rpm_queue
    measured_rpm_queue = si.subscribe("measuredRPM", 200)
    target_rpm_queue = si.subscribe("targetRPM", 200)
    window = _create_gui()
    _check_spinbox_changes(dry_run=True)
    _embed_plots(window)
    
    window.protocol("WM_DELETE_WINDOW", on_closing)
    while running:
        _update_plots()
        window.update_idletasks()
        window.update()