import tkinter as tk
from tkinter import ttk
import queue
import serial_interface as si
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

target_rpm_lower_limit = 50
target_rpm_upper_limit = 1000

max_data_points = 200

measured_rpm_queue = queue.Queue()
measured_rpm = list()
target_rpm_queue = queue.Queue()
target_rpm = list()

canvas: FigureCanvasTkAgg
fig = Figure(figsize=(5,5), dpi=100)

def create_gui() -> tk.Tk:
    def on_kp_spinbox_change():
        value: float = float(ui_kp.get())
        si.set_kp(value)

    def on_ki_spinbox_change():
        value: float = float(ui_ki.get())
        si.set_ki(value)
    
    def on_kd_spinbox_change():
        value: float = float(ui_kd.get())
        si.set_kd(value)
    
    def on_rpm_spinbox_change():
        value: int = int(ui_rpm.get())
        si.set_target_rpm(value)
        
    def set_spinbox_values(kp: float, ki: float, kd: float, rpm: int):
        ui_kp.set(kp)
        ui_ki.set(ki)
        ui_kd.set(kd)
        ui_rpm.set(rpm)
        
    def on_motor_enable_btn_press():
        if bool(ui_mtr_btn_var.get()):
            print("Starting motor")
            si.start_motor()
        else:
            print("Stopping Motor")
            si.stop_motor()
            
    def on_update_btn_press():
        ui_mtr_btn_var.set(si.get_motor_state())
        set_spinbox_values(si.get_kp(), si.get_ki(), si.get_kd(), si.get_target_rpm())

    window = tk.Tk()
    window.title("PID Tuning GUI")
    spinbox_vertical_spacing = 5
    
    pid_label_frame = ttk.Labelframe(window, text="PID values")
    pid_label_frame.grid(column=0, row=0, padx=20, pady=20)

    label_col = 0
    spinbox_col = 1
    ui_kp_label = ttk.Label(pid_label_frame, text="Kp")
    ui_kp_label.grid(column=label_col, row=0)
    ui_kp = ttk.Spinbox(pid_label_frame, from_=0, to=10, increment=0.01, command=on_kp_spinbox_change)
    ui_kp.grid(column=spinbox_col, row=0, pady=spinbox_vertical_spacing)
    
    ui_ki_label = ttk.Label(pid_label_frame, text="Ki")
    ui_ki_label.grid(column=label_col, row=1)
    ui_ki = ttk.Spinbox(pid_label_frame, from_=0, to=10, increment=0.01, command=on_ki_spinbox_change)
    ui_ki.grid(column=spinbox_col, row=1, pady=spinbox_vertical_spacing)

    ui_kd_label = ttk.Label(pid_label_frame, text="Kd")
    ui_kd_label.grid(column=label_col, row=2)
    ui_kd = ttk.Spinbox(pid_label_frame, from_=0, to=10, increment=0.01, command=on_kd_spinbox_change)
    ui_kd.grid(column=spinbox_col, row=2, pady=spinbox_vertical_spacing)
    
    motor_label_frame = ttk.LabelFrame(window, text="Motor control")
    motor_label_frame.grid(column=0, row=1, padx=20, pady=20)
    
    ui_rpm_label = ttk.Label(motor_label_frame, text="Target RPM")
    ui_rpm_label.grid(column=label_col, row=0)
    ui_rpm = ttk.Spinbox(motor_label_frame, from_=target_rpm_lower_limit, to=target_rpm_upper_limit, increment=10, command=on_rpm_spinbox_change)
    ui_rpm.grid(column=spinbox_col, row=0)
    
    ui_mtr_btn_var = tk.IntVar()
    ui_mtr_btn = ttk.Checkbutton(motor_label_frame, text="Motor on", command=on_motor_enable_btn_press, onvalue=True, offvalue=False, variable=ui_mtr_btn_var)
    ui_mtr_btn_var.set(0)
    ui_mtr_btn.grid(column=0, row=1, pady=20)
    
    ui_update_btn = ttk.Button(window, text="Update from device", command=on_update_btn_press)
    ui_update_btn.grid(column=0, row=2, pady=20)
    
    set_spinbox_values(0,0,0, target_rpm_lower_limit)
    return window

def embed_plots(window: tk.Tk):
    global fig, canvas
    def on_clear_data_btn_press():
        global measured_rpm, target_rpm
        measured_rpm = list()
        target_rpm = list()
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas.draw()
    canvas.get_tk_widget().grid(column=1, row=0)
    
    ui_clear_data_btn = ttk.Button(window, text="Clear plot", command=on_clear_data_btn_press)
    ui_clear_data_btn.grid(column=1, row=1)
    
def get_queued_values(q: queue.Queue, tgt_list: list):
    while q.qsize() > 0:
        ts, val = q.get()
        if len(tgt_list) >= max_data_points:
            tgt_list.pop(0)
        tgt_list.append((ts, val))

def update_plots():
    global measured_rpm_queue, target_rpm_queue, fig, canvas
    # get data
    get_queued_values(measured_rpm_queue, measured_rpm)
    get_queued_values(target_rpm_queue, target_rpm)
    
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

def run_gui():
    running: bool = True
    def on_closing():
        nonlocal running
        running = False
        window.destroy()

    global measured_rpm_queue, target_rpm_queue
    measured_rpm_queue = si.subscribe("measuredRPM", 200)
    target_rpm_queue = si.subscribe("targetRPM", 200)
    window = create_gui()
    embed_plots(window)
    
    window.protocol("WM_DELETE_WINDOW", on_closing)
    while running:
        update_plots()
        window.update_idletasks()
        window.update()