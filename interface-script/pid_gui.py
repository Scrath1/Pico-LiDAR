import tkinter as tk
from tkinter import ttk
import serial_interface as si

target_rpm_lower_limit = 50
target_rpm_upper_limit = 1000

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
        
    def spinbox_set_init_values(kp: float, ki: float, kd: float, rpm: int):
        ui_kp.set(kp)
        ui_ki.set(ki)
        ui_kd.set(kd)
        ui_rpm.set(rpm)
        
    def on_motor_enable_btn_press():
        if bool(btn_var.get()):
            print("Starting motor")
            si.start_motor()
        else:
            print("Stopping Motor")
            si.stop_motor()


    window = tk.Tk()
    window.title("PID Tuning GUI")
    spinbox_vertical_spacing = 5
    
    label_frame = ttk.Labelframe(window, text="PID values")
    label_frame.grid(column=0, row=0, padx=20, pady=20)

    label_col = 0
    spinbox_col = 1
    ui_kp_label = ttk.Label(label_frame, text="Kp")
    ui_kp_label.grid(column=label_col, row=0)
    ui_kp = ttk.Spinbox(label_frame, from_=0, to=10, increment=0.01, command=on_kp_spinbox_change)
    ui_kp.grid(column=spinbox_col, row=0, pady=spinbox_vertical_spacing)
    
    ui_ki_label = ttk.Label(label_frame, text="Ki")
    ui_ki_label.grid(column=label_col, row=1)
    ui_ki = ttk.Spinbox(label_frame, from_=0, to=10, increment=0.01, command=on_ki_spinbox_change)
    ui_ki.grid(column=spinbox_col, row=1, pady=spinbox_vertical_spacing)

    ui_kd_label = ttk.Label(label_frame, text="Kd")
    ui_kd_label.grid(column=label_col, row=2)
    ui_kd = ttk.Spinbox(label_frame, from_=0, to=10, increment=0.01, command=on_kd_spinbox_change)
    ui_kd.grid(column=spinbox_col, row=2, pady=spinbox_vertical_spacing)
    
    ui_rpm_label = ttk.Label(label_frame, text="Target RPM")
    ui_rpm_label.grid(column=label_col, row=3)
    ui_rpm = ttk.Spinbox(label_frame, from_=target_rpm_lower_limit, to=target_rpm_upper_limit, increment=10, command=on_rpm_spinbox_change)
    ui_rpm.grid(column=spinbox_col, row=3)
    
    btn_var = tk.IntVar()
    ui_mtr_btn = ttk.Checkbutton(window, text="Motor on", command=on_motor_enable_btn_press, onvalue=True, offvalue=False, variable=btn_var)
    btn_var.set(0)
    ui_mtr_btn.grid(column=0, row=1, pady=20)
    
    spinbox_set_init_values(0,0,0, target_rpm_lower_limit)
    return window

def run_gui():
    window = create_gui()
    window.mainloop()