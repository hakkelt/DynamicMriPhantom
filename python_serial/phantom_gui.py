
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from phantom_controller import PhantomController
import numpy as np

DEFAULT_PORT = "/dev/ttyACM0"

def calculate_max_speed(amplitude, move_ms, gamma, microstep):

    move_s = move_ms/1000
    N_points = 100

    times = np.linspace(0,move_s, N_points+1)
    dt = move_s/N_points
    Positions = (1.0 - np.cos(2*np.pi * (times/move_s)**gamma)) * amplitude * microstep / 1.8
    max_speed = np.max(np.abs(np.gradient(Positions))) / dt
    return max_speed

class PhantomGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MRI Phantom Controller")

        self.controller = None

        main = ttk.Frame(root, padding=10)
        main.grid(sticky="nsew")

        ttk.Label(main, text="Serial Port").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        ttk.Entry(main, textvariable=self.port_var, width=20).grid(row=0, column=1, sticky="ew")

        fields = [
            ("Amplitude", "180"),
            ("Move Time (ms)", "2000"),
            ("Dwell Time (ms)", "3000"),
            ("Gamma", "1.0"),
            ("Microstep", "2"),
            ("Cycles", "99"),
        ]

        self.vars = {}
        for i, (label, default) in enumerate(fields, start=1):
            ttk.Label(main, text=label).grid(row=i, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=default)
            ttk.Entry(main, textvariable=var).grid(row=i, column=1, sticky="ew", pady=2)
            self.vars[label] = var

        self.max_speed = 0

        


        btns = ttk.Frame(main)
        btns.grid(row=len(fields)+2, column=0, columnspan=2, pady=10)

        self.vars['MRI_signal'] = tk.BooleanVar()

        ttk.Button(btns, text="Connect", command=self.connect).pack(side="left", padx=5)
        ttk.Checkbutton(btns, text='MRI signal', variable=self.vars['MRI_signal']).pack(side="left", padx=5)
        ttk.Button(btns, text="Start", command=self.start_cycle).pack(side="left", padx=5)
        ttk.Button(btns, text="Stop", command=self.stop_motion).pack(side="left", padx=5)
        ttk.Button(btns, text="Reset", command=self.reset_device).pack(side="left", padx=5)

        self.status = tk.StringVar(value="Not connected")
        ttk.Label(main, textvariable=self.status).grid(row=len(fields)+3, column=0, columnspan=2, sticky="w")

        self.log = scrolledtext.ScrolledText(main, height=10, width=60)
        self.log.grid(row=len(fields)+4, column=0, columnspan=2, pady=10, sticky="nsew")

        main.columnconfigure(1, weight=1)

    def write_log(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def connect(self):
        try:
            self.controller = PhantomController(port=self.port_var.get())
            self.status.set(f"Connected: {self.port_var.get()}")
            self.write_log("Connected successfully.")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def start_cycle(self):
        if not self.controller:
            messagebox.showwarning("Not Connected", "Connect to the Arduino first.")
            return

        try:
            amplitude = float(self.vars["Amplitude"].get())
            move_ms = int(self.vars["Move Time (ms)"].get())
            dwell_ms = int(self.vars["Dwell Time (ms)"].get())
            gamma = float(self.vars["Gamma"].get())
            microstep = int(self.vars["Microstep"].get())
            mri_signal = int(self.vars['MRI_signal'].get())
            cycles = int(self.vars['Cycles'].get())

            self.max_speed = calculate_max_speed(amplitude, move_ms, gamma, microstep)

            self.controller.start_cycle(
                amplitude,
                move_ms,
                dwell_ms,
                gamma,
                microstep,
                mri_signal,
                cycles,
                #self.max_speed,
            )

            
            
            self.write_log(f"Maximum speed: {int(self.max_speed)}")

            if(mri_signal):
                self.write_log(
                    f"ARMED amp={amplitude}, move={move_ms}, dwell={dwell_ms}, gamma={gamma}, ustep={microstep}, mri={bool(mri_signal)}"
                )
            else:
                self.write_log(
                    f"START amp={amplitude}, move={move_ms}, dwell={dwell_ms}, gamma={gamma}, ustep={microstep}, mri={bool(mri_signal)}"
                )

        except Exception as e:
            messagebox.showerror("Error", str(e))

    def stop_motion(self):
        if self.controller:
            self.controller.stop_motion()
            self.write_log("STOP")

    def reset_device(self):
        if self.controller:
            self.controller.reset_to_baseline()
            self.write_log("RESET")

    



if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("640x530")
    app = PhantomGUI(root)
    root.mainloop()
