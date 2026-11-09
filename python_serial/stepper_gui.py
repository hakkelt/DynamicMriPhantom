import sys
import os
import time
import math
import csv
import queue
import threading
from pathlib import Path

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class StepperControlGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Arduino Stepper Trajectory Controller")
        self.geometry("820x780")
        self.minsize(780, 700)

        # Communication Variables
        self.ser = None
        self.is_connected = False
        self.comm_queue = queue.Queue()

        # UI State Variables
        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        self.baud_var = tk.StringVar(value="115200")
        self.wait_mri_var = tk.BooleanVar(value=False)

        # Trajectory Parameters
        self.move_time_var = tk.StringVar(value="800")
        self.dwell_time_var = tk.StringVar(value="200")
        self.amplitude_var = tk.StringVar(value="90")
        self.gamma_var = tk.StringVar(value="1.0")
        self.microstep_var = tk.StringVar(value="2")
        self.save_filename_var = tk.StringVar(value="trajectory.csv")

        # Pregenerated File Variable
        self.selected_file_var = tk.StringVar()

        # Manual Move Variables
        self.steps_var = tk.StringVar(value="100")

        self.num_points = 1024

        self._build_ui()
        self._refresh_ports()
        self._scan_csv_files()
        self.after(100, self._process_queue)

    # ============================================================
    # UI INITIALIZATION
    # ============================================================
    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")

        main_frame = ttk.Frame(self, padding=12)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Header
        header_lbl = ttk.Label(main_frame, text="Stepper Trajectory & Motor Control", font=("Helvetica", 16, "bold"))
        header_lbl.pack(anchor=tk.W, pady=(0, 10))

        # --------------------------------------------------------
        # SECTION 1: Serial Connection
        # --------------------------------------------------------
        conn_frame = ttk.LabelFrame(main_frame, text="Serial Connection", padding=8)
        conn_frame.pack(fill=tk.X, pady=4)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=4)
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, padx=4)

        ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports, width=8).grid(row=0, column=2, padx=4)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, sticky=tk.W, padx=(12, 4))
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "57600", "115200"], width=10)
        baud_combo.grid(row=0, column=4, padx=4)

        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection, width=12)
        self.btn_connect.grid(row=0, column=5, padx=12)

        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", foreground="red", font=("Helvetica", 10, "bold"))
        self.lbl_status.grid(row=0, column=6, padx=8)

        # --------------------------------------------------------
        # SECTION 2: Trigger Options
        # --------------------------------------------------------
        trig_frame = ttk.LabelFrame(main_frame, text="Trigger Settings", padding=8)
        trig_frame.pack(fill=tk.X, pady=4)

        chk_mri = ttk.Checkbutton(
            trig_frame,
            text="Wait for MRI signal / Trigger button before starting trajectory",
            variable=self.wait_mri_var,
            command=self._on_mri_toggle
        )
        chk_mri.pack(anchor=tk.W, padx=4)

        # --------------------------------------------------------
        # SECTION 3: Trajectory Generator
        # --------------------------------------------------------
        gen_frame = ttk.LabelFrame(main_frame, text="Generate Trajectory Parameters", padding=8)
        gen_frame.pack(fill=tk.X, pady=4)

        # Inputs Grid
        ttk.Label(gen_frame, text="Move Time (ms):").grid(row=0, column=0, sticky=tk.W, padx=4, pady=2)
        ttk.Entry(gen_frame, textvariable=self.move_time_var, width=10).grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(gen_frame, text="Dwell Time (ms):").grid(row=0, column=2, sticky=tk.W, padx=(16, 4), pady=2)
        ttk.Entry(gen_frame, textvariable=self.dwell_time_var, width=10).grid(row=0, column=3, padx=4, pady=2)

        ttk.Label(gen_frame, text="Amplitude:").grid(row=1, column=0, sticky=tk.W, padx=4, pady=2)
        ttk.Entry(gen_frame, textvariable=self.amplitude_var, width=10).grid(row=1, column=1, padx=4, pady=2)

        ttk.Label(gen_frame, text="Gamma:").grid(row=1, column=2, sticky=tk.W, padx=(16, 4), pady=2)
        ttk.Entry(gen_frame, textvariable=self.gamma_var, width=10).grid(row=1, column=3, padx=4, pady=2)

        ttk.Label(gen_frame, text="Save Filename:").grid(row=2, column=0, sticky=tk.W, padx=4, pady=2)
        ttk.Entry(gen_frame, textvariable=self.save_filename_var, width=18).grid(row=2, column=1, columnspan=2, sticky=tk.W, padx=4, pady=2)

        gen_btn_frame = ttk.Frame(gen_frame)
        gen_btn_frame.grid(row=2, column=3, columnspan=2, sticky=tk.E, padx=4, pady=2)

        ttk.Button(gen_btn_frame, text="Save CSV Only", command=self._action_generate_and_save).pack(side=tk.LEFT, padx=2)
        ttk.Button(gen_btn_frame, text="Save & Send to Arduino", command=self._action_generate_save_send).pack(side=tk.LEFT, padx=2)

        # --------------------------------------------------------
        # SECTION 4: Pregenerated CSV File Selector
        # --------------------------------------------------------
        file_frame = ttk.LabelFrame(main_frame, text="Pregenerated CSV File Selector", padding=8)
        file_frame.pack(fill=tk.X, pady=4)

        ttk.Label(file_frame, text="CSV File:").grid(row=0, column=0, sticky=tk.W, padx=4)
        self.file_combo = ttk.Combobox(file_frame, textvariable=self.selected_file_var, width=32)
        self.file_combo.grid(row=0, column=1, padx=4)

        ttk.Button(file_frame, text="Refresh List", command=self._scan_csv_files).grid(row=0, column=2, padx=4)
        ttk.Button(file_frame, text="Browse...", command=self._browse_file).grid(row=0, column=3, padx=4)
        ttk.Button(file_frame, text="Send File to Arduino", command=self._action_send_selected_file).grid(row=0, column=4, padx=8)

        # --------------------------------------------------------
        # SECTION 5: Manual Motor Control
        # --------------------------------------------------------
        motor_frame = ttk.LabelFrame(main_frame, text="Manual Motor Control", padding=8)
        motor_frame.pack(fill=tk.X, pady=4)

        ttk.Button(motor_frame, text="Reset Motor to Zero", command=self._action_reset_zero).grid(row=0, column=0, padx=6, pady=4)
        ttk.Button(motor_frame, text="Set Current Pos as Zero", command=self._action_set_zero).grid(row=0, column=1, padx=6, pady=4)

        ttk.Label(motor_frame, text="Steps:").grid(row=0, column=2, sticky=tk.W, padx=(16, 4))
        ttk.Entry(motor_frame, textvariable=self.steps_var, width=8).grid(row=0, column=3, padx=4)

        ttk.Button(motor_frame, text="Move +Steps", command=lambda: self._action_move_steps(1)).grid(row=0, column=4, padx=4)
        ttk.Button(motor_frame, text="Move -Steps", command=lambda: self._action_move_steps(-1)).grid(row=0, column=5, padx=4)
        ttk.Button(motor_frame, text="STOP", command=self._action_stop).grid(row=0, column=6, padx=(12, 4))

        # --------------------------------------------------------
        # SECTION 6: Output Console Log
        # --------------------------------------------------------
        log_frame = ttk.LabelFrame(main_frame, text="Activity & Serial Log", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True, pady=4)

        self.txt_log = scrolledtext.ScrolledText(log_frame, height=10, state=tk.DISABLED, font=("Consolas", 9))
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        btn_clear = ttk.Button(log_frame, text="Clear Log", command=self._clear_log)
        btn_clear.pack(anchor=tk.E, pady=(4, 0))

    # ============================================================
    # HELPER & LOGGING METHODS
    # ============================================================
    def log(self, msg):
        self.comm_queue.put(("LOG", msg))

    def _process_queue(self):
        try:
            while True:
                msg_type, payload = self.comm_queue.get_nowait()
                if msg_type == "LOG":
                    self.txt_log.config(state=tk.NORMAL)
                    timestamp = time.strftime("[%H:%M:%S] ")
                    self.txt_log.insert(tk.END, timestamp + payload + "\n")
                    self.txt_log.see(tk.END)
                    self.txt_log.config(state=tk.DISABLED)
                elif msg_type == "STATUS":
                    is_conn, text = payload
                    self.is_connected = is_conn
                    if is_conn:
                        self.lbl_status.config(text=text, foreground="green")
                        self.btn_connect.config(text="Disconnect")
                    else:
                        self.lbl_status.config(text=text, foreground="red")
                        self.btn_connect.config(text="Connect")
        except queue.Empty:
            pass
        self.after(100, self._process_queue)

    def _clear_log(self):
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.delete(1.0, tk.END)
        self.txt_log.config(state=tk.DISABLED)

    def _refresh_ports(self):
        ports = []
        if serial and hasattr(serial.tools, "list_ports"):
            ports = [p.device for p in serial.tools.list_ports.comports()]
        if "/dev/ttyACM0" not in ports:
            ports.insert(0, "/dev/ttyACM0")
        self.port_combo['values'] = ports
        if not self.port_var.get():
            self.port_var.set("/dev/ttyACM0")

    def _scan_csv_files(self):
        current_dir = Path(".")
        csv_files = [f.name for f in current_dir.glob("*.csv")]
        self.file_combo['values'] = csv_files
        if csv_files and not self.selected_file_var.get():
            self.selected_file_var.set(csv_files[0])

    def _browse_file(self):
        filename = filedialog.askopenfilename(
            initialdir=".",
            title="Select Pregenerated Trajectory CSV",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
        if filename:
            self.selected_file_var.set(filename)

    # ============================================================
    # SERIAL COMMUNICATION THREADING
    # ============================================================
    def _toggle_connection(self):
        if self.is_connected:
            self._disconnect()
        else:
            threading.Thread(target=self._connect_thread, daemon=True).start()

    def _connect_thread(self):
        if serial is None:
            self.log("ERROR: 'pyserial' module is not installed!")
            return

        port = self.port_var.get().strip()
        baud = int(self.baud_var.get().strip())

        self.log(f"Connecting to {port} at {baud} baud...")
        try:
            self.ser = serial.Serial(port, baud, timeout=1.5)
            time.sleep(2.0) # Allow Nano auto-reset to settle
            self.ser.reset_input_buffer()

            # Send HELLO handshake
            handshake_success = False
            for _ in range(3):
                self.ser.write(b"HELLO\n")
                self.ser.flush()
                resp = self.ser.readline().decode('ascii', errors='ignore').strip()
                if resp == "READY":
                    handshake_success = True
                    break
                time.sleep(0.3)

            if handshake_success:
                self.log(f"Connected to Arduino on {port}")
                self.comm_queue.put(("STATUS", (True, "Connected")))
                self._send_wait_mri_command(self.wait_mri_var.get())
            else:
                self.log("Connection failed: Arduino did not respond with READY")
                self.ser.close()
                self.ser = None
                self.comm_queue.put(("STATUS", (False, "Failed")))
        except Exception as e:
            self.log(f"Serial Connection Error: {e}")
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None
            self.comm_queue.put(("STATUS", (False, "Error")))

    def _disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.log("Disconnected from serial port.")
        self.comm_queue.put(("STATUS", (False, "Disconnected")))

    def _send_cmd(self, cmd_str):
        if not self.ser or not self.ser.is_open:
            self.log("Error: Not connected to Arduino.")
            return None
        try:
            self.ser.write((cmd_str + "\n").encode('ascii'))
            self.ser.flush()
            resp = self.ser.readline().decode('ascii', errors='ignore').strip()
            return resp
        except Exception as e:
            self.log(f"Serial Error sending '{cmd_str}': {e}")
            return None

    def _on_mri_toggle(self):
        if self.is_connected:
            threading.Thread(target=self._send_wait_mri_command, args=(self.wait_mri_var.get(),), daemon=True).start()

    def _send_wait_mri_command(self, wait_bool):
        val = 1 if wait_bool else 0
        resp = self._send_cmd(f"WAITMRI {val}")
        if resp:
            self.log(f"Arduino response: {resp}")

    # ============================================================
    # TRAJECTORY GENERATION & CSV HANDLING
    # ============================================================
    def generate_trajectory(self):
        move_time = float(self.move_time_var.get())
        dwell_time = float(self.dwell_time_var.get())
        amplitude = float(self.amplitude_var.get())
        gamma = float(self.gamma_var.get())
        microstep = float(self.microstep_var.get())

        full_period_ms = move_time + dwell_time
        if full_period_ms > 65535:
            raise ValueError("Period exceeds maximum limit of 65535 ms.")

        trajectory = []
        for i in range(self.num_points):
            fraction = i / self.num_points
            t_ms = fraction * full_period_ms
            if t_ms < move_time:
                ratio = t_ms / move_time if move_time > 0 else 0
                position = amplitude * microstep * (1.0 - math.cos(2.0 * math.pi * math.pow(ratio, gamma))) / 1.8
            else:
                position = 0.0

            trajectory.append((int(round(t_ms)), int(round(position))))

        return trajectory, int(round(full_period_ms))

    def save_trajectory_csv(self, filename, trajectory, period_ms):
        path = Path(filename)
        with open(path, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([f"# period_ms:{period_ms}"])
            writer.writerow(["time_ms", "position_steps"])
            for t_ms, pos in trajectory:
                writer.writerow([t_ms, pos])
        self.log(f"Trajectory saved to {path.resolve()}")

    def load_trajectory_csv(self, filename):
        path = Path(filename)
        if not path.exists():
            raise FileNotFoundError(f"File '{filename}' not found.")

        trajectory = []
        period_ms = None

        with open(path, "r") as file:
            reader = csv.reader(file)
            for row in reader:
                if not row:
                    continue
                first = row[0].strip()
                if first.startswith("#"):
                    if "period_ms:" in first:
                        period_ms = int(first.split("period_ms:")[1])
                    continue
                if first.lower() in ["time_ms", "time", "t"]:
                    continue
                if len(row) >= 2:
                    try:
                        t = int(row[0].strip())
                        p = int(row[1].strip())
                        trajectory.append((t, p))
                    except ValueError:
                        continue

        if not trajectory:
            raise ValueError("No valid trajectory points found in CSV file.")

        if period_ms is None:
            times = [t for t, p in trajectory]
            if len(times) > 1:
                dt = (times[-1] - times[0]) / (len(times) - 1)
                period_ms = int(round(times[-1] + dt))
            else:
                period_ms = times[0]

        return trajectory, period_ms

    # ============================================================
    # ACTIONS
    # ============================================================
    def _action_generate_and_save(self):
        try:
            traj, period = self.generate_trajectory()
            fn = self.save_filename_var.get().strip() or "trajectory.csv"
            self.save_trajectory_csv(fn, traj, period)
            self._scan_csv_files()
            messagebox.showinfo("Success", f"Trajectory saved to {fn}\nPoints: {len(traj)}\nPeriod: {period} ms")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _action_generate_save_send(self):
        try:
            traj, period = self.generate_trajectory()
            fn = self.save_filename_var.get().strip() or "trajectory.csv"
            self.save_trajectory_csv(fn, traj, period)
            self._scan_csv_files()
            if not self.is_connected:
                messagebox.showwarning("Warning", f"Trajectory saved to {fn}, but serial port is not connected.")
                return
            threading.Thread(target=self._upload_trajectory_thread, args=(traj, period), daemon=True).start()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _action_send_selected_file(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Serial port is not connected.")
            return

        filepath = self.selected_file_var.get().strip()
        if not filepath:
            messagebox.showerror("Error", "Please select a CSV file first.")
            return

        try:
            traj, period = self.load_trajectory_csv(filepath)
            threading.Thread(target=self._upload_trajectory_thread, args=(traj, period), daemon=True).start()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load CSV: {e}")

    def _upload_trajectory_thread(self, trajectory, period_ms):
        self.log(f"Uploading trajectory ({len(trajectory)} points, {period_ms} ms)...")
        try:
            self._send_wait_mri_command(self.wait_mri_var.get())

            # Send Header
            resp = self._send_cmd(f"SEND {len(trajectory)} {period_ms}")
            if resp != "OK HEADER":
                self.log(f"Upload failed at header: {resp}")
                return

            # Stream Points
            for idx, (t_ms, pos) in enumerate(trajectory):
                line = f"{t_ms},{pos}"
                resp = self._send_cmd(line)
                if resp != f"OK {idx}":
                    self.log(f"Upload error at point {idx}: {resp}")
                    return
                if idx % 100 == 0 or idx == len(trajectory) - 1:
                    pct = 100.0 * (idx + 1) / len(trajectory)
                    self.log(f"Progress: {pct:5.1f}%")

            # Finish Transfer
            resp = self._send_cmd("END")
            self.log(f"Arduino final status: {resp}")
            if resp in ["START", "WAITING_TRIGGER"]:
                self.log("Trajectory loaded successfully.")
        except Exception as e:
            self.log(f"Upload exception: {e}")

    def _action_reset_zero(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected.")
            return
        threading.Thread(target=lambda: self.log(f"Arduino: {self._send_cmd('RESET')}"), daemon=True).start()

    def _action_set_zero(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected.")
            return
        threading.Thread(target=lambda: self.log(f"Arduino: {self._send_cmd('SETZERO')}"), daemon=True).start()

    def _action_move_steps(self, direction):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected.")
            return
        try:
            steps = int(self.steps_var.get()) * direction
            threading.Thread(target=lambda: self.log(f"Arduino: {self._send_cmd(f'MOVE {steps}')}"), daemon=True).start()
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid integer for steps.")

    def _action_stop(self):
        if not self.is_connected:
            messagebox.showwarning("Warning", "Not connected.")
            return
        threading.Thread(target=lambda: self.log(f"Arduino: {self._send_cmd('STOP')}"), daemon=True).start()


if __name__ == "__main__":
    app = StepperControlGUI()
    app.mainloop()