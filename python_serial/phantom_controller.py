# --- Shared Controller Logic (Save as phantom_controller.py) ---
import serial
import json
import time

class PhantomController:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) # Wait for auto-reset

    def _send_command(self, cmd_dict):
        payload = json.dumps(cmd_dict) + '\n'
        self.ser.write(payload.encode('utf-8'))
        response = self.ser.readline().decode('utf-8').strip()
        print(f"\n[Device Response]: {response}")

    def start_cycle(self, amplitude, move_ms, dwell_ms, gamma_val, microstep, mri_signal = 0, cycles = 99, max_speed = 0):
        self._send_command({
            "cmd": "start",
            "params": {"amplitude": amplitude, "move_ms": move_ms, "dwell_ms": dwell_ms, "gamma": gamma_val,
                       "microstep": microstep, "mri_signal": mri_signal, "cycles": cycles, "max_speed": max_speed}
        })

    def stop_motion(self):
        self._send_command({"cmd": "stop"})

    def reset_to_baseline(self):
        self._send_command({"cmd": "reset"})