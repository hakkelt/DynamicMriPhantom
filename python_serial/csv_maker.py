import serial
import time
import math
import csv
from pathlib import Path


# ============================================================
# USER SETTINGS
# ============================================================

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Number of trajectory points.

# More points = more accurate representation of the function,
# but more data must be stored in the Nano's RAM.
NUM_POINTS = 1024

# File in which the trajectory will be saved
OUTPUT_FILE = Path("trajectory.csv")


# ============================================================
# TRAJECTORY GENERATION
# ============================================================

def generate_trajectory(move_time, dwell_time, amplitude, gamma, microstep = 2):
    """
    Generate one complete sine-wave cycle.

    Returns:
        list of (time_ms, position_steps)
    """

    # One complete sine cycle occurs over 2*pi*TAU
    full_period_ms = move_time + dwell_time

    if full_period_ms > 65535:
        raise ValueError(
            "The period is too long. "
            "The Arduino protocol uses uint16_t time values "
            "(maximum 65535 ms)."
        )

    trajectory = []

    for i in range(NUM_POINTS):

        # Time from 0 to just before one complete period
        fraction = i / NUM_POINTS
        t_ms = fraction * full_period_ms

        # The requested function:
        if(t_ms < move_time):
            position = amplitude * microstep * (1 - math.cos(2 * math.pi * math.pow( (t_ms / move_time),gamma) )) / 1.8
        else:
            position = 0

        time_ms = int(round(t_ms))
        position_steps = int(round(position))

        trajectory.append((time_ms, position_steps))

    return trajectory, full_period_ms


# ============================================================
# SAVE TRAJECTORY TO CSV
# ============================================================

def save_trajectory(trajectory, period_ms):

    with open(OUTPUT_FILE, "w", newline="") as file:

        writer = csv.writer(file)

        writer.writerow([
            "time_ms",
            "position_steps"
        ])

        for time_ms, position in trajectory:
            writer.writerow([
                time_ms,
                position
            ])

    print(f"Trajectory saved to: {OUTPUT_FILE}")
    print(f"Number of points: {len(trajectory)}")
    print(f"Period: {period_ms} ms")


# ============================================================
# SERIAL COMMUNICATION
# ============================================================

def connect_to_arduino(ser, timeout=10):

    print("Connecting to Arduino...")

    start = time.time()

    while time.time() - start < timeout:

        print("Sending HELLO...")

        ser.write(b"HELLO\n")
        ser.flush()

        response = ser.readline().decode(
            "ascii",
            errors="ignore"
        ).strip()

        if response:
            print("Arduino:", response)

        if response == "READY":
            print("Arduino is ready.")
            return True

        time.sleep(0.5)

    return False


def send_trajectory(ser, trajectory, period_ms):

    print("Sending trajectory...")

    # --------------------------------------------------------
    # HEADER
    #
    # SEND <number_of_points> <period_ms>
    # --------------------------------------------------------

    header = f"SEND {len(trajectory)} {period_ms}\n"

    ser.write(header.encode("ascii"))
    ser.flush()

    # Wait for Arduino acknowledgement
    response = ser.readline().decode(
        "ascii",
        errors="ignore"
    ).strip()

    print("Arduino:", response)

    if response != "OK HEADER":
        raise RuntimeError(
            f"Arduino did not confirm transfer: {response}"
        )

    # --------------------------------------------------------
    # SEND DATA
    #
    # Each line:
    #
    # time_ms,position_steps
    #
    # --------------------------------------------------------

    for index, (time_ms, position) in enumerate(trajectory):

        line = f"{time_ms},{position}\n"

        print("LINE: " + line)

        ser.write(line.encode("ascii"))

        # Wait for acknowledgement for each point.
        #
        # This is slower than a binary protocol, but extremely
        # useful during development because it makes debugging
        # very easy.
        response = ser.readline().decode(
            "ascii",
            errors="ignore"
        ).strip()

        expected = f"OK {index}"

        print(response)

        if response != expected:
            raise RuntimeError(
                f"Transfer error at point {index}. "
                f"Expected '{expected}', got '{response}'"
            )

        if index % 10 == 0 or index == len(trajectory) - 1:

            percentage = (
                100.0 * (index + 1) / len(trajectory)
            )

            print(
                f"\rProgress: {percentage:6.2f}%",
                end=""
            )

    print()

    # --------------------------------------------------------
    # TELL ARDUINO THE TRANSFER IS COMPLETE
    # --------------------------------------------------------

    ser.write(b"END\n")
    ser.flush()

    response = ser.readline().decode(
        "ascii",
        errors="ignore"
    ).strip()

    print("Arduino:", response)

    if response != "START":
        raise RuntimeError(
            f"Arduino did not start trajectory: {response}"
        )

    print()
    print("Trajectory successfully transferred.")
    print("Arduino has started the trajectory.")
    print("The Arduino will now repeat it autonomously.")


# ============================================================
# MAIN
# ============================================================

def main():

    # Generate trajectory
    trajectory, period_ms = generate_trajectory(
        move_time= 1000,
        dwell_time= 500,
        amplitude= 180,
        gamma= 1,
    )

    # Save CSV
    save_trajectory(
        trajectory,
        period_ms
    )

    print()
    print(f"Opening serial port {SERIAL_PORT}...")

    try:

        ser = serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=1
        )

        # Opening the Nano's serial port normally resets it.
        time.sleep(2)

        # Remove any stale data.
        ser.reset_input_buffer()

        # Explicit handshake.
        if not connect_to_arduino(ser):

            ser.close()

            raise RuntimeError(
                "Arduino did not respond to HELLO"
            )


        # ----------------------------------------------------
        # Send trajectory
        # ----------------------------------------------------

        send_trajectory(
            ser,
            trajectory,
            period_ms
        )

        print()
        print("Trajectory successfully transferred.")
        print("Arduino is now running the trajectory.")
        print()
        print("Python can now exit; the Arduino will continue")
        print("repeating the trajectory.")

        ser.close()

    except serial.SerialException as error:

        print()
        print("Serial error:")
        print(error)
        print()
        print("Check:")
        print("  - Is the Arduino connected?")
        print("  - Is SERIAL_PORT correct?")
        print("  - Is another program using the serial port?")


if __name__ == "__main__":
    main()