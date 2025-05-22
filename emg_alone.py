import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import threading
import queue

'''
For EMG diagnostics.
'''
# ---- Config ----
BAUD_RATE = 115200
PORT = None  # If known, set manually (e.g., "COM4" or "/dev/ttyUSB0")

# ---- Globals ----
emg_queue = queue.Queue()
emg_start_time = -1

# ---- Helper: Auto-find Arduino ----
def find_arduino(baud_rate=115200, timeout=2):
    arduino_vids = ["2341", "1A86", "10C4", "2A03"]  # Common VIDs
    ports = serial.tools.list_ports.comports()

    for port in ports:
        if (port.vid and f"{port.vid:04X}" in arduino_vids) or "Arduino" in port.description:
            try:
                with serial.Serial(port.device, baud_rate, timeout=timeout) as ser:
                    ser.flushInput()
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"Found Arduino on {port.device}: {line}")
                        return port.device
            except Exception as e:
                print(f"Error reading {port.device}: {e}")
    print("No Arduino found.")
    return None

# ---- Serial Reader Thread ----
def read_emg_data(port, baud_rate=115200):
    global emg_start_time
    try:
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8').strip()
                if ',' not in line:
                    continue

                t_str, v_str = line.split(',')

                if emg_start_time == -1:
                    emg_start_time = float(t_str)

                timestamp = float(t_str) - emg_start_time
                value = float(v_str)

                emg_queue.put((timestamp, value))
    except Exception as e:
        print(f"[EMG Thread] Error: {e}")

# ---- Plotting ----
def live_plot():
    x_vals = []
    y_vals = []

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], label="EMG Signal")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("EMG Value")
    ax.set_title("Live EMG Stream")
    ax.grid(True)
    ax.legend()

    while True:
        try:
            while not emg_queue.empty():
                t, v = emg_queue.get()
                x_vals.append(t)
                y_vals.append(v)

            line.set_xdata(x_vals)
            line.set_ydata(y_vals)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)
        except KeyboardInterrupt:
            print("Plotting stopped.")
            break

# ---- Main ----
if __name__ == "__main__":
    if PORT is None:
        PORT = find_arduino(BAUD_RATE)

    if not PORT:
        print("No Arduino found. Exiting.")
        exit(1)

    # Start EMG reading in a background thread
    threading.Thread(target=read_emg_data, args=(PORT,), daemon=True).start()

    # Start live plot
    live_plot()
