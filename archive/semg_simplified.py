import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import threading
import queue
import numpy as np


BAUD_RATE = 115200
PORT = None  # Can be manually set if known
FS = 100  # Sampling rate
BUFFER_SIZE = 3 * FS  # Buffer size for plotting (3 seconds)
STREAM_CMD = bytes([3])


data_queue = queue.Queue()
start_time = None

def find_arduino(baud_rate=115200, timeout=2):
    arduino_vids = ["2341", "1A86", "10C4", "2A03"]
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if (port.vid and f"{port.vid:04X}" in arduino_vids) or "Arduino" in port.description:
            try:
                with serial.Serial(port.device, baud_rate, timeout=timeout) as ser:
                    ser.flushInput()
                    line = ser.readline().decode("utf-8").strip()
                    if line:
                        print(f"Found Arduino on {port.device}: {line}")
                        return port.device
            except Exception as e:
                print(f"Error checking {port.device}: {e}")
    return None

def read_serial_data(port):
    global start_time
    try:
        with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
            print(f"[INFO] Connected to {port}")
            import time; time.sleep(2)
            ser.write(STREAM_CMD)
            while True:
                line = ser.readline().decode("utf-8").strip()
                if "," not in line:
                    continue
                try:
                    t_str, v_str = line.split(",")
                    t = float(t_str) / 1000.0
                    v = float(v_str)
                    if start_time is None:
                        start_time = t
                    data_queue.put((t - start_time, v))
                except ValueError:
                    continue
    except Exception as e:
        print(f"[ERROR] Serial read failed: {e}")

def live_plot():
    x_vals = []
    y_vals = []

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], label="Raw EMG")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("EMG Value")
    ax.set_title("Live EMG")
    ax.grid(True)
    ax.legend()

    while True:
        try:
            while not data_queue.empty():
                t, v = data_queue.get()
                x_vals.append(t)
                y_vals.append(v)

            # Limit window
            while x_vals and x_vals[-1] - x_vals[0] > 10:
                x_vals.pop(0)
                y_vals.pop(0)

            line.set_xdata(x_vals)
            line.set_ydata(y_vals)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.01)
        except KeyboardInterrupt:
            print("Plotting stopped.")
            break

if __name__ == "__main__":
    if PORT is None:
        PORT = find_arduino(BAUD_RATE)
    if not PORT:
        print("No Arduino found. Exiting.")
        exit(1)
    threading.Thread(target=read_serial_data, args=(PORT,), daemon=True).start()
    live_plot()
