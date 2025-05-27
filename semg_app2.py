from scipy.signal import butter, lfilter, find_peaks
from collections import deque
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import threading
import queue
import time
import sys
import numpy as np


# ---- Constants ----
BAUD_RATE = 115200
PLOT_WINDOW_SECONDS = 10
STREAM_CMD = bytes([3])

# Butterworth filter parameters

CUTOFF = 20  # Cutoff frequency in Hz
FS = 100  # EMG sampling rate must be >= 900–1000 Hz for 450 Hz upper cutoff
BUFFER_SIZE = 3 * FS  # 3 seconds of data


FILTER_ORDER = 4

# ---- Globals ----
data_queue = queue.Queue()
start_time = None

def find_arduino():
    arduino_vids = ["2341", "1A86", "10C4", "2A03"]
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if (port.vid and f"{port.vid:04X}" in arduino_vids) or "Arduino" in port.description:
            return port.device
    return None

def read_serial_data(port):
    global start_time
    try:
        with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
            print(f"[INFO] Connected to {port}")
            time.sleep(2)
            ser.write(STREAM_CMD)

            while True:
                line = ser.readline().decode('utf-8').strip()
                if ',' not in line:
                    continue
                try:
                    t_str, v_str = line.split(',')
                    t = float(t_str) / 1000.0
                    v = float(v_str)
                    if start_time is None:
                        start_time = t
                    data_queue.put((t - start_time, v))
                except ValueError:
                    continue
    except Exception as e:
        print(f"[ERROR] Serial read failed: {e}")
        sys.exit(1)

# Butterworth Filter Setup
def butter_lowpass(cutoff, fs, order=4):
    nyq = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyq
    return butter(order, normal_cutoff, btype='low', analog=False)


def apply_filter(data, b, a):
    return lfilter(b, a, data)

def live_plot():
    x_vals = []
    y_vals = []
    raw_buffer = deque(maxlen=BUFFER_SIZE)

    b, a = butter_lowpass(CUTOFF, FS, FILTER_ORDER)
    plot_filtered = [False]

    # Peak tracking
    total_peaks = [0]
    max_peak_amp = [0]
    max_peak_width = [0]  # In milliseconds

    plt.ion()
    fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(10, 8))
    plt.subplots_adjust(bottom=0.3, hspace=0.4)

    # Time-domain plot
    raw_line, = ax_time.plot([], [], label="Raw EMG")
    filt_line, = ax_time.plot([], [], label="Filtered EMG")
    peak_line, = ax_time.plot([], [], 'rx', label="Detected Peaks")
    ax_time.set_xlabel("Time (s)")
    ax_time.set_ylabel("EMG Value")
    ax_time.set_title("Live EMG Time Domain")
    ax_time.grid(True)
    ax_time.legend()

    # Frequency-domain plot
    fft_line, = ax_freq.plot([], [], label="FFT")
    ax_freq.set_xlabel("Frequency (Hz)")
    ax_freq.set_ylabel("Amplitude")
    ax_freq.set_title("Live EMG Frequency Spectrum")
    ax_freq.grid(True)
    ax_freq.set_xlim(0, FS // 2)
    ax_freq.legend()

    # Display text
    text_box = plt.axes([0.1, 0.01, 0.8, 0.1])
    info_text = text_box.text(0, 0.6, '', transform=text_box.transAxes, fontsize=10, verticalalignment='top')
    text_box.axis('off')

    # Button handlers
    def set_raw(event):
        plot_filtered[0] = False
        print("[INFO] Raw mode")

    def set_filtered(event):
        plot_filtered[0] = True
        print("[INFO] Filtered mode")

    # Buttons
    ax_raw = plt.axes([0.1, 0.15, 0.15, 0.075])
    ax_filt = plt.axes([0.3, 0.15, 0.2, 0.075])
    btn_raw = Button(ax_raw, 'Raw')
    btn_filt = Button(ax_filt, 'Filtered')
    btn_raw.on_clicked(set_raw)
    btn_filt.on_clicked(set_filtered)

    while True:
        try:
            while not data_queue.empty():
                t, v = data_queue.get()
                x_vals.append(t)
                y_vals.append(v)
                raw_buffer.append(v)

            # Trim data to plot window
            while x_vals and x_vals[-1] - x_vals[0] > PLOT_WINDOW_SECONDS:
                x_vals.pop(0)
                y_vals.pop(0)

            if plot_filtered[0]:
                if len(raw_buffer) >= FILTER_ORDER:
                    filtered = apply_filter(list(raw_buffer), b, a)
                    filt_y = filtered[-len(y_vals):]
                    # --- Compute FFT ---
                    if len(filt_y) >= FS:  # At least 1 second of data
                        windowed = np.array(filt_y[-FS:]) * np.hanning(FS)
                        fft_vals = np.abs(np.fft.rfft(windowed))
                        fft_freqs = np.fft.rfftfreq(FS, d=1.0/FS)

                        fft_line.set_xdata(fft_freqs)
                        fft_line.set_ydata(fft_vals)

                    filt_x = x_vals[-len(filt_y):]

                    # Peak detection
                    peaks, props = find_peaks(filt_y, height=200, distance=10, width=5)

                    total_peaks[0] += len(peaks)

                    if len(peaks) > 0:
                        max_peak_amp[0] = max(max_peak_amp[0], max(props['peak_heights']))
                        peak_width_ms = max(props['widths']) * (1000 / FS)
                        max_peak_width[0] = max(max_peak_width[0], peak_width_ms)

                    # Plot filtered + peaks
                    raw_line.set_xdata([])
                    raw_line.set_ydata([])
                    filt_line.set_xdata(filt_x)
                    filt_line.set_ydata(filt_y)
                    peak_line.set_xdata(np.array(filt_x)[peaks])
                    peak_line.set_ydata(np.array(filt_y)[peaks])

                    # Update text
                    info_text.set_text(f"Peaks Detected: {total_peaks[0]}\n"
                                       f"Max Peak Amplitude: {max_peak_amp[0]:.2f}\n"
                                       f"Max Peak Width: {max_peak_width[0]:.2f} ms")
            else:
                filt_line.set_xdata([])
                filt_line.set_ydata([])
                peak_line.set_xdata([])
                peak_line.set_ydata([])
                raw_line.set_xdata(x_vals)
                raw_line.set_ydata(y_vals)
                fft_line.set_xdata([])
                fft_line.set_ydata([])


            ax_time.relim()
            ax_time.autoscale_view()
            ax_freq.relim()
            ax_freq.autoscale_view()

            plt.pause(0.01)

        except KeyboardInterrupt:
            print("[INFO] Plotting stopped.")
            break

if __name__ == "__main__":
    port = find_arduino()
    if not port:
        print("[ERROR] Arduino not found.")
        sys.exit(1)

    threading.Thread(target=read_serial_data, args=(port,), daemon=True).start()
    live_plot()
