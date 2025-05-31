from scipy.signal import find_peaks
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import threading
import queue
import time
import sys
import numpy as np
from odrive.enums import AxisState
import pandas as pd
from datetime import datetime

from utility import *

# Game functions
def fight_user_emg_based(odrv0):
    '''
    Uses EMG signals to modulate torque in a human-vs-machine arm wrestling scenario.
    If high EMG activity is detected without user movement, the system reduces torque.
    Otherwise, it ramps up torque as needed to simulate resistance.
    '''

    # parameters for streaming EMG data
    BAUD_RATE = 115200
    PLOT_WINDOW_SECONDS = 10

    # Cutoff frequency in Hz
    CUTOFF = 20  
    # EMG sampling rate must be >= 900–1000 Hz for 450 Hz upper cutoff
    FS = 100 
    # define the buffer to be 3 seconds of data
    BUFFER_SIZE = 3 * FS  
    # Butterworth filter order
    FILTER_ORDER = 4  

    # initialize EMG data lists
    emg_data = []
    emg_timestamps = []

    # ---- Globals ----
    # queue of time series data from Arduino
    global data_queue
    data_queue = queue.Queue()

    # Constants
    global start_position
    start_position = 0.0
    max_position = -4.2       # hehe funny number but it's true
    position_step = -0.1       # position increment (in encoder counts or turns)
    torque_step = -0.05         # increment per cycle
    pause_between_modes = 0.1  # seconds to pause between mode switches
    max_torque = -3.48           # Nm

    # Game mode stuff
    winning_thresh = 0.1      # within here of the max position, consider the machine to have won
    max_user_failures_before_easing_up = 2
    max_max_torque_times_before_user_win = 15

    # Intialize threshold
    start_relaxing_position_thresh = -0.75
    emg_effort_threshold = 0.6   # CONSTANT: when emg_avg_value > threshold + relaxed_emg_value, ease up

    # Initialize axis and its state
    axis = odrv0.axis0
    curr_torque = axis.controller.effective_torque_setpoint
    curr_current = odrv0.ibus
    curr_position = axis.pos_estimate
    print(f"START: current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
    time.sleep(0.5)

    # initialize torque control variables
    desired_torque = 0
    last_position = curr_position

    # keep track of number of failures
    num_user_failures = 0
    num_machine_failures = 0
    num_max_torque_times = 0

    # EMG average variables
    emg_avg_val_voltages = []
    emg_avg_val_times = []
    relaxed_emg_avg_value = -1
    emg_avg_value = 0
    emg_effort_val = 0

    # Logging torque and EMG
    torque_log = []
    time_log = []
    function_start_time2 = time.time()

    # Start EMG thread, passing in start_time
    port = find_arduino()
    if not port:
        print("[ERROR] Arduino not found.")
        sys.exit(1)
    threading.Thread(target=read_serial_data, args=(port,data_queue,BAUD_RATE,), daemon=True).start()

    # initialize data structures for plotting EMG data
    x_vals = []
    y_vals = []
    raw_buffer = deque(maxlen=BUFFER_SIZE)
    b, a = butter_lowpass(CUTOFF, FS, FILTER_ORDER)
    plot_filtered = [False]

    # Peak tracking
    total_peaks = [0]
    max_peak_amp = [0]
    max_peak_width = [0]  # In milliseconds

    # plot setup
    plt.ion()
    fig, (ax_time, ax_freq, ax_torque) = plt.subplots(3, 1, figsize=(10, 8))
    plt.subplots_adjust(bottom=0.3, hspace=0.4)

    # time-domain plot
    raw_line, = ax_time.plot([], [], label="Raw EMG")
    filt_line, = ax_time.plot([], [], label="Filtered EMG")
    peak_line, = ax_time.plot([], [], 'rx', label="Detected Peaks")
    line_torque, = ax_torque.plot([], [], label='Torque (Nm)')

    # axes labels
    ax_time.set_xlabel("Time (s)")
    ax_time.set_ylabel("EMG Value")
    ax_time.set_title("Live EMG Time Domain")
    ax_time.grid(True)
    ax_time.legend()

    # torque sublot
    ax_torque.set_ylabel('Torque (Nm)')
    ax_torque.set_title('Torque and EMG vs Time')
    ax_torque.grid(True)
    ax_torque.legend()

    # frequency-domain plot
    fft_line, = ax_freq.plot([], [], label="FFT")
    ax_freq.set_xlabel("Frequency (Hz)")
    ax_freq.set_ylabel("Amplitude")
    ax_freq.set_title("Live EMG Frequency Spectrum")
    ax_freq.grid(True)
    ax_freq.set_xlim(0, FS // 2)
    ax_freq.legend()

    # display text
    text_box = plt.axes([0.1, 0.01, 0.8, 0.1])
    info_text = text_box.text(0, 0.6, '', transform=text_box.transAxes, fontsize=10, verticalalignment='top')
    text_box.axis('off')

    # button handlers
    def set_raw(event):
        plot_filtered[0] = False
        print("[INFO] Raw mode")

    def set_filtered(event):
        plot_filtered[0] = True
        print("[INFO] Filtered mode")

    # add buttons for switching between raw vs filtered data on the plot
    ax_raw = plt.axes([0.1, 0.15, 0.15, 0.075])
    ax_filt = plt.axes([0.3, 0.15, 0.2, 0.075])
    btn_raw = Button(ax_raw, 'Raw')
    btn_filt = Button(ax_filt, 'Filtered')
    btn_raw.on_clicked(set_raw)
    btn_filt.on_clicked(set_filtered)

    # Start with the arm up
    set_torque(odrv0=odrv0, target_torque=0)
    set_position(target_position=start_position, axis=axis)
    print(f"START: current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")

    # while the game hasn't been won
    while (abs(max_position) - abs(curr_position) > winning_thresh):
        # get the current time, relative to the start time of the EMG thread
        timestamp = time.time() - function_start_time2
        # read the motor torque, motor position, calcualte EMG effort
        curr_torque = axis.controller.effective_torque_setpoint
        curr_position = axis.pos_estimate
        emg_effort_val = emg_avg_value - relaxed_emg_avg_value
        print(f"EMG: effort = {emg_effort_val} = {emg_avg_value} - {relaxed_emg_avg_value}")

        # if the user is winning
        if ((abs(curr_position) == abs(last_position) and emg_effort_val < emg_effort_threshold) 
            or (abs(curr_position) < abs(last_position))
            or (abs(curr_position) < abs(start_relaxing_position_thresh))):

            # increment the number of machine's losses
            num_machine_failures += 1
            # clear the number of user's losses
            num_user_failures = 0

            # increment the torque, maxed at the max_torque
            # abs values let the logic hold for both directions of motor spin
            desired_torque = curr_torque + torque_step
            if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

            print(f"user winning!", end='  ')
       
        # else, the machine is winning
        else:
            # increment the number of user's losses
            num_user_failures += 1
            # clear the number of machine's losses
            num_machine_failures = 0

            # if the user's effort is above the threshold
            if emg_effort_val > emg_effort_threshold:
                # decrease the torque
                desired_torque = curr_torque - emg_effort_val*torque_step if (abs(curr_torque - emg_effort_val*torque_step) < abs(curr_torque)) else 0
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque
                if (desired_torque > 0): desired_torque = 0

                # reset number of user failures since we're decreasing difficulty
                print("user losing !", end='  ')
                num_user_failures = 0

            # if the user's effort isn't above the threshold then just check that the desired torque isn't above the max
            else:
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque
                print("user losing!!", end='  ')

        # if motor is at max torque, count the number of times it is
        if (abs(curr_torque) >= abs(max_torque)):
            num_max_torque_times += 1
            print(f"!!!!!! AT MAX TORQUE: TIME {num_max_torque_times}")
            # if the user can hold the motor at max torque enough times, user wins
            if (num_max_torque_times >= max_max_torque_times_before_user_win): break
        # if motor isn't at max torque, clear the max torque counter to 0
        else:
            num_max_torque_times = 0

        # update the motor's torque and record its current position
        print(f"desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f}")
        set_torque(odrv0=odrv0, target_torque=desired_torque)
        last_position = curr_position

        # Log torque time series data
        torque_log.append(abs(curr_torque))
        time_log.append(timestamp)

        # Get EMG data from queue (queue is Arduino data stream)
        num_emg_data = 0
        emg_avg_value = 0
        emg_avg_time = 0
        try:
            while not data_queue.empty():
                print(list(data_queue.queue))
                # pop a datapoint from the queue
                t, v = data_queue.get()

                # add the data to the corresponding data structures
                x_vals.append(t)
                y_vals.append(v)
                raw_buffer.append(v)

                num_emg_data += 1
                emg_t, emg_val = t, v

                # Add to overall emg lists
                emg_data.append(emg_val)
                emg_timestamps.append(emg_t)

                # Calculating average
                emg_avg_time += emg_t
                emg_avg_value += emg_val

            # TODO check that this does what you want it to, previous code looked weird
            # calculate and append the average time and voltage data
            emg_avg_val_times.append(sum(emg_timestamps)/len(emg_timestamps))
            emg_avg_val_voltages.append(sum(emg_data)/len(emg_data))

            # assume the first emg datapoint is relaxed, user should be relaxed when starting the game
            if num_emg_data:
                if (relaxed_emg_avg_value == -1): relaxed_emg_avg_value = emg_avg_value                    

            # Trim data to plot window
            while x_vals and x_vals[-1] - x_vals[0] > PLOT_WINDOW_SECONDS:
                x_vals.pop(0)
                y_vals.pop(0)

            # if we want to look at filtered data
            if plot_filtered[0]:
                # if we have enough unfiltered data
                if len(raw_buffer) >= FILTER_ORDER:
                    # apply the filter
                    filtered = apply_filter(list(raw_buffer), b, a)
                    # get the filtered voltage data post trim
                    filt_y = filtered[-len(y_vals):]
                    # --- Compute FFT ---
                    if len(filt_y) >= FS:  # At least 1 second of data
                        windowed = np.array(filt_y[-FS:]) * np.hanning(FS)
                        fft_vals = np.abs(np.fft.rfft(windowed))
                        fft_freqs = np.fft.rfftfreq(FS, d=1.0/FS)

                        # add the data to the plot
                        fft_line.set_xdata(fft_freqs)
                        fft_line.set_ydata(fft_vals)

                    # get the filtered time data post trim
                    filt_x = x_vals[-len(filt_y):]

                    # peak detection
                    peaks, props = find_peaks(filt_y, height=200, distance=10, width=5)

                    # increment the total peaks count
                    total_peaks[0] += len(peaks)

                    # if we have peaks
                    if len(peaks) > 0:
                        # find the maximum peak height and width
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
                    
            # else add the unfiltered data to the corresponding plot
            else:
                filt_line.set_xdata([])
                filt_line.set_ydata([])
                peak_line.set_xdata([])
                peak_line.set_ydata([])
                raw_line.set_xdata(x_vals)
                raw_line.set_ydata(y_vals)
                fft_line.set_xdata([])
                fft_line.set_ydata([])

            # autoscale the plot
            ax_time.relim()
            ax_time.autoscale_view()
            ax_freq.relim()
            ax_freq.autoscale_view()

            plt.pause(0.01)

        # handle manual interrupt
        except KeyboardInterrupt:
            print("[INFO] Plotting stopped.")
            break

        # autoscale the plot
        if len(time_log) >= 2:
            line_torque.set_xdata(time_log)
            line_torque.set_ydata(torque_log)
            ax_torque.relim()
            ax_torque.autoscale_view()
            plt.pause(0.01)         


    # after the game has been won, check whether the user or machine won and print the result
    if (num_max_torque_times >= max_max_torque_times_before_user_win):
        print("User has won.")
    else:
        print("Machine has won.")

    # Final stats
    total_time = time.time() - function_start_time2
    avg_torque = sum(torque_log) / len(torque_log) if torque_log else 0
    plt.ioff()
    plt.close(fig)

    # Reset position
    time.sleep(1)
    set_torque(odrv0=odrv0, target_torque=0)
    set_position(target_position=start_position, axis=axis)

    # Save CSV
    df = pd.DataFrame({'time_sec': time_log, 'torque_Nm': torque_log})
    filename = f"logs/torque_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    df.to_csv(filename, index=False)
    print(f"Saved torque log to {filename}")

    # Add subtitle with stats
    subtitle = f"Average Torque: {avg_torque:.2f} Nm | Total Time: {total_time:.2f} s | Mode: EMG-depedent"
    ax_torque.set_title('Torque vs Time\n' + subtitle)

    # Save the figure
    plot_filename = filename.replace(".csv", ".png")
    fig.savefig(plot_filename)
    print(f"Saved torque plot to {plot_filename}")

    return {
        "total_time_sec": total_time,
        "average_torque_Nm": avg_torque,
        #"csv_file": filename,
        #"plot_file": plot_filename
    }

def fight_machine():
    
    try:
        # Setup
        odrv0 = find_odrive()

        # Confirming config settings
        odrv0.axis0.controller.config.vel_limit = 20.0    # Limiting the velocity (rad/s). LIMITS EFFECTIVE TORQUE SETPOINT
        odrv0.axis0.controller.config.vel_gain = 0.5      # Lower vel_gain for smoother motion. LIMITS EFFECTIVE TORQUE SETPOINT
        odrv0.axis0.config.motor.current_soft_max = 80    # The Flipsky can take up to 80A 
        odrv0.axis0.config.motor.current_hard_max = 80    
        odrv0.axis0.config.motor.current_control_bandwidth = 200
        
        # odrv0.axis0.pos_estimate = 0  # Uncomment to set the current position as 0

        clear_errors(odrv0)
        axis = odrv0.axis0

        fight_user_emg_based(odrv0)
        # constant_torque_mode(odrv0, begin_fight_pos=max_position, torque_setpoint= -0.35)

        odrv0.axis0.requested_state = AxisState.IDLE

    except KeyboardInterrupt:
        print("Keyboard Interrupt. Stopping motor...")
        set_torque(odrv0=odrv0, target_torque=0)
        set_position(target_position=start_position, axis=axis)
        odrv0.axis0.requested_state = AxisState.IDLE    
    except Exception as e:
        print(f"An error occurred: {e}")
        print("Stopping motor...")
        clear_errors(odrv0)
        set_torque(odrv0=odrv0, target_torque=0)
        set_position(target_position=start_position, axis=axis)
        odrv0.axis0.requested_state = AxisState.IDLE
