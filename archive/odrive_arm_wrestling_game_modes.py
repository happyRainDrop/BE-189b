import time
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import serial.tools.list_ports
import serial
import threading
import queue

emg_start_time = -1
emg_data = []
emg_timestamps = []
emg_queue = queue.Queue()


# Helper functions: Arduino

def find_arduino(baud_rate=115200, timeout=2):
    """
    Scans available serial ports for an Arduino device by checking for known USB VID/PID
    and/or testing for readable output.

    Returns:
        A string with the port name (e.g., 'COM3') or None if not found.
    """
    arduino_vids = ["2341", "1A86", "10C4", "2A03"]  # Known Arduino/CH340/Silabs VIDs
    ports = serial.tools.list_ports.comports()

    for port in ports:
        # Try to match based on known vendor IDs
        if (port.vid and f"{port.vid:04X}" in arduino_vids) or "Arduino" in port.description:
            print(f"Found potential Arduino on {port.device} ({port.description})")
            try:
                with serial.Serial(port.device, baud_rate, timeout=timeout) as ser:
                    ser.flushInput()
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        print(f"  Read test successful: {line}")
                        return port.device
            except Exception as e:
                print(f"  Failed to read from {port.device}: {e}")

    print("No Arduino found.")
    return None

def read_emg_data(serial_port="COM8", baud_rate=115200):
    global emg_start_time

    with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
        while True:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8').strip()
                if ',' not in line:
                    continue

                if line:
                    t_str, v_str = line.split(',')

                    # set start time the first time the function is called
                    if emg_start_time == -1: emg_start_time = float(t_str)

                    value = float(v_str)
                    timestamp = float(t_str) - float(emg_start_time)
                    emg_queue.put((timestamp, value))

            except UnicodeDecodeError as e:
                print("Decode error:", e)
                line = None
            except Exception as e:
                print(f"EMG read error: {e}")
                break

# Helper functions: Odrive
def find_odrive():
    print("Waiting for ODrive...")
    odrv0 = odrive.find_sync()
    print(f"Found ODrive {odrv0._dev.serial_number}")
    return odrv0

def clear_errors(odrv0):
    if odrv0.axis0.active_errors != 0:
        dump_errors(odrv0)
    odrv0.clear_errors()
    # print("All errors cleared on ODrive")

def set_torque(odrv0, axis, target_torque, torque_step=0.01, torque_ramp_delay=0.05, max_power_supply_curr = 15):
    """
    Set torque first with ramp-up.

    Parameters:
    - axis: ODrive axis (e.g., odrv0.axis0)
    - target_torque: Target torque in Amps
    - target_position: Final position in encoder counts (or turns)
    - torque_step: Amps to increase per torque step
    - torque_ramp_delay: Delay between torque steps (seconds)
    - position_step: Position increment per step
    - position_ramp_delay: Delay between position steps (seconds)
    """

    verbose = False

    # --- Ramp Torque ---

    current_torque = axis.controller.effective_torque_setpoint
    num_current_set_failures = 0

    if verbose: print(f"[Torque Ramp] Switching to TORQUE_CONTROL with goal {target_torque} starting from {current_torque}")
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    request_state(axis, AxisState.CLOSED_LOOP_CONTROL)
    time.sleep(0.1)

    while abs(current_torque - target_torque) > abs(torque_step):
        current_torque = (current_torque + torque_step) if target_torque > current_torque else (current_torque-torque_step)
        if verbose: print(f" -> Commanded Torque: {axis.controller.input_torque}, Effective Torque: {axis.controller.effective_torque_setpoint}")
        if (odrv0.ibus >= max_power_supply_curr): break
        axis.controller.input_torque = current_torque

        # Don't get stuck here forever
        if ((current_torque < target_torque and axis.controller.effective_torque_setpoint < current_torque) or 
            (current_torque > target_torque and axis.controller.effective_torque_setpoint > current_torque)):
            num_current_set_failures += 1
        else:
            num_current_set_failures = 0
        if (num_current_set_failures > 5):
            # simply give up
            if verbose: print("QUITTING.")
            break

        current_torque = axis.controller.effective_torque_setpoint
        if verbose:  print(f" -> Torque: {current_torque:.3f} Nm")
        time.sleep(torque_ramp_delay)

    # Final torque set (if not exceed power supply limit)
    if (odrv0.ibus <= max_power_supply_curr): axis.controller.input_torque = target_torque
    if verbose:  
        print(f" -> Final Torque Setpoint: {target_torque:.2f} Nm, actual {axis.controller.effective_torque_setpoint}")

def set_position(target_position, axis, position_step = 0.05, position_ramp_delay=0.02):

    # Enter closed loop control
    axis.controller.config.input_mode = InputMode.PASSTHROUGH
    axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
    request_state(odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)

    print(f"Going to start position at {target_position}")
    curr_position = axis.pos_estimate

    while(curr_position > target_position):
        curr_position -= position_step
        odrv0.axis0.controller.input_pos = curr_position
        curr_position = axis.pos_estimate
        time.sleep(position_ramp_delay)

    while(curr_position < target_position):
        curr_position += position_step
        odrv0.axis0.controller.input_pos = curr_position
        curr_position = axis.pos_estimate
        time.sleep(position_ramp_delay)

    odrv0.axis0.controller.input_pos = target_position
    return


# Game functions
def fight_user(odrv0):
    '''
    If it detects it is not moving, sets torque higher.
    Streams a plot of torque over time and saves torque log.
    Gentle mode: Decreases torque when user at a standstill
    Else, keeps increasing it
    '''
    # Intialize threshold
    start_relaxing_position_thresh = -0.75
    modes = ["Gentle", "Hardcore"]
    mode_index = 1
    user_win = False

    # Initialize variables
    axis = odrv0.axis0
    curr_torque = axis.controller.effective_torque_setpoint
    curr_current = odrv0.ibus
    curr_position = axis.pos_estimate
    time.sleep(0.5)

    desired_torque = 0
    last_position = curr_position
    num_user_failures = 0
    num_machine_failures = 0
    num_max_torque_times = 0

    # Logging torque and EMG
    torque_log = []
    time_log = []
    start_time = time.time()
    # Start EMG thread, passing in start_time
    arduino_port = find_arduino()
    emg_thread = threading.Thread(
        target=read_emg_data,
        kwargs={'serial_port': arduino_port},
        daemon=True
    )
    emg_thread.start()

    # Plot setup
    plt.ion()
    fig, (ax_torque, ax_emg) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    line_torque, = ax_torque.plot([], [], label='Torque (Nm)')
    line_emg, = ax_emg.plot([], [], label='EMG', color='orange')
            # Torque sublot
    ax_torque.set_ylabel('Torque (Nm)')
    ax_torque.set_title('Torque and EMG vs Time')
    ax_torque.grid(True)
    ax_torque.legend()
            # Emg subplot
    ax_emg.set_xlabel('Time (s)')
    ax_emg.set_ylabel('EMG Value')
    ax_emg.grid(True)
    ax_emg.legend()


    # Start with the arm up
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)
    set_position(target_position=start_position, axis=axis)
    print(f"START: current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")

    # Fight user logic
    while (abs(max_position) - abs(curr_position) > winning_thresh):
        timestamp = time.time() - start_time
        curr_torque = axis.controller.effective_torque_setpoint
        curr_position = axis.pos_estimate

        # USER IS WINNING
        if ((abs(curr_position) < abs(last_position) and mode_index == 0)
            or (abs(curr_position) <= abs(last_position) and mode_index == 1) 
            or (abs(curr_position) < abs(start_relaxing_position_thresh))):

            num_machine_failures += 1
            num_user_failures = 0

            desired_torque = curr_torque + torque_step * num_machine_failures
            if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

            print(f"user winning!", end='  ')
       
        # USER IS LOSING
        else:
            num_user_failures += 1
            num_machine_failures = 0
            if num_user_failures > max_user_failures_before_easing_up:
                desired_torque = curr_torque - torque_step if (abs(curr_torque - torque_step) < abs(curr_torque)) else 0
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

                print("user losing !", end='  ')
                num_user_failures = 0
            else:
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque
                print("user losing!!", end='  ')

        ######################################################################## Max torque correction
        if (abs(curr_torque) >= abs(max_torque)):
            num_max_torque_times += 1
            print(f"!!!!!! AT MAX TORQUE: TIME {num_max_torque_times}")
            if (num_max_torque_times >= max_max_torque_times_before_user_win): break #User has won!
        else:
            num_max_torque_times = 0
        ######################################################################## Write torque, update position
        print(f"desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f}")
        set_torque(odrv0=odrv0, axis=axis, target_torque=desired_torque)
        last_position = curr_position

        ######################################################################## Plotting

        # Log torque data
        torque_log.append(abs(curr_torque))
        time_log.append(timestamp)
        # Get EMG data from queue
        while not emg_queue.empty():
            emg_t, emg_val = emg_queue.get()
            emg_data.append(emg_val)
            emg_timestamps.append(emg_t)

        # Update live plots
        line_torque.set_xdata(time_log)
        line_torque.set_ydata(torque_log)

        line_emg.set_xdata(emg_timestamps)
        line_emg.set_ydata(emg_data)

        ax_torque.relim()
        ax_torque.autoscale_view()
        ax_emg.relim()
        ax_emg.autoscale_view()
        plt.pause(0.01)

    # Game end condition

    if (num_max_torque_times >= max_max_torque_times_before_user_win):
        print("User has won.")
        user_win = True
    else:
        print("Machine has won.")

    # Final stats
    total_time = time.time() - start_time
    avg_torque = sum(torque_log) / len(torque_log) if torque_log else 0
    plt.ioff()
    plt.close(fig)

    # Reset position
    time.sleep(1)
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)
    set_position(target_position=start_position, axis=axis)

    # Save CSV
    df = pd.DataFrame({'time_sec': time_log, 'torque_Nm': torque_log})
    filename = f"logs/torque_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    # df.to_csv(filename, index=False)
    # print(f"Saved torque log to {filename}")

    # Add subtitle with stats
    subtitle = f"Average Torque: {avg_torque:.2f} Nm | Total Time: {total_time:.2f} s | Mode: {modes[mode_index]}"
    ax_torque.set_title('Torque vs Time\n' + subtitle)

    # Save the figure
    plot_filename = filename.replace(".csv", ".png")
    fig.savefig(plot_filename)
    print(f"Saved torque plot to {plot_filename}")

    return {
        "total_time_sec": total_time,
        "average_torque_Nm": avg_torque,
        "csv_file": filename,
        "plot_file": plot_filename
    }

# Game functions
def fight_user_emg_based(odrv0):
    '''
    If it detects user is flexing hard and arm is not moving, sets torque lower.
    (Flexing strength dependent on average EMG value)
    Streams a plot of torque and emg over time.
    Gentle mode: Decreases torque when user at a standstill
    Else, keeps increasing it
    '''
    # Intialize threshold
    start_relaxing_position_thresh = -0.75
    emg_effort_threshold = 0.2   # CONSTANT: when emg_avg_value > threshold + relaxed_emg_value, ease up

    # Initialize variables
    axis = odrv0.axis0
    curr_torque = axis.controller.effective_torque_setpoint
    curr_current = odrv0.ibus
    curr_position = axis.pos_estimate
    print(f"START: current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
    time.sleep(0.5)

    desired_torque = 0
    last_position = curr_position
    num_user_failures = 0
    num_machine_failures = 0
    num_max_torque_times = 0

    # NEW: Emg average variables
    emg_avg_val_voltages = []
    emg_avg_val_times = []
    relaxed_emg_avg_value = -1
    emg_avg_value = 0
    emg_effort_val = 0

    # Logging torque and EMG
    torque_log = []
    time_log = []
    start_time = time.time()
    # Start EMG thread, passing in start_time
    arduino_port = find_arduino()
    emg_thread = threading.Thread(
        target=read_emg_data,
        kwargs={'serial_port': arduino_port},
        daemon=True
    )
    emg_thread.start()

    # Plot setup
    plt.ion()
    fig, (ax_torque, ax_emg) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    line_torque, = ax_torque.plot([], [], label='Torque (Nm)')
    line_raw, = ax_emg.plot([], [], label="Raw EMG", color='pink', alpha=0.6)
    line_avg, = ax_emg.plot([], [], label="Avg EMG", color='red', linewidth=2)
            # Torque sublot
    ax_torque.set_ylabel('Torque (Nm)')
    ax_torque.set_title('Torque and EMG vs Time')
    ax_torque.grid(True)
    ax_torque.legend()
            # Emg subplot
    ax_emg.set_xlabel('Time (s)')
    ax_emg.set_ylabel('EMG Value')
    ax_emg.grid(True)
    ax_emg.legend()


    # Start with the arm up
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)
    set_position(target_position=start_position, axis=axis)

    # Fight user logic
    while (abs(max_position) - abs(curr_position) > winning_thresh):
        timestamp = time.time() - start_time
        curr_torque = axis.controller.effective_torque_setpoint
        curr_position = axis.pos_estimate
        emg_effort_val = emg_avg_value - relaxed_emg_avg_value
        print(f"EMG: effort = {emg_effort_val} = {emg_avg_value} - {relaxed_emg_avg_value}")

        # USER IS WINNING
        if ((abs(curr_position) == abs(last_position) and emg_effort_val < emg_effort_threshold) 
            or (abs(curr_position) < abs(last_position))
            or (abs(curr_position) < abs(start_relaxing_position_thresh))):

            num_machine_failures += 1
            num_user_failures = 0

            desired_torque = curr_torque + torque_step
            if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

            print(f"user winning!", end='  ')
       
        # USER IS LOSING
        else:
            num_user_failures += 1
            num_machine_failures = 0

            if emg_effort_val > emg_effort_threshold:
                desired_torque = curr_torque - emg_effort_val*torque_step if (abs(curr_torque - emg_effort_val*torque_step) < abs(curr_torque)) else 0
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

                print("user losing !", end='  ')
                num_user_failures = 0
            else:
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque
                print("user losing!!", end='  ')

        ######################################################################## Max torque correction
        if (abs(curr_torque) >= abs(max_torque)):
            num_max_torque_times += 1
            print(f"!!!!!! AT MAX TORQUE: TIME {num_max_torque_times}")
            if (num_max_torque_times >= max_max_torque_times_before_user_win): break #User has won!
        else:
            num_max_torque_times = 0
        ######################################################################## Write torque, update position
        print(f"desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f}")
        set_torque(odrv0=odrv0, axis=axis, target_torque=desired_torque)
        last_position = curr_position

        ######################################################################## Plotting

        # Log torque data
        torque_log.append(abs(curr_torque))
        time_log.append(timestamp)
        
        # Get EMG data from queue
        num_emg_data = 0
        emg_avg_value = 0
        emg_avg_time = 0
        while not emg_queue.empty():
            num_emg_data += 1
            emg_t, emg_val = emg_queue.get()

            # Add to overall emg lists
            emg_data.append(emg_val)
            emg_timestamps.append(emg_t)

            # Calculating average
            emg_avg_time += emg_t
            emg_avg_value += emg_val

        if (num_emg_data != 0):
            emg_avg_time /= num_emg_data
            emg_avg_value /= num_emg_data
            emg_avg_val_times.append(emg_avg_time)
            emg_avg_val_voltages.append(emg_avg_value)

            if (relaxed_emg_avg_value == -1): relaxed_emg_avg_value = emg_avg_value

        # Update live plots
            # Torque plot
        line_torque.set_xdata(time_log)
        line_torque.set_ydata(torque_log)
            # EMG plot
        line_raw.set_xdata(emg_timestamps)
        line_raw.set_ydata(emg_data)
        line_avg.set_xdata(emg_avg_val_times)
        line_avg.set_ydata(emg_avg_val_voltages)

        ax_torque.relim()
        ax_torque.autoscale_view()
        ax_emg.relim()
        ax_emg.autoscale_view()
        plt.pause(0.01)

    # Game end condition

    if (num_max_torque_times >= max_max_torque_times_before_user_win):
        print("User has won.")
        user_win = True
    else:
        print("Machine has won.")

    # Final stats
    total_time = time.time() - start_time
    avg_torque = sum(torque_log) / len(torque_log) if torque_log else 0
    plt.ioff()
    plt.close(fig)

    # Reset position
    time.sleep(1)
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)
    set_position(target_position=start_position, axis=axis)

    # Save CSV
    df = pd.DataFrame({'time_sec': time_log, 'torque_Nm': torque_log})
    filename = f"logs/torque_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    # df.to_csv(filename, index=False)
    # print(f"Saved torque log to {filename}")

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
        "csv_file": filename,
        "plot_file": plot_filename
    }


def constant_torque_mode(odrv0, begin_fight_pos, torque_setpoint = -1):
    '''
    Go to begin_fight_pos and apply constant torque from there.
    '''

    min_dist_from_start = 0.1

    # Lower to the start position, which the user will fight against
    axis = odrv0.axis0
    print("Standby. Reving up to position.")
    set_position(target_position=begin_fight_pos, axis=axis)
    print("Go ahead.")

    curr_torque = axis.controller.effective_torque_setpoint
    curr_current = odrv0.ibus
    curr_position = axis.pos_estimate
    print(f"START: current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
    

    # FIGHT THE USER
    while (abs(curr_position) - abs(start_position) > min_dist_from_start):
        set_torque(odrv0=odrv0, axis=axis, target_torque=torque_setpoint)

        curr_torque = axis.controller.effective_torque_setpoint
        curr_current = odrv0.ibus
        curr_position = axis.pos_estimate

        print(f"current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
        clear_errors(odrv0)
        time.sleep(0.1)

    print("User has won.")
    time.sleep(1)
    set_position(target_position=start_position, axis=axis)
    return


# Constants
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
    print("Stopping motor...")
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)
    set_position(target_position=start_position, axis=axis)
    odrv0.axis0.requested_state = AxisState.IDLE    
except Exception as e:
    print(f"An error occurred: {e}")
    print("Stopping motor...")
    clear_errors(odrv0)
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)
    set_position(target_position=start_position, axis=axis)
    odrv0.axis0.requested_state = AxisState.IDLE


