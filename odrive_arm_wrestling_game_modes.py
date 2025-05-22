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

emg_data = []
emg_timestamps = []
emg_queue = queue.Queue()


# Helper functions: Arduino

def find_arduino(baud_rate=9600, timeout=2):
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

def fight_user(odrv0):
    '''
    If it detects it is not moving, sets torque higher.
    Streams a plot of torque over time and saves torque log.
    Gentle mode: Decreases torque when user at a standstill
    Else, keeps increasing it
    '''
    start_relaxing_position_thresh = -0.75
    modes = ["Gentle", "Hardcore"]
    mode_index = 1
    user_win = False

    axis = odrv0.axis0
    curr_torque = axis.controller.effective_torque_setpoint
    curr_current = odrv0.ibus
    curr_position = axis.pos_estimate
    print(f"START: current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
    time.sleep(0.5)

    desired_torque = curr_torque
    last_position = curr_position
    num_user_failures = 0
    num_machine_failures = 0
    num_max_torque_times = 0

    torque_log = []
    time_log = []
    start_time = time.time()

    # Plot setup
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], label='Torque (Nm)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Effective Torque (Nm)')
    ax.set_title('Torque vs Time')
    ax.grid(True)
    ax.legend()

    # Start with the arm up
    set_position(target_position=start_position, axis=axis)
    set_torque(odrv0=odrv0, axis=axis, target_torque=0)

    while (abs(max_position) - abs(curr_position) > winning_thresh):
        timestamp = time.time() - start_time
        curr_torque = axis.controller.effective_torque_setpoint
        curr_position = axis.pos_estimate

        if ((abs(curr_position) < abs(last_position) and mode_index == 0)
            or (abs(curr_position) <= abs(last_position) and mode_index == 1) 
            or (abs(curr_position) < abs(start_relaxing_position_thresh))):

            num_machine_failures += 1
            num_user_failures = 0

            desired_torque = curr_torque + torque_step * num_machine_failures
            if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

            print(f"    user winning!     desired torque: {desired_torque:.3f} Nm  ||   curr position: {curr_position:.3f}")
       
        else:
            num_user_failures += 1
            num_machine_failures = 0
            if num_user_failures > max_user_failures_before_easing_up:
                desired_torque = curr_torque - torque_step if (abs(curr_torque - torque_step) < abs(curr_torque)) else 0
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque

                print(f"    user losing!     desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f}")
                num_user_failures = 0
            else:
                if (abs(desired_torque) >= abs(max_torque)): desired_torque = max_torque
                print(f"    user losing!     desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f} FAIL TIMES {num_user_failures}")

        ######################################################################## Max torque correction
        if (abs(curr_torque) >= abs(max_torque)):
            num_max_torque_times += 1
            print(f"!!!!!! AT MAX TORQUE: TIME {num_max_torque_times}")
            if (num_max_torque_times >= max_max_torque_times_before_user_win): break #User has won!
        else:
            num_max_torque_times = 0
        ######################################################################## Write torque, update position
        set_torque(odrv0=odrv0, axis=axis, target_torque=desired_torque)
        last_position = curr_position

        # Log torque data
        torque_log.append(abs(curr_torque))
        time_log.append(timestamp)

        # Update live plot
        line.set_xdata(time_log)
        line.set_ydata(torque_log)
        ax.relim()
        ax.autoscale_view()
        plt.pause(0.01)

        print(f"current on DC bus: {odrv0.ibus:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
        clear_errors(odrv0)
        time.sleep(0.1)

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
    set_position(target_position=start_position, axis=axis)

    # Save CSV
    df = pd.DataFrame({'time_sec': time_log, 'torque_Nm': torque_log})
    filename = f"logs/torque_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    df.to_csv(filename, index=False)
    print(f"Saved torque log to {filename}")

    # Add subtitle with stats
    subtitle = f"Average Torque: {avg_torque:.2f} Nm | Total Time: {total_time:.2f} s | Mode: {modes[mode_index]}"
    ax.set_title('Torque vs Time\n' + subtitle)

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

    fight_user(odrv0)
    # constant_torque_mode(odrv0, begin_fight_pos=max_position, torque_setpoint= -0.35)

    odrv0.axis0.requested_state = AxisState.IDLE

except KeyboardInterrupt:
    print("Stopping motor...")
    odrv0.axis0.requested_state = AxisState.IDLE    
except Exception as e:
    print(f"An error occurred: {e}")
    print("Stopping motor...")
    clear_errors(odrv0)
    odrv0.axis0.requested_state = AxisState.IDLE