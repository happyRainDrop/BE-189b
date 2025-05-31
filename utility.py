from scipy.signal import butter, lfilter
import serial
import serial.tools.list_ports
import time
import sys
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state

# bytes to signal the Arduino to start streaming
STREAM_CMD = bytes([3])

# start time of Arduino streaming
start_time = None

# ==============================================================================
# Filtering Utility Functions
# ==============================================================================

# Butterworth Filter Setup
def butter_lowpass(cutoff, fs, order=4):
    # Nyquist frequency
    nyq = 0.5 * fs 
    # cutoff frequency normalized to Nyquist frequency
    normal_cutoff = cutoff / nyq
    return butter(order, normal_cutoff, btype='low', analog=False)

# applies butterworth filter
def apply_filter(data, b, a):
    return lfilter(b, a, data)

# ==============================================================================
# Arduino Utility Functions
# ==============================================================================

# find arduino device connected to the computer
def find_arduino():
    # Known Arduino/CH340/Silabs VIDs (vendor IDs)
    arduino_vids = ["2341", "1A86", "10C4", "2A03"]

    # for each port on the computer, check if any have an arduino device connected
    # if Arduino device found, return port else return None
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if (port.vid and f"{port.vid:04X}" in arduino_vids) or "Arduino" in port.description:
            return port.device
    return None

# read EMG data from Arduino
def read_serial_data(port, data_queue, BAUD_RATE=115200):
    global start_time
    try:
        # open a serial connection and write bytes to signal the Arduino to start streaming
        with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
            print(f"[INFO] Connected to {port}")
            time.sleep(2)
            ser.write(STREAM_CMD)

            while True:
                # read a line from the serial connection
                line = ser.readline().decode('utf-8').strip()

                # if the line does not contain data, continue
                if ',' not in line:
                    continue
                try:
                    # parse the Arduino's EMG data
                    t_str, v_str = line.split(',')
                    t = float(t_str) / 1000.0
                    v = float(v_str)

                    # store the time of the first response as our start time
                    if start_time is None:
                        start_time = t
                        continue

                    # add the EMG data to the data queue
                    data_queue.put((t - start_time, v))

                # if the data isn't a float, just continue (will be handled below)
                except ValueError:
                    continue

    # error handling
    except Exception as e:
        print(f"[ERROR] Serial read failed: {e}")
        sys.exit(1)

# ==============================================================================
# ODrive Utility Functions
# ==============================================================================

# find odrive motor controller connected to the computer
def find_odrive():
    print("Waiting for ODrive...")
    odrv0 = odrive.find_sync()
    print(f"Found ODrive {odrv0._dev.serial_number}")
    return odrv0

# clear ODrive errors
def clear_errors(odrv0):
    if odrv0.axis0.active_errors != 0:
        dump_errors(odrv0)
    odrv0.clear_errors()

# set torque on ODrive motor controller with ramp-up
def set_torque(odrv0, target_torque, torque_step=0.01, torque_ramp_delay=0.05, max_power_supply_curr = 15, verbose = False):
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

    axis = odrv0.axis0

    # get the current torque setpoint of the motor controller
    current_torque = axis.controller.effective_torque_setpoint
    # keep track of the number of failures to set the current
    num_current_set_failures = 0

    # set the motor controller to torque control mode
    if verbose: print(f"[Torque Ramp] Switching to TORQUE_CONTROL with goal {target_torque} starting from {current_torque}")
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    request_state(axis, AxisState.CLOSED_LOOP_CONTROL)
    time.sleep(0.1)

    # while we're not at our target torque (abs enables control of both motor directions)
    while abs(current_torque - target_torque) > abs(torque_step):
        # step the current torque towards the target torque
        current_torque = (current_torque + torque_step) if target_torque > current_torque else (current_torque - torque_step)
        if verbose: print(f" -> Commanded Torque: {axis.controller.input_torque}, Effective Torque: {axis.controller.effective_torque_setpoint}")
        # if the current exceeds the limit, break
        if (odrv0.ibus >= max_power_supply_curr): break
        # set the controller torque to the current torque
        axis.controller.input_torque = current_torque

        # if the controller fails to set the torque to our current torque 5 times consecutively, quit
        if ((current_torque < target_torque and axis.controller.effective_torque_setpoint < current_torque) or 
            (current_torque > target_torque and axis.controller.effective_torque_setpoint > current_torque)):
            num_current_set_failures += 1
        else:
            num_current_set_failures = 0
        if (num_current_set_failures > 5):
            if verbose: print("Failed to set the torque to the setpoint.")
            break

        # update the current torque to the motor controller's effective torque setpoint
        # note that the effective torque setpoint may be different from the current_torque setpoint
        current_torque = axis.controller.effective_torque_setpoint
        if verbose:  print(f" -> Torque: {current_torque:.3f} Nm")
        time.sleep(torque_ramp_delay)

    # Final torque set (if doesn't exceed power supply limit)
    if (odrv0.ibus <= max_power_supply_curr): axis.controller.input_torque = target_torque
    if verbose:  
        print(f" -> Final Torque Setpoint: {target_torque:.2f} Nm, actual {axis.controller.effective_torque_setpoint}")

# set the position of the motor
def set_position(target_position, axis, position_step = 0.05, position_ramp_delay=0.02, verbose = False):
    # enter position control mode for the motor
    axis.controller.config.input_mode = InputMode.PASSTHROUGH
    axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
    request_state(axis, AxisState.CLOSED_LOOP_CONTROL)

    # get the current position of the motor
    if verbose: print(f"Going to start position at {target_position}")
    curr_position = axis.pos_estimate

    # while the current position is above the target position
    while(curr_position > target_position):
        # decrement the motor position by a step
        curr_position -= position_step
        axis.controller.input_pos = curr_position
        # update the current position
        curr_position = axis.pos_estimate
        # wait for delay
        time.sleep(position_ramp_delay)

    # while the current position is below the target position
    while(curr_position < target_position):
        # increment the motor position by a step
        curr_position += position_step
        axis.controller.input_pos = curr_position
        # update the current position
        curr_position = axis.pos_estimate
        # wait for delay
        time.sleep(position_ramp_delay)

    # after the while loops, we end up close enough to the target and we
    # can just set the motor position to be the target position
    axis.controller.input_pos = target_position