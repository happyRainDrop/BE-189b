#!/usr/bin/env python3

import time
import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state
import numpy as np

# Helper functions
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
        if verbose:  print(f" -> Torque setpoint: {current_torque:.3f} Nm")
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
    If it detects it is not moving, sets torque higher
    '''

    start_relaxing_position_thresh = -0.5

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
    
    # Start with the arm up
    set_position(target_position=start_position, axis=axis)

    # FIGHT THE USER
    while (abs(max_position) - abs(curr_position) > winning_thresh):
        if (abs(curr_position) < abs(last_position) or abs(curr_position) < abs(start_relaxing_position_thresh)):
            # Oh no! We are loosing! Up the torque!
            num_machine_failures += 1
            num_user_failures = 0
            desired_torque = curr_torque+torque_step*num_machine_failures
            set_torque(odrv0=odrv0, axis=axis, target_torque=desired_torque)
            print(f"    user winning!     desired torque: {desired_torque:.3f} Nm  ||   curr position: {curr_position:.3f}")
        else:
            # We have maintained position or pushed it forward.
            num_user_failures += 1
            num_machine_failures = 0
            if (num_user_failures > max_user_failures_before_easing_up):
                # Ease up on the torque
                desired_torque = curr_torque-torque_step if (abs(curr_torque-torque_step)<abs(curr_torque)) else 0
                set_torque(odrv0=odrv0, axis=axis, target_torque=desired_torque)
                print(f"    user losing!     desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f}")
                num_user_failures = 0
            else:
                # Keep the same torque -- let them fight!
                print(f"    user losing!     desired torque: {desired_torque:.3f} Nm  ||  curr position: {curr_position:.3f} FAIL TIMES {num_user_failures}")
        
        curr_torque = axis.controller.effective_torque_setpoint
        curr_current = odrv0.ibus
        last_position = curr_position
        curr_position = axis.pos_estimate

        print(f"current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_torque:.3f} Nm  ||   current position: {curr_position:.3f}")
        clear_errors(odrv0)

        time.sleep(0.1)

    print("Machine has won.")
    time.sleep(1)
    set_position(target_position=start_position, axis=axis)
    return

def constant_torque_mode(odrv0, begin_fight_pos, torque_setpoint = -1):
    '''
    Go to begin_fight_pos and apply constant torque from there.
    '''

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
    while (abs(curr_position) > curr_position):
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
max_position = -4.20       # hehe funny number but it's true
position_step = -0.1       # position increment (in encoder counts or turns)
torque_step = -0.05         # increment per cycle
pause_between_modes = 0.1  # seconds to pause between mode switches

# Game mode stuff
winning_thresh = 0.25      # within here of the max position, consider the machine to have won
max_user_failures_before_easing_up = 5

try:
    # Setup
    odrv0 = find_odrive()

    # Confirming config settings
    odrv0.axis0.controller.config.vel_limit = 10.0    # Limiting the velocity (rad/s)
    odrv0.axis0.controller.config.vel_gain = 0.1      # Lower vel_gain for smoother motion
    odrv0.axis0.config.motor.current_soft_max = 80    # The Flipsky can take up to 80A 
    odrv0.axis0.config.motor.current_hard_max = 80    
    
    # odrv0.axis0.pos_estimate = 0  # Uncomment to set the current position as 0

    clear_errors(odrv0)
    axis = odrv0.axis0

    # fight_user(odrv0)
    constant_torque_mode(odrv0, begin_fight_pos=max_position, torque_setpoint= -0.3)

    odrv0.axis0.requested_state = AxisState.IDLE

    
except KeyboardInterrupt:
    print("Stopping motor...")
    odrv0.axis0.requested_state = AxisState.IDLE