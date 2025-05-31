#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""
import math
import time

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state

def find_odrive():
    # Find a connected ODrive (this will block until you connect one)
    print("waiting for ODrive...")
    odrv0 = odrive.find_sync()
    print(f"found ODrive {odrv0._dev.serial_number}")
    return odrv0

def clear_errors(odrv0):
# Check for errors
    if odrv0.axis0.active_errors != 0:
        print("Errors found on axis0: 0x{:04X}".format(odrv0.axis0.active_errors))
    # Clear all errors on the entire ODrive
    odrv0.clear_errors()
    print("All errors cleared on ODrive")

odrv0 = find_odrive()
clear_errors(odrv0)

# Enter closed loop control
odrv0.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
request_state(odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)

# Run a sine wave until ODrive reports an error or user hits Ctrl+C
try:
    p0 = odrv0.axis0.controller.input_pos
    setpoint = p0
    t0 = time.monotonic()
    while odrv0.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
        setpoint -= 0.1
        
        print(f"Actual position: {odrv0.axis0.pos_estimate:.3f}, Setpoint: {setpoint:.3f}")
        odrv0.axis0.controller.input_pos = setpoint
        setpoint = odrv0.axis0.pos_estimate
        time.sleep(0.01)

finally:
    request_state(odrv0.axis0, AxisState.IDLE)

# Show errors
dump_errors(odrv0)



