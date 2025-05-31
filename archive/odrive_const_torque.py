import odrive
from odrive.enums import *
import time
import numpy as np

print("Searching for ODrive...")
odrv0 = odrive.find_sync()
print(f"ODrive found! Firmware Version: {odrv0.fw_version_major}.{odrv0.fw_version_minor}.{odrv0.fw_version_revision}")

# Ensure the axis is in closed-loop control
print("Setting up axis...")
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

# no brake resistor
odrv0.clear_errors()
time.sleep(1)

# Set the control mode to torque control
odrv0.axis0.controller.config.control_mode = ControlMode.TORQUE_CONTROL

# Set current limits for safety
odrv0.axis0.config.motor.current_soft_max = 18  # Max current in Amps


# Constants
start_position = 0.0
max_position = -4.40       # hehe funny number but it's true
position_step = -0.1       # position increment (in encoder counts or turns)
max_torque = -18            # target torque (Amps)
torque_step = -0.05         # increment per cycle
pause_between_modes = 0.1  # seconds to pause between mode switches

# Apply constant torque command (e.g., 1.0 Nm equivalent)
constant_current_goal = -5  # Amps
constant_current_setpoint = 0 # Amps
constant_current_step_size = -0.05
t0 = time.monotonic()
# Slow ramp up, stop ramping up if user loosing badly
while (np.abs(constant_current_setpoint) < np.abs(constant_current_goal)):
    odrv0.clear_errors()
    if (time.monotonic() - t0 > 0.1): 
        constant_current_setpoint += constant_current_step_size
        t0 = time.monotonic()
        print(f"Applying constant current: {constant_current_setpoint} A")
        odrv0.axis0.controller.input_torque = constant_current_setpoint

try:
    while True:
        axis = odrv0.axis0
        print(f"Current torque: {odrv0.axis0.controller.input_torque:.2f} A")
        curr_torque = axis.controller.effective_torque_setpoint
        curr_current = odrv0.ibus
        curr_position = axis.pos_estimate

        print(f"current on DC bus: {curr_current:.3f} A  ||   effective torque: {curr_current:.3f} Nm  ||   current position: {curr_position:.3f}")

        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping motor...")
    odrv0.axis0.requested_state = AxisState.IDLE