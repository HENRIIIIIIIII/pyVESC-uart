"""
Example 1 – Running the Motor at RPM
=====================================
Connects to VESC, spins the motor to a target electrical RPM,
monitors live telemetry for 5 seconds, then stops the motor cleanly.

This is the simplest "go motor" example. It is useful when you want to test
that the UART connection works, the VESC responds, and the motor can spin.

What you should edit:
    - SERIAL_PORT: change this to your real COM port like "COM5" or your Linux port.
    - TARGET_RPM: this is the target speed in electrical RPM. Higher values = faster.
    - DURATION: how long the motor runs before it is commanded to stop.

What the code is doing:
    1. Create a VESC UART connection on the chosen serial port.
    2. Send a set_rpm command to the VESC.
    3. Poll get_values repeatedly to read RPM, current, and duty.
    4. Stop the motor by sending set_rpm(0).

Useful safety note:
    - Do not use a very high RPM at first. Start low and increase carefully.
    - VESC uses electrical RPM, not mechanical RPM.

Usage:
    python 1_run_at_rpm.py
"""

import time
import sys
sys.path.append("..")   # adjust if vesc_uart.py is in a different location

from vesc_uart import VescUart

# -----------------------------------------------------------------------------
# IMPORTANT: edit this to match your hardware.
# On Windows, this is usually something like "COM3", "COM5", etc.
# On Linux/macOS, this is often "/dev/ttyUSB0" or "/dev/ttyACM0".
# -----------------------------------------------------------------------------
SERIAL_PORT = "COM5"

# Target speed setting for the motor in electrical RPM.
# Example: 3000 means the VESC tries to spin the motor at 3000 ERPM.
TARGET_RPM = 5000

# How long to keep the motor running before stopping it.
DURATION = 2

# Create the UART connection to the VESC.
# This is the main object used for all the commands below.
vesc = VescUart(SERIAL_PORT)

print(f"Spinning motor to {TARGET_RPM} ERPM...")
vesc.set_rpm(TARGET_RPM)

start = time.time()
while time.time() - start < DURATION:
    # get_values() asks the VESC for telemetry. The returned data is stored in
    # vesc.data, and you can inspect fields like rpm, avg_motor_current, etc.
    if vesc.get_values():
        print(f"  RPM: {vesc.data.rpm:>8.0f}  |  "
              f"Current: {vesc.data.avg_motor_current:>6.2f} A  |  "
              f"Duty: {vesc.data.duty_cycle_now * 100:>5.1f}%")
    else:
        print("  -- No response from VESC --")
    time.sleep(0.5)

print("Stopping motor...")
# motor stop command: set_rpm(0) tells the VESC to stop driving the motor.
vesc.set_rpm(0)
time.sleep(1)
vesc.close()