"""
Example 2 – Running the Motor at Duty Cycle
=============================================
This example ramps the motor from 0% duty cycle to 50% duty cycle in steps.
It is useful when you want to test how the motor behaves as the PWM output
increases over time.

What you should edit:
    - SERIAL_PORT: your COM port, like "COM5".
    - The loop range(0, 55, 5): this controls the duty steps.
      It currently does 0%, 5%, 10%, ..., 50%.
    - The wait time after each duty step: time.sleep(0.4)

What the code is doing:
    1. Opens the UART connection to the VESC.
    2. Sends set_duty(...) commands to increase the PWM duty cycle gradually.
    3. Waits briefly for the motor to respond.
    4. Reads back telemetry to see the actual duty and RPM.
    5. Stops the motor by sending a 0 duty command.

Important safety notes:
    - Duty cycle is a percentage of the PWM command, not an RPM value.
    - Use gradual changes. Large jumps can cause faults or overcurrent.

Usage:
    python 2_run_at_duty_cycle.py
"""

import time
import sys
sys.path.append("..")

from vesc_uart import VescUart

# -----------------------------------------------------------------------------
# Edit this to your VESC serial connection.
# On Windows, usually "COM3", "COM5", etc.
# -----------------------------------------------------------------------------
SERIAL_PORT = "COM5"

# Open the connection to the VESC.
# The VescUart object exposes commands like set_duty(), set_rpm(), get_values().
vesc = VescUart(SERIAL_PORT)

print("Ramping duty cycle from 0% to 50%...")

# Loop through duty percentages.
# percent is the commanded duty value in percent.
# Example: 25 -> duty = 25/100.0 = 0.25
for percent in range(0, 55, 5):       # 0%, 5%, 10%, ..., 50%
    duty = percent / 100.0

    # This command tells the VESC how much duty cycle to use.
    # 0.0 == 0%, 0.5 == 50%, etc.
    vesc.set_duty(duty)

    # Wait a moment so the motor can react before reading telemetry.
    time.sleep(0.4)

    # Read back current telemetry. This is useful to compare the requested duty to
    # the actual duty cycle reported by the VESC.
    if vesc.get_values():
        print(f"  Duty set: {percent:>3d}%  |  "
              f"Actual duty: {vesc.data.duty_cycle_now * 100:>5.1f}%  |  "
              f"RPM: {vesc.data.rpm:>7.0f}")
    else:
        print(f"  Duty set: {percent:>3d}%  |  -- No response --")

print("Stopping motor (duty = 0)...")
# Command the VESC to stop driving the motor by setting duty to zero.
vesc.set_duty(0.0)
time.sleep(1)
vesc.close()