"""
Example 3 – Basic Configuration
=================================
This example reads the live motor status and then reads the motor configuration
(MCCONF) from the VESC. This is one of the most useful examples when tuning a
motor or verifying that UART communication is healthy.

What you should edit:
    - SERIAL_PORT: set this to the correct serial port for your machine.
    - vesc.set_debug(False): change to True to print the raw UART packets if you
      want debugging information.

What the code is doing:
    1. Opens the serial connection.
    2. Calls get_values() to read live telemetry like input voltage and fault code.
    3. Calls get_mcconf() to read configuration values stored in the VESC.
    4. Prints the Speed PID configuration values: Kp, Ki, Kd, etc.

Important values in this example:
    - input_voltage: battery voltage seen by the VESC
    - controller_id: CAN/controller ID for the VESC
    - fault_code: 0 means no fault
    - s_pid_kp, s_pid_ki, s_pid_kd: speed PID gains that affect motor response

Usage:
    python 3_basic_config.py
"""

import sys
sys.path.append("..")

from vesc_uart import VescUart

# -----------------------------------------------------------------------------
# Edit this to match your actual serial port.
# Windows example: "COM5"
# Linux example: "/dev/ttyUSB0"
# -----------------------------------------------------------------------------
SERIAL_PORT = "COM5"

# Create the VESC object.
# The object uses pyserial to communicate over UART.
vesc = VescUart(SERIAL_PORT)

# Toggle this to True if you want to see raw hex packets sent/received.
# It is useful for debugging framing/CRC issues.
vesc.set_debug(False)

print("=== VESC Basic Configuration Check ===\n")

# -----------------------------------------------------------------------------
# Live status section
# get_values() is the standard command to read the current telemetry.
# -----------------------------------------------------------------------------
if vesc.get_values():
    print("Live Status:")
    print(f"  Input Voltage  : {vesc.data.input_voltage:.1f} V")
    print(f"  Controller ID  : {vesc.data.controller_id}")
    print(f"  Fault Code     : {vesc.data.fault_code} "
          f"({'OK' if vesc.data.fault_code == 0 else 'FAULT'})")
else:
    print("ERROR: Could not reach VESC. Check wiring and baud rate.")

print()

# -----------------------------------------------------------------------------
# MCCONF (motor configuration) section
# get_mcconf() reads the stored motor config from the VESC.
# This is where the PID settings live.
# -----------------------------------------------------------------------------
print("Motor Configuration (Speed PID):")
config = vesc.get_mcconf()

if config:
    # These are the speed loop values that control how the VESC responds to
    # speed errors. Adjusting these is how you tune motor response.
    print(f"  Kp             : {config['s_pid_kp']:.6f}")
    print(f"  Ki             : {config['s_pid_ki']:.6f}")
    print(f"  Kd             : {config['s_pid_kd']:.6f}")
    print(f"  Kd Filter      : {config['s_pid_kd_filter']:.6f}")
    print(f"  Min ERPM       : {config['s_pid_min_erpm']:.1f}")
    print(f"  Allow Braking  : {config['s_pid_allow_braking']}")
else:
    print("  ERROR: Could not read motor configuration.")

vesc.close()