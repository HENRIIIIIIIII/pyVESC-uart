"""
Example 5 – Advanced Configuration (Speed PID Tuning)
=======================================================
This example reads the VESC configuration, then writes new Kp/Ki/Kd values to
Speed PID settings and verifies them.

What you should edit:
    - SERIAL_PORT: set to the correct COM port.
    - DEBUG: set to True to print every raw UART packet for debugging.
    - NEW_KP / NEW_KI / NEW_KD: change these to the values you want to test.

What the code is doing:
    1. Connects to the VESC over UART.
    2. Reads the current config using get_mcconf().
    3. Prints the current Kp/Ki/Kd values.
    4. Calls set_kp(), set_ki(), set_kd() to write new values.
    5. Reads the config again to verify the write.

Important notes:
    - Kp, Ki, and Kd are speed-loop tuning parameters.
    - Too high: may cause oscillation or jitter.
    - Too low: motor response may feel weak or sluggish.
    - Typical ranges are around 0.0001 to 0.01, but exact values depend on the
      motor and setup.

Usage:
    python 5_advanced_pid_tuning.py
"""

import time
import sys
sys.path.append("..")

from vesc_uart import VescUart

# -----------------------------------------------------------------------------
# Edit these values before running.
# SERIAL_PORT must match your VESC serial port.
# DEBUG True is useful when debugging packet framing / CRC / bad writes.
# -----------------------------------------------------------------------------
SERIAL_PORT = "COM5"
DEBUG = False

# New PID values to try.
# These are the numbers that will be written to the VESC.
NEW_KP = 0.005000
NEW_KI = 0.003000
NEW_KD = 0.000050

vesc = VescUart(SERIAL_PORT)
vesc.set_debug(DEBUG)

print("=== Speed PID Tuning ===\n")

# -----------------------------------------------------------------------------
# Step 1: read current values
# get_mcconf() returns a dictionary with the current VESC config values.
# -----------------------------------------------------------------------------
config = vesc.get_mcconf()
if config:
    print("Current PID gains:")
    print(f"  Kp = {config['s_pid_kp']:.6f}")
    print(f"  Ki = {config['s_pid_ki']:.6f}")
    print(f"  Kd = {config['s_pid_kd']:.6f}\n")
else:
    print("ERROR: Failed to read configuration. Is VESC in UART app mode?")
    vesc.close()
    sys.exit(1)

# -----------------------------------------------------------------------------
# Step 2: write new values
# set_kp(), set_ki(), and set_kd() each read the current config, modify the
# relevant float value, and write the full config back to the VESC.
# -----------------------------------------------------------------------------
print(f"Applying new gains: Kp={NEW_KP}, Ki={NEW_KI}, Kd={NEW_KD}")

ok_kp = vesc.set_kp(NEW_KP)
ok_ki = vesc.set_ki(NEW_KI)
ok_kd = vesc.set_kd(NEW_KD)

print(f"  set_kp result: {'OK' if ok_kp else 'FAILED'}")
print(f"  set_ki result: {'OK' if ok_ki else 'FAILED'}")
print(f"  set_kd result: {'OK' if ok_kd else 'FAILED'}")

# -----------------------------------------------------------------------------
# Step 3: verify that the write took effect.
# We read the config again and compare the returned values to the target values.
# -----------------------------------------------------------------------------
time.sleep(0.3)
verify = vesc.get_mcconf()

if verify:
    print("\nVerified gains after write:")
    for label, target, key in [
        ("Kp", NEW_KP, "s_pid_kp"),
        ("Ki", NEW_KI, "s_pid_ki"),
        ("Kd", NEW_KD, "s_pid_kd"),
    ]:
        actual = verify[key]
        match  = "✓" if abs(actual - target) < 1e-6 else "✗ MISMATCH"
        print(f"  {label} = {actual:.6f}  (target: {target:.6f})  {match}")
else:
    print("ERROR: Could not verify written configuration.")

vesc.close()