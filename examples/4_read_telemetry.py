"""
Example 4 – Reading and Analyzing Telemetry Values
====================================================
This is the live-monitoring example. It repeatedly requests telemetry and prints
it in a table.

What you should edit:
    - SERIAL_PORT: set this to your COM port or Linux serial port.
    - MONITOR_RPM: if you want the VESC to spin during monitoring, set this to a
      positive ERPM value. Set it to 0 if you only want to monitor without driving.
    - POLL_INTERVAL: how often to request fresh values (seconds).

What the code is doing:
    1. Opens a serial connection to the VESC.
    2. Optionally sends a set_rpm command to run the motor.
    3. Enters a loop that calls get_values() repeatedly.
    4. Prints temperature, RPM, duty, current, voltage, and fault state.
    5. Stops the motor when the script exits.

Important telemetry values that you can monitor live:
    - rpm: electrical RPM
    - duty_cycle_now: current duty cycle
    - avg_motor_current: motor current in amps
    - input_voltage: supply battery voltage
    - temp_mosfet / temp_motor: temperatures
    - fault_code: active fault flag, if any

Usage:
    python 4_read_telemetry.py
"""

import time
import sys
sys.path.append("..")

from vesc_uart import VescUart, FaultCode

# -----------------------------------------------------------------------------
# Most important settings in this example:
# - SERIAL_PORT: choose your VESC COM port, e.g. "COM5"
# - MONITOR_RPM: positive number means make the VESC spin while monitoring
# - POLL_INTERVAL: how often to poll the VESC (seconds)
# -----------------------------------------------------------------------------
SERIAL_PORT  = "COM5"   # Windows: COM5
MONITOR_RPM  = 2000             # ERPM to spin during monitoring (set 0 to monitor without running)
POLL_INTERVAL = 0.5             # seconds between readings

# Mapping each fault code to a readable name.
# This is only for display in the console. It does not control the VESC.
FAULT_NAMES = {
    FaultCode.FAULT_CODE_NONE            : "None",
    FaultCode.FAULT_CODE_OVER_VOLTAGE    : "Over Voltage",
    FaultCode.FAULT_CODE_UNDER_VOLTAGE   : "Under Voltage",
    FaultCode.FAULT_CODE_DRV             : "DRV Fault",
    FaultCode.FAULT_CODE_ABS_OVER_CURRENT: "Abs Over Current",
    FaultCode.FAULT_CODE_OVER_TEMP_FET   : "Over Temp FET",
    FaultCode.FAULT_CODE_OVER_TEMP_MOTOR : "Over Temp Motor",
}

# Connect to the VESC.
vesc = VescUart(SERIAL_PORT)

# If MONITOR_RPM is set, command the VESC to spin at that speed.
# Otherwise we just read telemetry without commanding a target speed.
if MONITOR_RPM:
    vesc.set_rpm(MONITOR_RPM)
    print(f"Motor set to {MONITOR_RPM} ERPM. Press Ctrl+C to stop.\n")
else:
    print("Monitoring in passive mode (motor not driven). Press Ctrl+C to stop.\n")

print(f"{'Time':>6} | {'RPM':>7} | {'Duty':>6} | {'Mot A':>5} | "
      f"{'In A':>5} | {'Volt':>5} | {'FET°C':>5} | {'Mot°C':>5} | {'Ah Used':>7} | Fault")
print("-" * 95)

try:
    elapsed = 0.0
    while True:
        # Request telemetry every loop and print the values.
        if vesc.get_values():
            d = vesc.data
            fault_name = FAULT_NAMES.get(d.fault_code, f"Code {d.fault_code}")
            warning = " ⚠" if d.fault_code != 0 or d.temp_mosfet > 70 or d.temp_motor > 80 else ""

            print(f"{elapsed:>5.1f}s | "
                  f"{d.rpm:>7.0f} | "
                  f"{d.duty_cycle_now*100:>5.1f}% | "
                  f"{d.avg_motor_current:>5.2f} | "
                  f"{d.avg_input_current:>5.2f} | "
                  f"{d.input_voltage:>5.1f} | "
                  f"{d.temp_mosfet:>5.1f} | "
                  f"{d.temp_motor:>5.1f} | "
                  f"{d.amp_hours:>7.3f} | "
                  f"{fault_name}{warning}")
        else:
            print(f"{elapsed:>5.1f}s | -- No response from VESC --")

        time.sleep(POLL_INTERVAL)
        elapsed += POLL_INTERVAL

except KeyboardInterrupt:
    print("\nMonitoring stopped by user.")

finally:
    # Always stop the motor at the end so it does not keep running.
    vesc.set_rpm(0)
    time.sleep(0.5)
    vesc.close()