"""
Set PID values on a VESC over UART interactively.

This script is a user-friendly version of the tuning example. It asks you to
enter values, confirms them, writes them to the VESC, and can optionally verify
by reading the config back.

What you should edit:
    - default_port = "COM5": change the default serial port if needed.
    - The prompts for Kp, Ki, Kd: these are the values you want to write.

What the script is doing:
    1. Asks for the serial port.
    2. Prompts for Kp, Ki, Kd values.
    3. Reads the current config before writing.
    4. Calls set_kp(), set_ki(), and set_kd().
    5. Optionally reboots the VESC for configuration to apply.
    6. Optionally reads the config again to verify the values.

Usage:
    python set_pid.py
"""
import os
import sys
import time

# Make project root importable.
# This allows the script to import vesc_uart.py when run from the examples folder.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from vesc_uart import VescUart


def ask(prompt, default=None, cast=str):
    """Prompt the user for input and cast it to the requested type."""
    raw = input(f"{prompt} [{'Enter' if default is None else default}]: ").strip()
    if raw == "":
        return default
    try:
        return cast(raw)
    except Exception:
        print("Invalid value, using default.")
        return default


def main():
    print("Set PID values on VESC\n")

    # Default Windows port. Change if yours is different.
    default_port = "COM5"
    port = input(f"Serial port [{default_port}]: ").strip() or default_port

    # These are the tuning values we are about to write to the VESC.
    kp = ask("New Kp", 0.005, float)
    ki = ask("New Ki", 0.003, float)
    kd = ask("New Kd", 0.00005, float)

    print("\nSummary:")
    print(f"  Port: {port}")
    print(f"  Kp: {kp}")
    print(f"  Ki: {ki}")
    print(f"  Kd: {kd}")

    confirm = input("Proceed to write these values to the VESC? (yes/NO): ").strip().lower()
    if confirm not in ('y', 'yes'):
        print("Aborted by user.")
        return

    vesc = None
    try:
        vesc = VescUart(port)

        print("Reading current configuration...")
        cfg = vesc.get_mcconf()
        if cfg:
            print(f"Current Kp={cfg.get('s_pid_kp'):.6f}, Ki={cfg.get('s_pid_ki'):.6f}, Kd={cfg.get('s_pid_kd'):.6f}")
        else:
            print("Warning: could not read current configuration before write.")

        # These are the actual writes to the VESC config.
        print("Writing Kp...")
        ok_kp = vesc.set_kp(kp)
        print("Writing Ki...")
        ok_ki = vesc.set_ki(ki)
        print("Writing Kd...")
        ok_kd = vesc.set_kd(kd)

        print(f"Results: Kp={'OK' if ok_kp else 'FAIL'}, Ki={'OK' if ok_ki else 'FAIL'}, Kd={'OK' if ok_kd else 'FAIL'}")

        # Many VESC setups require a reboot to load the setting into runtime memory.
        reboot = input("Reboot VESC to apply changes now? (yes/NO): ").strip().lower()
        if reboot in ('y', 'yes'):
            print("Rebooting VESC (it will disconnect/reconnect)...")
            vesc._reboot_device(wait_seconds=1.0)

        verify = input("Verify by reading back values? (yes/NO): ").strip().lower()
        if verify in ('y', 'yes'):
            time.sleep(0.6)
            new_cfg = vesc.get_mcconf()
            if new_cfg:
                print("Verified values:")
                print(f"  Kp = {new_cfg.get('s_pid_kp'):.6f}")
                print(f"  Ki = {new_cfg.get('s_pid_ki'):.6f}")
                print(f"  Kd = {new_cfg.get('s_pid_kd'):.6f}")
            else:
                print("Failed to read configuration for verification.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


if __name__ == '__main__':
    main()
