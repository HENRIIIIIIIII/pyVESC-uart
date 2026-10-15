"""
Interactive example runner for pyVESC-uart

Run this from the `examples/` folder or from the project root. It will
prompt for a serial port and example-specific parameters, then execute
one of the six provided examples using `VescUart`.

Usage:
    python run_example.py

"""
import os
import sys
import time

# Ensure project root is importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from vesc_uart import VescUart
from benchmark_pid_response import main as run_benchmark

EXAMPLES = {
    '1': 'Run at RPM (spin motor for a short duration)',
    '2': 'Run at Duty Cycle (ramp duty from 0% to 50%)',
    '3': 'Basic Config (read live status and PID)',
    '4': 'Read Telemetry (continuous monitor until Ctrl+C)',
    '5': 'Advanced PID Tuning (read/write/verify PID gains)',
    '6': 'Set PID (write values only)',
    '7': 'PID Benchmark (before/after response test)'
}


def ask(prompt, default=None, cast=str):
    val = input(f"{prompt} [{'Enter' if default is None else default}]: ").strip()
    if val == "":
        return default
    try:
        return cast(val)
    except Exception:
        print("Invalid input, using default.")
        return default


def read_current_pid_values(port):
    """Read the PID values currently on the VESC and fall back to known defaults."""
    fallback = (0.005000, 0.003000, 0.000050)
    vesc = None
    try:
        vesc = VescUart(port)
        config = vesc.get_mcconf()
        if config:
            return float(config.get('s_pid_kp', fallback[0])), float(config.get('s_pid_ki', fallback[1])), float(config.get('s_pid_kd', fallback[2]))
    except Exception as exc:
        print(f"Warning: could not read current PID values from {port}: {exc}")
    finally:
        if vesc:
            vesc.close()
    return fallback


def run_example_1(port, target_rpm=3000, duration=2):
    vesc = None
    try:
        vesc = VescUart(port)
        print(f"Spinning motor to {target_rpm} ERPM...")
        vesc.set_rpm(target_rpm)
        start = time.time()
        while time.time() - start < duration:
            if vesc.get_values():
                d = vesc.data
                print(f"  RPM: {d.rpm:>8.0f}  |  Current: {d.avg_motor_current:>6.2f} A  |  Duty: {d.duty_cycle_now*100:>5.1f}%")
            else:
                print("  -- No response from VESC --")
            time.sleep(0.5)
        print("Stopping motor...")
        vesc.set_rpm(0)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


def run_example_2(port):
    vesc = None
    try:
        vesc = VescUart(port)
        print("Ramping duty cycle from 0% to 50%...")
        for percent in range(0, 55, 5):
            duty = percent / 100.0
            vesc.set_duty(duty)
            time.sleep(0.4)
            if vesc.get_values():
                print(f"  Duty set: {percent:>3d}%  |  Actual duty: {vesc.data.duty_cycle_now*100:>5.1f}%  |  RPM: {vesc.data.rpm:>7.0f}")
            else:
                print(f"  Duty set: {percent:>3d}%  |  -- No response --")
        print("Stopping motor (duty = 0)...")
        vesc.set_duty(0.0)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


def run_example_3(port):
    vesc = None
    try:
        vesc = VescUart(port)
        vesc.set_debug(False)
        print("=== VESC Basic Configuration Check ===\n")
        if vesc.get_values():
            print(f"  Input Voltage  : {vesc.data.input_voltage:.1f} V")
            print(f"  Controller ID  : {vesc.data.controller_id}")
            print(f"  Fault Code     : {vesc.data.fault_code} ({'OK' if vesc.data.fault_code == 0 else 'FAULT'})")
        else:
            print("ERROR: Could not reach VESC. Check wiring and baud rate.")

        print()
        print("Motor Configuration (Speed PID):")
        config = vesc.get_mcconf()
        if config:
            print(f"  Kp             : {config['s_pid_kp']:.6f}")
            print(f"  Ki             : {config['s_pid_ki']:.6f}")
            print(f"  Kd             : {config['s_pid_kd']:.6f}")
        else:
            print("  ERROR: Could not read motor configuration.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


def run_example_4(port, monitor_rpm=2000, poll_interval=0.5):
    vesc = None
    try:
        vesc = VescUart(port)
        if monitor_rpm:
            vesc.set_rpm(monitor_rpm)
            print(f"Motor set to {monitor_rpm} ERPM. Press Ctrl+C to stop.\n")
        else:
            print("Monitoring in passive mode (motor not driven). Press Ctrl+C to stop.\n")

        elapsed = 0.0
        while True:
            if vesc.get_values():
                d = vesc.data
                print(f"{elapsed:>5.1f}s | RPM: {d.rpm:>7.0f} | Duty: {d.duty_cycle_now*100:>5.1f}% | In A: {d.avg_input_current:>5.2f} | Volt: {d.input_voltage:>5.1f}")
            else:
                print(f"{elapsed:>5.1f}s | -- No response from VESC --")
            time.sleep(poll_interval)
            elapsed += poll_interval
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            try:
                vesc.set_rpm(0)
                time.sleep(0.5)
            except Exception:
                pass
            vesc.close()


def run_example_5(port, new_kp=0.005, new_ki=0.003, new_kd=0.00005, debug=False):
    vesc = None
    try:
        vesc = VescUart(port)
        vesc.set_debug(debug)
        print("=== Speed PID Tuning ===\n")
        config = vesc.get_mcconf()
        if not config:
            print("ERROR: Failed to read configuration. Is VESC in UART app mode?")
            return
        print("Current PID gains:")
        print(f"  Kp = {config['s_pid_kp']:.6f}")
        print(f"  Ki = {config['s_pid_ki']:.6f}")
        print(f"  Kd = {config['s_pid_kd']:.6f}\n")

        print(f"Applying new gains: Kp={new_kp}, Ki={new_ki}, Kd={new_kd}")
        ok_kp = vesc.set_kp(new_kp)
        ok_ki = vesc.set_ki(new_ki)
        ok_kd = vesc.set_kd(new_kd)
        print(f"  set_kp result: {'OK' if ok_kp else 'FAILED'}")
        print(f"  set_ki result: {'OK' if ok_ki else 'FAILED'}")
        print(f"  set_kd result: {'OK' if ok_kd else 'FAILED'}")

        time.sleep(0.3)
        verify = vesc.get_mcconf()
        if verify:
            print("\nVerified gains after write:")
            for label, target, key in [
                ("Kp", new_kp, "s_pid_kp"),
                ("Ki", new_ki, "s_pid_ki"),
                ("Kd", new_kd, "s_pid_kd"),
            ]:
                actual = verify[key]
                match = 'OK' if abs(actual - target) < 1e-6 else 'MISMATCH'
                print(f"  {label} = {actual:.6f}  (target: {target:.6f})  {match}")
        else:
            print("ERROR: Could not verify written configuration.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


def run_example_6(port, kp, ki, kd):
    vesc = None
    try:
        vesc = VescUart(port)
        vesc.set_debug(False)
        print("Writing new PID values to VESC...\n")
        print(f"  Kp = {kp}")
        print(f"  Ki = {ki}")
        print(f"  Kd = {kd}")

        print("  Writing Kp...")
        ok_kp = vesc.set_kp(kp)
        print("  Writing Ki...")
        ok_ki = vesc.set_ki(ki)
        print("  Writing Kd...")
        ok_kd = vesc.set_kd(kd)
        print(f"Results: Kp={'OK' if ok_kp else 'FAIL'}, Ki={'OK' if ok_ki else 'FAIL'}, Kd={'OK' if ok_kd else 'FAIL'}")

        time.sleep(0.3)
        verify = vesc.get_mcconf()
        if verify:
            print("\nVerified values after write:")
            print(f"  Kp = {verify.get('s_pid_kp'):.6f}")
            print(f"  Ki = {verify.get('s_pid_ki'):.6f}")
            print(f"  Kd = {verify.get('s_pid_kd'):.6f}")
        else:
            print("ERROR: Could not verify written configuration.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


def run_example_7(port, duration=5, target_rpm=3000):
    vesc = None
    try:
        vesc = VescUart(port)
        print(f"Running PID benchmark: {duration}s at target RPM {target_rpm}...")
        run_benchmark(duration=duration, target_rpm=target_rpm, port=port)
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if vesc:
            vesc.close()


def main():
    print("pyVESC-uart - Interactive Example Runner\n")
    for k, v in EXAMPLES.items():
        print(f"{k}. {v}")

    choice = input("\nChoose an example to run (1-7): ").strip()
    if choice not in EXAMPLES:
        print("Invalid choice.")
        return

    if choice == '7':
        run_benchmark()
        return

    default_port = "COM5"
    port = input(f"Serial port [{default_port}]: ").strip() or default_port

    if choice == '1':
        # Keep these defaults in sync with the values defined in examples/1_run_at_rpm.py
        target_rpm = ask("Target RPM", 3000, int)
        duration = ask("Duration (s)", 2, float)
        run_example_1(port, target_rpm=target_rpm, duration=duration)
    elif choice == '2':
        # Example 2 is a fixed ramp pattern and does not expose adjustable parameters.
        run_example_2(port)
    elif choice == '3':
        # Example 3 is a read-only configuration report.
        run_example_3(port)
    elif choice == '4':
        # Keep this aligned with examples/4_read_telemetry.py
        monitor_rpm = ask("Monitor RPM (0 to not drive)", 2000, int)
        poll_interval = ask("Poll interval (s)", 0.5, float)
        run_example_4(port, monitor_rpm=monitor_rpm, poll_interval=poll_interval)
    elif choice == '5':
        # Keep these defaults aligned with examples/5_advanced_pid_tuning.py
        kp, ki, kd = read_current_pid_values(port)
        new_kp = ask("New Kp", kp, float)
        new_ki = ask("New Ki", ki, float)
        new_kd = ask("New Kd", kd, float)
        debug = ask("Enable debug (True/False)", False, lambda v: v.lower() in ('1','true','yes') if isinstance(v, str) else bool(v))
        run_example_5(port, new_kp, new_ki, new_kd, debug)
    elif choice == '6':
        # Keep these defaults aligned with the interactive PID writer and the tuning example.
        kp, ki, kd = read_current_pid_values(port)
        kp = ask("New Kp", kp, float)
        ki = ask("New Ki", ki, float)
        kd = ask("New Kd", kd, float)
        confirm = input(f"Write Kp={kp}, Ki={ki}, Kd={kd} to {port}? (yes/NO): ").strip().lower()
        if confirm in ('y', 'yes'):
            try:
                vesc = VescUart(port)
                vesc.set_debug(False)
                print("Writing Kp...")
                ok_kp = vesc.set_kp(kp)
                print("Writing Ki...")
                ok_ki = vesc.set_ki(ki)
                print("Writing Kd...")
                ok_kd = vesc.set_kd(kd)
                print(f"Results: Kp={'OK' if ok_kp else 'FAIL'}, Ki={'OK' if ok_ki else 'FAIL'}, Kd={'OK' if ok_kd else 'FAIL'}")
                reboot = input("Reboot VESC to apply changes now? (yes/NO): ").strip().lower()
                if reboot in ('y', 'yes'):
                    print("Rebooting VESC (it will disconnect/reconnect)...")
                    vesc._reboot_device(wait_seconds=1.0)

                if input("Verify by reading back values? (yes/NO): ").strip().lower() in ('y','yes'):
                    time.sleep(0.6)
                    verify = vesc.get_mcconf()
                    if verify:
                        print("Verified values:")
                        print(f"  Kp = {verify.get('s_pid_kp'):.6f}")
                        print(f"  Ki = {verify.get('s_pid_ki'):.6f}")
                        print(f"  Kd = {verify.get('s_pid_kd'):.6f}")
                    else:
                        print("Failed to read configuration for verification.")
            except Exception as e:
                print(f"Error: {e}")
            finally:
                try:
                    vesc.close()
                except Exception:
                    pass
        else:
            print("Aborted by user.")
    elif choice == '7':
        duration = ask("Benchmark duration (s)", 5, int)
        target_rpm = ask("Benchmark target RPM", 3000, int)
        run_example_7(port, duration=duration, target_rpm=target_rpm)


if __name__ == '__main__':
    main()
