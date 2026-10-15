"""
PID response benchmark for VESC.

This script is a full test bench that:
    1. Prompts for the serial port and test values.
    2. Runs a baseline RPM step test.
    3. Writes a new Kp/Ki/Kd set.
    4. Reboots the VESC so the config is applied.
    5. Runs the same RPM step again.
    6. Saves all data to a CSV log.
    7. Optionally plots the RPM curve before vs. after.

Usage:
    python benchmark_pid_response.py
"""

import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from statistics import mean

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from vesc_uart import VescUart

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:
    plt = None

DEFAULT_PORT = "COM5"
DEFAULT_TARGET_RPM = 2000
DEFAULT_STEP_SECONDS = 5.0
DEFAULT_SAMPLE_INTERVAL = 0.05


def prompt_value(label, default, cast, min_value=None, max_value=None):
    while True:
        raw = input(f"{label} [{default}]: ").strip()
        if raw == "":
            value = default
        else:
            try:
                value = cast(raw)
            except Exception:
                print("Invalid value. Try again.")
                continue

        if min_value is not None and value < min_value:
            print(f"Value must be >= {min_value}.")
            continue
        if max_value is not None and value > max_value:
            print(f"Value must be <= {max_value}.")
            continue
        return value


def read_pid(vesc):
    config = vesc.get_mcconf()
    if not config:
        raise RuntimeError("Could not read mcconf from VESC. Check UART connection and app mode.")
    return {
        "kp": float(config.get("s_pid_kp", 0.005)),
        "ki": float(config.get("s_pid_ki", 0.003)),
        "kd": float(config.get("s_pid_kd", 0.00005)),
    }


def run_rpm_step(vesc, target_rpm, duration_seconds, sample_interval):
    """Run a fixed RPM step and return sampled points."""
    samples = []
    start_time = time.monotonic()

    print(f"Running RPM step: target={target_rpm} ERPM for {duration_seconds:.1f}s")
    vesc.set_rpm(0)
    time.sleep(0.3)

    vesc.set_rpm(target_rpm)
    while time.monotonic() - start_time < duration_seconds:
        if vesc.get_values():
            d = vesc.data
            samples.append({
                "time": time.monotonic() - start_time,
                "target": float(target_rpm),
                "rpm": float(d.rpm),
                "motor_current": float(d.avg_motor_current),
                "duty": float(d.duty_cycle_now),
                "voltage": float(d.input_voltage),
            })
        time.sleep(sample_interval)

    vesc.set_rpm(0)
    time.sleep(0.3)
    return samples


def summarize_response(samples, target_rpm):
    if not samples:
        return {
            "rise_time": None,
            "overshoot_percent": None,
            "settle_time": None,
            "final_error": None,
            "avg_current": None,
            "avg_duty": None,
            "peak_rpm": None,
        }

    actual_rpms = [s["rpm"] for s in samples]
    peak_rpm = max(actual_rpms)
    overshoot = ((peak_rpm - target_rpm) / target_rpm) * 100.0 if target_rpm else 0.0

    rise_time = None
    for s in samples:
        if s["rpm"] >= 0.9 * target_rpm:
            rise_time = s["time"]
            break

    settle_time = None
    for s in samples:
        if abs(s["rpm"] - target_rpm) <= 0.05 * abs(target_rpm):
            settle_time = s["time"]
        else:
            settle_time = None

    final_value = samples[-1]["rpm"]
    return {
        "rise_time": rise_time,
        "overshoot_percent": overshoot,
        "settle_time": settle_time,
        "final_error": final_value - target_rpm,
        "avg_current": mean(s["motor_current"] for s in samples),
        "avg_duty": mean(s["duty"] for s in samples),
        "peak_rpm": peak_rpm,
    }


def print_summary(label, response, target_rpm):
    print(f"\n=== {label} ===")
    print(f"Target RPM: {target_rpm}")
    print(f"Peak RPM: {response['peak_rpm'] if response['peak_rpm'] is not None else 'N/A'}")
    print(f"Rise time to 90%: {response['rise_time'] if response['rise_time'] is not None else 'N/A'} s")
    print(f"Overshoot: {response['overshoot_percent']:.2f}%")
    print(f"Settling time within 5%: {response['settle_time'] if response['settle_time'] is not None else 'N/A'} s")
    print(f"Final error: {response['final_error']:.0f} ERPM")
    print(f"Average motor current: {response['avg_current']:.2f} A")
    print(f"Average duty: {response['avg_duty'] * 100.0:.1f}%")


def make_log_rows(run_name, samples, kp, ki, kd):
    rows = []
    for s in samples:
        rows.append({
            "run": run_name,
            "time_s": s["time"],
            "target_rpm": s["target"],
            "rpm": s["rpm"],
            "motor_current_a": s["motor_current"],
            "duty_cycle": s["duty"],
            "voltage_v": s["voltage"],
            "kp": kp,
            "ki": ki,
            "kd": kd,
        })
    return rows


def save_csv_log(rows, output_path):
    fieldnames = [
        "run",
        "target_rpm",
        "rise_time_s",
        "overshoot_percent",
        "settle_time_s",
        "peak_rpm",
        "final_error_erpm",
        "avg_current_a",
        "avg_duty_percent",
        "kp",
        "ki",
        "kd",
    ]

    with output_path.open("w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"CSV log saved to: {output_path}")


def save_sample_csv(rows, output_path):
    fieldnames = [
        "time_s",
        "rpm_before",
        "motor_current_a_before",
        "duty_cycle_before",
        "voltage_v_before",
        "rpm_after",
        "motor_current_a_after",
        "duty_cycle_after",
        "voltage_v_after",
        "kp_before",
        "ki_before",
        "kd_before",
        "kp_after",
        "ki_after",
        "kd_after",
    ]

    with output_path.open("w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Sample CSV saved to: {output_path}")


def plot_response(before_samples, after_samples, output_path):
    if plt is None:
        print("Matplotlib is not installed, so no graph was generated.")
        return False

    fig, ax = plt.subplots(figsize=(10, 6))

    if before_samples:
        ax.plot([s["time"] for s in before_samples], [s["rpm"] for s in before_samples], label="Before", alpha=0.9)
    if after_samples:
        ax.plot([s["time"] for s in after_samples], [s["rpm"] for s in after_samples], label="After", alpha=0.9)

    ax.set_title("VESC PID RPM Response Benchmark")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RPM")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    print(f"Graph saved to: {output_path}")
    return True


def build_summary_row(run_name, samples, target_rpm, kp, ki, kd):
    summary = summarize_response(samples, target_rpm)
    return {
        "run": run_name,
        "target_rpm": target_rpm,
        "rise_time_s": summary["rise_time"],
        "overshoot_percent": summary["overshoot_percent"],
        "settle_time_s": summary["settle_time"],
        "peak_rpm": summary["peak_rpm"],
        "final_error_erpm": summary["final_error"],
        "avg_current_a": summary["avg_current"],
        "avg_duty_percent": summary["avg_duty"] * 100.0,
        "kp": kp,
        "ki": ki,
        "kd": kd,
    }


def build_sample_rows(baseline_samples, after_samples, before_pid, after_pid):
    max_len = max(len(baseline_samples), len(after_samples))
    rows = []
    for i in range(max_len):
        before = baseline_samples[i] if i < len(baseline_samples) else {}
        after = after_samples[i] if i < len(after_samples) else {}
        rows.append({
            "time_s": before.get("time", after.get("time", i * 0.05)),
            "rpm_before": before.get("rpm", ""),
            "motor_current_a_before": before.get("motor_current", ""),
            "duty_cycle_before": before.get("duty", ""),
            "voltage_v_before": before.get("voltage", ""),
            "rpm_after": after.get("rpm", ""),
            "motor_current_a_after": after.get("motor_current", ""),
            "duty_cycle_after": after.get("duty", ""),
            "voltage_v_after": after.get("voltage", ""),
            "kp_before": before_pid["kp"],
            "ki_before": before_pid["ki"],
            "kd_before": before_pid["kd"],
            "kp_after": after_pid["kp"],
            "ki_after": after_pid["ki"],
            "kd_after": after_pid["kd"],
        })
    return rows


def main():
    print("pyVESC-uart - PID Response Benchmark")
    print("This benchmark tests whether new PID values change real motor behavior.\n")

    port = prompt_value("Serial port", DEFAULT_PORT, str)
    target_rpm = prompt_value("Target RPM for test", DEFAULT_TARGET_RPM, int, min_value=0)
    step_seconds = prompt_value("Step duration (seconds)", DEFAULT_STEP_SECONDS, float, min_value=0.5)
    sample_interval = prompt_value("Sample interval (seconds)", DEFAULT_SAMPLE_INTERVAL, float, min_value=0.01)

    vesc = VescUart(port)
    vesc.set_debug(False)

    try:
        current = read_pid(vesc)
        print("\nCurrent VESC PID values:")
        print(f"  Kp = {current['kp']:.6f}")
        print(f"  Ki = {current['ki']:.6f}")
        print(f"  Kd = {current['kd']:.6f}\n")

        new_kp = prompt_value("New Kp", current['kp'], float, min_value=0.0)
        new_ki = prompt_value("New Ki", current['ki'], float, min_value=0.0)
        new_kd = prompt_value("New Kd", current['kd'], float, min_value=0.0)

        print("\nRunning baseline test...")
        baseline_samples = run_rpm_step(vesc, target_rpm, step_seconds, sample_interval)
        baseline_summary = summarize_response(baseline_samples, target_rpm)
        print_summary("Baseline", baseline_summary, target_rpm)

        print("\nWriting new PID values to VESC...")
        ok_kp = vesc.set_kp(new_kp)
        ok_ki = vesc.set_ki(new_ki)
        ok_kd = vesc.set_kd(new_kd)
        print(f"  Kp write: {'OK' if ok_kp else 'FAIL'}")
        print(f"  Ki write: {'OK' if ok_ki else 'FAIL'}")
        print(f"  Kd write: {'OK' if ok_kd else 'FAIL'}")

        print("Rebooting VESC to apply the config...")
        vesc._reboot_device(wait_seconds=1.0)

        time.sleep(1.0)
        vesc.close()
        time.sleep(0.5)
        vesc = VescUart(port)
        vesc.set_debug(False)

        after_config = read_pid(vesc)
        print("\nValues after reboot:")
        print(f"  Kp = {after_config['kp']:.6f}")
        print(f"  Ki = {after_config['ki']:.6f}")
        print(f"  Kd = {after_config['kd']:.6f}\n")

        print("Running post-update benchmark...")
        after_samples = run_rpm_step(vesc, target_rpm, step_seconds, sample_interval)
        after_summary = summarize_response(after_samples, target_rpm)
        print_summary("After Update", after_summary, target_rpm)

        print("\n=== Comparison ===")
        if baseline_summary["rise_time"] is not None and after_summary["rise_time"] is not None:
            print(f"Rise time changed: {baseline_summary['rise_time']:.2f}s -> {after_summary['rise_time']:.2f}s")
        else:
            print("Rise time could not be compared reliably.")
        print(f"Overshoot changed: {baseline_summary['overshoot_percent']:.2f}% -> {after_summary['overshoot_percent']:.2f}%")
        print(f"Final error changed: {baseline_summary['final_error']:.0f} -> {after_summary['final_error']:.0f} ERPM")

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = Path(__file__).resolve().parent
        summary_path = log_dir / f"pid_benchmark_summary_{timestamp}.csv"
        sample_path = log_dir / f"pid_benchmark_samples_{timestamp}.csv"
        graph_path = log_dir / f"pid_benchmark_{timestamp}.png"

        summary_rows = [
            build_summary_row("baseline", baseline_samples, target_rpm, current["kp"], current["ki"], current["kd"]),
            build_summary_row("after_update", after_samples, target_rpm, new_kp, new_ki, new_kd),
        ]
        sample_rows = build_sample_rows(
            baseline_samples,
            after_samples,
            current,
            {"kp": new_kp, "ki": new_ki, "kd": new_kd},
        )

        save_csv_log(summary_rows, summary_path)
        save_sample_csv(sample_rows, sample_path)
        plot_response(baseline_samples, after_samples, graph_path)

        print("\nBenchmark complete.")

    except Exception as exc:
        print(f"Benchmark failed: {exc}")
    finally:
        try:
            vesc.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
