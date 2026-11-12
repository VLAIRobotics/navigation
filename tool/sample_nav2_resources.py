#!/usr/bin/env python3
"""Write one-second process and system resource samples as JSON Lines."""

import argparse
import datetime as dt
import glob
import json
import os
import signal
import sys
import time
from pathlib import Path

_running = True


def _stop(_signum, _frame):
    global _running
    _running = False


def _read_text(path):
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").strip()
    except (OSError, ValueError):
        return ""


def _read_process(pid):
    proc = Path("/proc") / str(pid)
    stat = _read_text(proc / "stat")
    close = stat.rfind(")")
    if close < 0:
        return None
    fields = stat[close + 2 :].split()
    if len(fields) <= 12:
        return None
    try:
        ticks = int(fields[11]) + int(fields[12])
    except ValueError:
        return None

    status = {}
    for line in _read_text(proc / "status").splitlines():
        key, separator, value = line.partition(":")
        if separator:
            status[key] = value.strip()
    uid_value = status.get("Uid", "").split()
    try:
        process_uid = int(uid_value[0])
    except (IndexError, ValueError):
        return None
    if process_uid != os.getuid():
        return None
    rss_value = status.get("VmRSS", "0 kB").split()
    threads_value = status.get("Threads", "0").split()
    try:
        rss_kb = int(rss_value[0]) if rss_value else 0
        threads = int(threads_value[0]) if threads_value else 0
    except ValueError:
        rss_kb, threads = 0, 0

    cmdline = _read_text(proc / "cmdline").replace("\x00", " ").strip()
    if not cmdline:
        return None
    comm = stat[stat.find("(") + 1 : close]
    return {
        "pid": pid,
        "name": comm,
        "cmdline": cmdline[:512],
        "cpu_ticks": ticks,
        "rss_kb": rss_kb,
        "threads": threads,
    }


def _read_cpu_totals():
    fields = _read_text("/proc/stat").splitlines()
    if not fields or not fields[0].startswith("cpu "):
        return None
    try:
        values = [int(value) for value in fields[0].split()[1:]]
    except ValueError:
        return None
    if len(values) < 5:
        return None
    # guest and guest_nice are already included in user and nice.
    return sum(values[:8]), values[3] + values[4]


def _read_memory():
    values = {}
    for line in _read_text("/proc/meminfo").splitlines():
        key, separator, value = line.partition(":")
        if separator:
            parts = value.split()
            if parts:
                try:
                    values[key] = int(parts[0])
                except ValueError:
                    pass
    swap_used = max(0, values.get("SwapTotal", 0) - values.get("SwapFree", 0))
    return {
        "total_kb": values.get("MemTotal"),
        "available_kb": values.get("MemAvailable"),
        "swap_used_kb": swap_used,
    }


def _read_temperatures():
    temperatures = {}
    for path in sorted(glob.glob("/sys/class/thermal/thermal_zone*/temp")):
        parent = Path(path).parent
        raw = _read_text(path)
        try:
            value = float(raw)
        except ValueError:
            continue
        if abs(value) > 1000.0:
            value /= 1000.0
        if -20.0 <= value <= 200.0:
            label = _read_text(parent / "type") or parent.name
            temperatures[label] = round(value, 2)
    return temperatures


def _read_frequency(path, divisor):
    raw = _read_text(path)
    try:
        value = float(raw)
    except ValueError:
        return None
    return value / divisor if value > 0.0 else None


def _read_frequencies():
    cpu_values = [
        value
        for path in glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq")
        if (value := _read_frequency(path, 1000.0)) is not None
    ]
    device_values = {}
    for path in glob.glob("/sys/class/devfreq/*/cur_freq"):
        divisor = 1_000_000.0 if (raw := _read_text(path)).isdigit() and int(raw) > 10_000_000 else 1000.0
        value = _read_frequency(path, divisor)
        if value is not None:
            device_values[Path(path).parent.name] = round(value, 2)
    return {
        "cpu_mhz_avg": round(sum(cpu_values) / len(cpu_values), 2) if cpu_values else None,
        "device_mhz": device_values,
    }


def _system_sample(previous_cpu):
    cpu = _read_cpu_totals()
    cpu_percent = None
    if cpu is not None and previous_cpu is not None:
        total_delta = cpu[0] - previous_cpu[0]
        idle_delta = cpu[1] - previous_cpu[1]
        if total_delta > 0:
            cpu_percent = round(max(0.0, min(100.0, 100.0 * (total_delta - idle_delta) / total_delta)), 2)
    return {
        "cpu_percent_total": cpu_percent,
        "cpu_count": os.cpu_count(),
        "memory": _read_memory(),
        "temperatures_c": _read_temperatures(),
        "frequencies": _read_frequencies(),
    }, cpu


def sample(output_path, interval):
    ticks_per_second = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    cpu_count = max(1, os.cpu_count() or 1)
    previous_processes = {}
    previous_cpu = None
    previous_time = time.monotonic()
    next_sample = previous_time
    output = open(output_path, "a", encoding="utf-8", buffering=1)
    try:
        while _running:
            now = time.monotonic()
            elapsed = max(1e-6, now - previous_time)
            timestamp = dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")
            system, cpu = _system_sample(previous_cpu)
            processes = []
            current_processes = {}
            try:
                pids = (int(entry.name) for entry in Path("/proc").iterdir() if entry.name.isdigit())
                for pid in pids:
                    process = _read_process(pid)
                    if process is None:
                        continue
                    signature = (process["name"], process["cmdline"])
                    old = previous_processes.get(pid)
                    if old is not None and old[0] == signature:
                        cpu_delta = max(0, process["cpu_ticks"] - old[1])
                        cpu_percent_one_core = (
                            100.0 * cpu_delta / ticks_per_second / elapsed
                        )
                    else:
                        cpu_percent_one_core = 0.0
                    process["cpu_percent_one_core"] = round(cpu_percent_one_core, 2)
                    process["cpu_percent_total"] = round(
                        cpu_percent_one_core / cpu_count, 2)
                    current_processes[pid] = (signature, process["cpu_ticks"])
                    process.pop("cpu_ticks")
                    processes.append(process)
            except OSError:
                pass
            processes.sort(key=lambda item: item["pid"])
            output.write(json.dumps({
                "timestamp_utc": timestamp,
                "system": system,
                "processes": processes,
            }, separators=(",", ":")) + "\n")
            output.flush()
            previous_processes = current_processes
            previous_cpu = cpu
            previous_time = now
            next_sample += interval
            time.sleep(max(0.0, next_sample - time.monotonic()))
    finally:
        output.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="JSONL output path")
    parser.add_argument("--interval", type=float, default=1.0, help="sample period in seconds")
    args = parser.parse_args()
    if not (args.interval > 0.0):
        parser.error("--interval must be positive")
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        sample(args.output, args.interval)
    except OSError as error:
        print(f"resource sampler failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
