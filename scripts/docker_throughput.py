#!/usr/bin/env python3
"""Usage: python3 docker_throughput.py <container_name_or_id> [interval_seconds]"""

import json
import os
import subprocess
import sys
import time

def format_rate(bytes_per_sec: float) -> str:
    """Format bytes into readable rates using standard binary units."""
    units = ["B/s", "KiB/s", "MiB/s", "GiB/s", "TiB/s"]
    val = float(bytes_per_sec)
    for unit in units:
        if val < 1024.0 or unit == units[-1]:
            return f"{val:6.2f} {unit}"
        val /= 1024.0
    return f"{bytes_per_sec:.2f} B/s"

def format_rate_bits(bytes_per_sec: float) -> str:
    bits_per_sec = bytes_per_sec * 8.0
    units = ["bps", "Kbps", "Mbps", "Gbps"]
    val = bits_per_sec
    for unit in units:
        if val < 1000.0 or unit == units[-1]:
            return f"{val:6.2f} {unit}"
        val /= 1000.0
    return f"{bits_per_sec:.2f} bps"

def get_container_pid(container: str) -> int:
    """Retrieve the host PID for the primary process of the container."""
    try:
        out = subprocess.check_output(
            ["docker", "inspect", "--format", "{{.State.Pid}}", container],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
        pid = int(out)
        if pid <= 0:
            raise ValueError(f"Container '{container}' is not running.")
        return pid
    except (subprocess.CalledProcessError, ValueError):
        return 0

def read_proc_net_dev(pid: int) -> tuple[int, int]:
    """Read cumulative rx_bytes and tx_bytes from /proc/<pid>/net/dev."""
    net_file = f"/proc/{pid}/net/dev"
    if not os.path.exists(net_file):
        raise PermissionError(f"Cannot access {net_file}. Ensure you have root/sudo privileges.")

    rx_total = 0
    tx_total = 0
    with open(net_file, "r", encoding="utf-8") as f:
        lines = f.readlines()[2:]  # Skip headers
        for line in lines:
            parts = line.strip().split()
            if not parts:
                continue
            iface = parts[0].rstrip(":")
            if iface == "lo":  # Exclude loopback traffic
                continue
            rx_total += int(parts[1])
            tx_total += int(parts[9])
    return rx_total, tx_total

def get_docker_stats_fallback(container: str) -> tuple[int, int]:
    """Parse raw bytes via docker stats JSON stream if /proc is inaccessible."""
    res = subprocess.run(
        ["docker", "stats", container, "--no-stream", "--format", "{{json .}}"],
        capture_output=True,
        text=True,
        check=True
    )
    data = json.loads(res.stdout)
    net_str = data.get("NetIO", "0B / 0B")
    
    def parse_suffix(s: str) -> float:
        s = s.strip().upper()
        multipliers = {
            "B": 1,
            "KB": 10**3, "KIB": 2**10,
            "MB": 10**6, "MIB": 2**20,
            "GB": 10**9, "GIB": 2**30,
            "TB": 10**12, "TIB": 2**40
        }
        for suffix, mult in sorted(multipliers.items(), key=lambda x: -len(x[0])):
            if s.endswith(suffix):
                val_str = s[:-len(suffix)].strip()
                return float(val_str) * mult
        return 0.0

    parts = net_str.split("/")
    if len(parts) == 2:
        return int(parse_suffix(parts[0])), int(parse_suffix(parts[1]))
    return 0, 0

def sample_bytes(container: str, pid: int) -> tuple[int, int]:
    if pid > 0 and os.path.exists(f"/proc/{pid}/net/dev"):
        return read_proc_net_dev(pid)
    return get_docker_stats_fallback(container)

def monitor_throughput(container: str, interval: float = 1.0):
    pid = get_container_pid(container)
    mode = "Direct /proc (High Precision)" if pid > 0 and os.path.exists(f"/proc/{pid}/net/dev") else "Docker Stats API (Fallback)"

    print(f"Tracking network traffic for '{container}'")
    print(f"Mode: {mode}")
    print("-" * 50)
    print(f"{'TIMESTAMP':<10} | {'RX RATE (Down)':<16} | {'TX RATE (Up)':<16}")
    print("-" * 50)

    try:
        rx_prev, tx_prev = sample_bytes(container, pid)
        t_prev = time.perf_counter()

        while True:
            time.sleep(interval)
            t_curr = time.perf_counter()
            rx_curr, tx_curr = sample_bytes(container, pid)

            delta_t = t_curr - t_prev
            if delta_t <= 0:
                continue

            rx_speed = max(0.0, (rx_curr - rx_prev) / delta_t)
            tx_speed = max(0.0, (tx_curr - tx_prev) / delta_t)

            ts_str = time.strftime("%H:%M:%S")
            print(f"{ts_str:<10} | {format_rate_bits(rx_speed):<16} | {format_rate_bits(tx_speed):<16}")

            rx_prev, tx_prev, t_prev = rx_curr, tx_curr, t_curr

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
    except Exception as exc:
        sys.exit(f"\nError: {exc}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Usage: python3 docker_throughput.py <container_name_or_id> [interval_seconds]")

    target_container = sys.argv[1]
    poll_interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    monitor_throughput(target_container, poll_interval)
