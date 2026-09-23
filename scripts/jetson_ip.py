import argparse
import os
import socket
import time

import requests


# Supplied by Codespaces secrets or the service environment.
WEBHOOK_URL = os.environ.get("DISCORD_WEBHOOK_URL", "").strip()


def get_ip():
    """Return the local IPv4 address selected for the outbound route."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # UDP route lookup; this does not verify internet connectivity.
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return None


def monitor(jetson_no, interval):
    """Report new IPs and retry notifications until they succeed."""
    last_reported_ip = None
    while True:
        ip_address = get_ip()
        if ip_address is None:
            # Report reconnection even when the address stays the same.
            last_reported_ip = None
        elif ip_address != last_reported_ip:
            data = {"content": f"Jetson {jetson_no} IP: {ip_address}"}
            try:
                response = requests.post(WEBHOOK_URL, json=data, timeout=10)
                response.raise_for_status()
            except requests.RequestException:
                # Avoid logging the exception, which may contain the webhook URL.
                print("Webhook request failed; will retry.", flush=True)
            else:
                last_reported_ip = ip_address
                print(f"Reported Jetson {jetson_no} IP: {ip_address}", flush=True)
        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description="Monitor and report the Jetson IP.")
    parser.add_argument("jetson_no")
    parser.add_argument("--interval", type=int, default=30,
                        help="Seconds between IP checks (default: 30)")
    args = parser.parse_args()
    if not WEBHOOK_URL:
        parser.error("Set the DISCORD_WEBHOOK_URL environment variable")
    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    try:
        monitor(args.jetson_no, args.interval)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
