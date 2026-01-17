import requests
import socket
import argparse

parser = argparse.ArgumentParser()


# Replace with your webhook URL
WEBHOOK_URL = "https://discord.com/api/webhooks/1435393485262946445/kpzKJ9iZ4o8wSZdboCGKRg_2LlTLTAHPFNhh_sUDncrlvFW1xgsy4BsUjbuznVZzhlUu"

parser.add_argument("jetson_no")
args = parser.parse_args()
JETSON_NO = args.jetson_no

# Get local IP (Jetson's eduroam IP)
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't need to connect, just to get the IP used to reach outside
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except:
        ip = "Unable to get IP"
    finally:
        s.close()
    return ip

ip_address = get_ip()

# Post to Discord
data = {"content": f"Jetson {JETSON_NO} IP: {ip_address}"}
requests.post(WEBHOOK_URL, json=data)