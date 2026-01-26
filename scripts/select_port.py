Import("env")
import json
import subprocess
import sys

target_location = env.GetProjectOption("custom_location")

print(f"[AUTO] Target LOCATION = {target_location}")

try:
    result = subprocess.check_output(
        ["pio", "device", "list", "--json-output"],
        text=True
    )
except Exception as e:
    raise RuntimeError("Failed to run pio device list") from e

devices = json.loads(result)

selected_port = None

for dev in devices:
    hwid = dev.get("hwid", "")
    port = dev.get("port", "")
    if f"LOCATION={target_location}" in hwid:
        selected_port = port
        print(f"[AUTO] Matched port {port} for LOCATION {target_location}")
        break

if not selected_port:
    print("[AUTO] Available devices:")
    for dev in devices:
        print(" ", dev)
    raise RuntimeError(f"No ESP32 found at LOCATION={target_location}")

# Apply port to upload & monitor
env.Replace(UPLOAD_PORT=selected_port)
env.Replace(MONITOR_PORT=selected_port)
