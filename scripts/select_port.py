Import("env")
import json
import subprocess
import sys
import platform

PIOENV = env["PIOENV"]
OS = platform.system()  # 'Windows' or 'Darwin' (macOS)

# ---- LOCATION MAP PER OS ----
TARGETS = {
    "Windows": {
        "server": "1-9.3",
        "indicator": "1-7",
    },
    "Darwin": {  # macOS
        "server": "1-1.3",
        "indicator": "1-1.1",
    }
}

if OS not in TARGETS:
    raise RuntimeError(f"Unsupported OS: {OS}")

if PIOENV not in TARGETS[OS]:
    raise RuntimeError(f"Unknown environment: {PIOENV}")

target_location = TARGETS[OS][PIOENV]

print(f"[AUTO] OS = {OS}")
print(f"[AUTO] Target {PIOENV} at USB LOCATION = {target_location}")

# Ask PlatformIO for connected devices
result = subprocess.run(
    ["pio", "device", "list", "--json-output"],
    capture_output=True,
    text=True
)

devices = json.loads(result.stdout)

for dev in devices:
    hwid = dev.get("hwid", "")
    port = dev.get("port", "")

    if f"LOCATION={target_location}" in hwid:
        print(f"[AUTO] Selected port: {port}")
        env.Replace(UPLOAD_PORT=port)
        env.Replace(MONITOR_PORT=port)
        break
else:
    raise RuntimeError(
        f"No ESP32 found for {PIOENV} at LOCATION={target_location}"
    )
