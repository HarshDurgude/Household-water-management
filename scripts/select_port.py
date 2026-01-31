Import("env")
import json
import subprocess
import sys
import platform

PIOENV = env["PIOENV"]
OS = platform.system()  # 'Windows' or 'Darwin' (macOS)

custom_location = env.GetProjectOption("custom_location")

# ---- LOCATION MAP PER OS ----
TARGETS = {
    "Windows": {
        "server": "1-9.3", # middle usb 2 port
        "indicator": "1-7", #usb 3 port (left most)
    },
    "Darwin": {  # macOS
        "server": "1-1.3", # middle port on hub
        "indicator": "1-1.1", # corner usb 2 port (not usb 3)
    }
}

if custom_location:
    target_location = custom_location
    print(f"[AUTO] Using custom_location from platformio.ini = {target_location}")
else:
    if OS not in TARGETS:
        raise RuntimeError(f"Unsupported OS: {OS}")

    if PIOENV not in TARGETS[OS]:
        raise RuntimeError(f"Unknown environment: {PIOENV} and no custom_location set")

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

        # Export for monitor usage
        env['ENV']['PIO_MONITOR_PORT'] = port

        print(f"[AUTO] Upload port  = {port}")
        print(f"[AUTO] Monitor port = {port}")

        
        break
else:
    raise RuntimeError(
        f"No ESP32 found for {PIOENV} at LOCATION={target_location}"
    )
