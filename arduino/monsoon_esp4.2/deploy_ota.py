#!/usr/bin/env python3
import os
import re
import sys
import subprocess
import json

OTA_H_PATH = "ota.h"
BINARY_PATH = ".pio/build/esp32_monsoon/firmware.bin"
PI_USER_IP = "nicolls@10.0.0.7"
PI_JSON_PATH = "/var/www/html/ota/monsoon.json"
PI_BIN_DIR = "/var/www/html/ota"

def bump_version():
    if not os.path.exists(OTA_H_PATH):
        print(f"Error: {OTA_H_PATH} not found.")
        sys.exit(1)
        
    with open(OTA_H_PATH, "r") as f:
        content = f.read()
        
    # Find: #define OTA_VERSION    "X.Y.Z"
    match = re.search(r'#define\s+OTA_VERSION\s+"([^"]+)"', content)
    if not match:
        print("Error: Could not find OTA_VERSION define in ota.h.")
        sys.exit(1)
        
    old_version = match.group(1)
    parts = old_version.split(".")
    if len(parts) != 3:
        print(f"Error: Invalid version format '{old_version}'. Expected X.Y.Z")
        sys.exit(1)
        
    parts[2] = str(int(parts[2]) + 1)
    new_version = ".".join(parts)
    
    new_content = re.sub(
        r'(#define\s+OTA_VERSION\s+")([^"]+)(")',
        r'\g<1>' + new_version + r'\g<3>',
        content
    )
    
    with open(OTA_H_PATH, "w") as f:
        f.write(new_content)
        
    print(f"Successfully bumped version: {old_version} -> {new_version}")
    return new_version

def build_project():
    print("Compiling code with PlatformIO...")
    result = subprocess.run(["pio", "run"], capture_output=True, text=True)
    if result.returncode != 0:
        print("Build failed!")
        print(result.stdout)
        print(result.stderr)
        sys.exit(1)
    print("Compilation successful.")

def deploy_to_pi(new_version):
    # Upload binary
    print(f"Uploading firmware binary to Pi ({PI_USER_IP})...")
    scp_result = subprocess.run([
        "scp",
        BINARY_PATH,
        f"{PI_USER_IP}:{PI_BIN_DIR}/firmware.bin"
    ])
    if scp_result.returncode != 0:
        print("Error: scp upload failed.")
        sys.exit(1)
        
    # Upload HTML GUI
    print(f"Uploading monsoon.html to Pi...")
    scp_html_result = subprocess.run([
        "scp",
        "monsoon.html",
        f"{PI_USER_IP}:/var/www/html/monsoon.html"
    ])
    if scp_html_result.returncode != 0:
        print("Warning: scp upload of monsoon.html failed.")

    # Update JSON file on the Pi
    print("Updating monsoon.json configuration on the Pi...")
    # Python script runner on Pi to safely load and update the JSON version
    py_cmd = (
        f"python3 -c \""
        f"import json; "
        f"f=open('{PI_JSON_PATH}', 'r+'); "
        f"d=json.load(f); "
        f"d['Configurations'][0]['Version']='{new_version}'; "
        f"f.seek(0); "
        f"json.dump(d, f, indent=2); "
        f"f.truncate()\""
    )
    ssh_result = subprocess.run(["ssh", PI_USER_IP, py_cmd])
    if ssh_result.returncode != 0:
        print("Error: Failed to update monsoon.json on the Pi.")
        sys.exit(1)

def main():
    new_ver = bump_version()
    build_project()
    deploy_to_pi(new_ver)
    print("\n" + "="*50)
    print(f"SUCCESS: Version {new_ver} deployed to local Pi OTA server.")
    print("Trigger 'Reboot' from your dashboard to apply the update!")
    print("="*50)

if __name__ == "__main__":
    main()
