#!/usr/bin/env python3
import os
import sys
import time
import re
import csv
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Error: The 'paho-mqtt' package is required. Install it using 'pip install paho-mqtt'.")
    sys.exit(1)

# Configurations
MQTT_BROKER = "10.0.0.9"
MQTT_PORT = 1883
TOPIC_TELEMETRY = "monsoon_outTopic"
TOPIC_LOGS = "monsoon_outlog"

LOGS_DIR = "logs"

# Session state variables
current_session_file = None
csv_writer = None
session_active = False
session_start_time = None
wash_seconds = 0
last_state = "OFF"
last_tick_time = None

# Calibration logging state
calib_file = None

# Telemetry parsed registers
telemetry_data = {
    "state": "OFF",
    "substate": "NONE",
    "temp": "0.0",
    "setpoint": "0.0",
    "flow0": "0.0",
    "flow1": "0.0",
    "flow1_est": "0.0",
    "flow1_target": "0.0",
    "pressure": "0",
    "pwm0": "0",
    "pwm1": "0",
    "ball_valve": "0",
    "tank_empty": "0",
    "tank_full": "0",
    "inlet_en": "0",
    "drain_en": "0",
    "deliver_en": "0",
    "heaterA_en": "0",
    "heater_en": "0",
    "pump_relay_en": "0",
    "burp_en": "0",
    "inlet_act": "0",
    "drain_act": "0",
    "deliver_act": "0",
    "heaterA_act": "0",
    "heater_act": "0",
    "pump_relay_act": "0",
    "burp_act": "0"
}

# Ensure log directory exists
os.makedirs(LOGS_DIR, exist_ok=True)

def is_first_shower_today():
    """Checks if any shower files exist in LOGS_DIR for today's date."""
    today = datetime.now().strftime("%Y%m%d")
    try:
        for f in os.listdir(LOGS_DIR):
            if f.startswith(today) and f.endswith("_shower.csv"):
                return False
    except Exception:
        pass
    return True

def write_to_system_log(message):
    """Appends general logs to a daily system log file."""
    today = datetime.now().strftime("%Y%m%d")
    log_filename = os.path.join(LOGS_DIR, f"{today}_system.log")
    timestamp = datetime.now().isoformat()
    try:
        with open(log_filename, mode='a') as f:
            f.write(f"[{timestamp}] {message}\n")
    except Exception as e:
        print(f"Error writing to system log: {e}")

def parse_telemetry(payload):
    """Parses incoming MQTT telemetry tokens."""
    tokens = re.findall(r'\*([^*]+)\*', payload)
    updated = False
    
    for token in tokens:
        if len(token) < 1:
            continue
        tag = token[0]
        val = token[1:]
        
        if tag == 'M': # Temperature (*M37.5*)
            telemetry_data["temp"] = val
            updated = True
        elif tag == 't': # Setpoint (*t42.0*)
            telemetry_data["setpoint"] = val
            updated = True
        elif tag == 'N': # Flow rates (*N4.2,5.8,4.3,4.5*)
            flows = val.split(',')
            if len(flows) >= 1:
                telemetry_data["flow0"] = flows[0].strip()
            if len(flows) >= 2:
                telemetry_data["flow1"] = flows[1].strip()
            if len(flows) >= 3:
                telemetry_data["flow1_est"] = flows[2].strip()
            else:
                telemetry_data["flow1_est"] = telemetry_data["flow1"]
            if len(flows) >= 4:
                telemetry_data["flow1_target"] = flows[3].strip()
            else:
                telemetry_data["flow1_target"] = "0.0"
            updated = True
        elif tag == 'F': # Delivery pump speed % (*F85*)
            telemetry_data["pwm0"] = val.strip()
            updated = True
        elif tag == 'f': # Scavenge pump speed % (*f90*)
            telemetry_data["pwm1"] = val.strip()
            updated = True
        elif tag == 'P': # Pressure in kPa (*P120.5*)
            telemetry_data["pressure"] = val.strip()
            updated = True
        elif tag == 'Q': # Level sensors: empty, full (*Q1010*)
            if len(val) >= 2:
                # val[0] is '0' when empty (dry), '1' when safe (has water)
                telemetry_data["tank_empty"] = "1" if val[0] == '0' else "0"
                telemetry_data["tank_full"] = "1" if val[1] == '1' else "0"
            updated = True
        elif tag == 'v': # Ball valve status (1=closed, 0=open)
            if len(val) >= 1 and val[0] in ('0', '1'):
                telemetry_data["ball_valve"] = val[0]
                updated = True
        elif tag == 'i': # Intent relay bits (*i0000000000*)
            if len(val) >= 8:
                telemetry_data["inlet_en"] = val[0]
                telemetry_data["drain_en"] = val[1]
                telemetry_data["deliver_en"] = val[2]
                telemetry_data["heaterA_en"] = val[3]
                telemetry_data["heater_en"] = val[4]
                telemetry_data["pump_relay_en"] = val[5]
                telemetry_data["burp_en"] = val[6]
            updated = True
        elif tag == 'k': # Actual relay physical states (*k0000000000*)
            if len(val) >= 8:
                telemetry_data["inlet_act"] = val[0]
                telemetry_data["drain_act"] = val[1]
                telemetry_data["deliver_act"] = val[2]
                telemetry_data["heaterA_act"] = val[3]
                telemetry_data["heater_act"] = val[4]
                telemetry_data["pump_relay_act"] = val[5]
                telemetry_data["burp_act"] = val[6]
            updated = True
        elif tag == 's': # State: Substate (*sWASH: AUTO*)
            if ":" in val:
                parts = val.split(":", 1)
                telemetry_data["state"] = parts[0].strip()
                telemetry_data["substate"] = parts[1].strip()
            else:
                telemetry_data["state"] = val.strip()
                telemetry_data["substate"] = "NONE"
            updated = True
            
    return updated

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        msg = f"Connected to MQTT Broker at {MQTT_BROKER}"
        print(f"[{datetime.now()}] {msg}")
        write_to_system_log(msg)
        
        client.subscribe(TOPIC_TELEMETRY)
        client.subscribe(TOPIC_LOGS)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    global current_session_file, csv_writer, session_active, last_state, session_start_time, wash_seconds, last_tick_time, calib_file
    
    topic = msg.topic
    payload = msg.payload.decode('utf-8', errors='ignore')
    
    # --- TELEMETRY STREAM PROCESSING ---
    if topic == TOPIC_TELEMETRY:
        if parse_telemetry(payload):
            state = telemetry_data["state"]
            now_time = datetime.now()
            
            # 1. Start shower session if state transitions out of OFF
            if state != "OFF" and not session_active:
                session_active = True
                session_start_time = now_time
                wash_seconds = 0
                last_tick_time = now_time
                
                first_today = "Yes" if is_first_shower_today() else "No"
                initial_temp = telemetry_data["temp"]
                
                timestamp_str = now_time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(LOGS_DIR, f"{timestamp_str}_shower.csv")
                
                print(f"[{now_time}] >>> Shower cycle started! State: {state}. Logging to {filename}")
                write_to_system_log(f"Shower session started. File: {filename}")
                
                try:
                    current_session_file = open(filename, mode='w', newline='')
                    csv_writer = csv.writer(current_session_file)
                    
                    # Write Session Metadata Header
                    csv_writer.writerow([f"# Session Start: {now_time.strftime('%Y-%m-%d %H:%M:%S')}"])
                    csv_writer.writerow([f"# First Shower Today: {first_today}"])
                    csv_writer.writerow([f"# Initial Ambient Water Temp: {initial_temp} C"])
                    csv_writer.writerow([])
                    
                    # Write CSV Headers
                    csv_writer.writerow([
                        "Elapsed_ms", "State", "Substate", 
                        "Temp", "Setpoint", 
                        "Flow0_Del", "Flow1_Rec", "Flow1_Rec_Est", "Flow1_Rec_Target",
                        "PWM0", "PWM1", 
                        "Filter_Yield", "Pressure", 
                        "Tank_Empty", "Tank_Full", 
                        "Inlet", "Drain", "Deliver", 
                        "Htr_Main", "Htr_Aux", 
                        "Ball_Valve", "Burp"
                    ])
                except Exception as e:
                    print(f"Error creating session log file: {e}")
                    current_session_file = None
                    csv_writer = None
            
            # 2. Append data point to active CSV
            if session_active and csv_writer:
                if last_tick_time:
                    delta = (now_time - last_tick_time).total_seconds()
                    if state == "WASH":
                        wash_seconds += delta
                last_tick_time = now_time
                
                elapsed_ms = int((now_time - session_start_time).total_seconds() * 1000)
                
                # Calculate recovery pump yield (LPM per % PWM duty) for direct filter health tracking
                try:
                    f1 = float(telemetry_data["flow1"])
                    pwm1 = float(telemetry_data["pwm1"])
                    filter_yield = f"{f1 / pwm1:.4f}" if pwm1 > 5.0 else "0.0000"
                except Exception:
                    filter_yield = "0.0000"

                try:
                    csv_writer.writerow([
                        elapsed_ms,
                        telemetry_data["state"],
                        telemetry_data["substate"],
                        telemetry_data["temp"],
                        telemetry_data["setpoint"],
                        telemetry_data["flow0"],
                        telemetry_data["flow1"],
                        telemetry_data.get("flow1_est", telemetry_data["flow1"]),
                        telemetry_data.get("flow1_target", "0.0"),
                        telemetry_data["pwm0"],
                        telemetry_data["pwm1"],
                        filter_yield,
                        telemetry_data["pressure"],
                        telemetry_data["tank_empty"],
                        telemetry_data["tank_full"],
                        telemetry_data["inlet_act"],
                        telemetry_data["drain_act"],
                        telemetry_data["deliver_act"],
                        telemetry_data["heater_act"],
                        telemetry_data["heaterA_act"],
                        telemetry_data["ball_valve"],
                        telemetry_data["burp_act"]
                    ])
                    current_session_file.flush()
                except Exception as e:
                    print(f"Error writing telemetry row: {e}")
            
            # 3. End shower session if state transitions back to OFF
            if state == "OFF" and session_active:
                session_active = False
                if current_session_file:
                    duration_sec = (now_time - session_start_time).total_seconds()
                    
                    csv_writer.writerow([])
                    csv_writer.writerow(["# === SESSION STATS ==="])
                    csv_writer.writerow([f"# Total Session Duration: {duration_sec/60.0:.2f} mins"])
                    csv_writer.writerow([f"# Total Active Wash Time: {wash_seconds/60.0:.2f} mins"])
                    
                    current_session_file.close()
                    current_session_file = None
                    csv_writer = None
                    print(f"[{now_time}] <<< Shower cycle complete. Log saved cleanly.")
                    write_to_system_log(f"Shower session completed. Total Duration: {duration_sec/60.0:.2f} mins")
            
            last_state = state

    # --- LOGS / CALIBRATIONS / STATUS MESSAGES PROCESSING ---
    elif topic == TOPIC_LOGS:
        print(f"[{datetime.now()}] [LOG] {payload}")
        
        # 1. Handle Calibration Data Capture
        if "CALIB_PUMP_MODEL" in payload or "CALIB_TEMP_LOG_START" in payload:
            if not calib_file:
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                calib_filename = os.path.join(LOGS_DIR, f"{timestamp_str}_calibration.log")
                print(f"[{datetime.now()}] >>> Calibration detected! Logging to {calib_filename}")
                write_to_system_log(f"Calibration run started. File: {calib_filename}")
                try:
                    calib_file = open(calib_filename, mode='w')
                except Exception as e:
                    print(f"Error opening calibration file: {e}")
                    calib_file = None

        if calib_file:
            try:
                calib_file.write(f"[{datetime.now().isoformat()}] {payload}\n")
                calib_file.flush()
            except Exception as e:
                print(f"Error writing calibration payload: {e}")

        if "CALIB_TEMP_LOG_END" in payload or "CALIB_TEMP_LOG_EMPTY" in payload:
            if calib_file:
                calib_file.close()
                calib_file = None
                print(f"[{datetime.now()}] <<< Calibration run complete. Log closed.")
                write_to_system_log("Calibration run completed.")
        
        # 2. Duplicate all logs to the daily system log file for chronological system records
        write_to_system_log(payload)

def main():
    print(f"Starting Unified Unix Datalogger...")
    print(f"Targeting MQTT Broker: {MQTT_BROKER}")
    print(f"Logging directory: ./{LOGS_DIR}")
    
    try:
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        client = mqtt.Client()
        
    client.on_connect = on_connect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    
    while True:
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
            break
        except Exception as e:
            print(f"Could not connect to broker ({e}). Retrying in 5 seconds...")
            time.sleep(5)
            
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nShutting down logger.")
        if current_session_file:
            current_session_file.close()
        if calib_file:
            calib_file.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
