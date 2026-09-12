# Project Rules: Monsoon (ESP32 Recirculating Shower)

Goal: Transitioning from Arduino IDE to PlatformIO while maintaining interchangeable compatibility.

## 1. Directory & Scoping Rules

* **Active Code:** Located strictly in the project root directory (`.`).
* **Excluded Folders:** Ignore `attic/`, `test/`, and `temp/` for all coding suggestions. These contain legacy or experimental code that conflicts with the current architecture.
* **Environment:** PlatformIO using the ESP32 v3.0 core.

## 2. Core Architecture: State vs. Intent

The project uses a decoupled "Gatekeeper" model to manage high-power hardware safely:

* **The Intent Layer (`_en`):** Control logic (state machines/thermostats) sets the *ideal* state using `setrelay_en(pin, state)`. It does not check safety; it only expresses what the system *wants* to do.
* **The Physical Layer (`loop_heaters`):** The hardware gatekeeper. It evaluates the `safety_veto` and synchronizes the physical pins to the intent only if safe.
* **Safety Veto Logic:**
  * `htrs_forcedisable`: GUI/User override (Manual Veto).
  * `level_safe`: Physical float switch check (`!tank_empty`).
  * `temp_safe`: Thermal cutoff (`temp1 <= htrs_maxtemp`, 62.5°C).
* **Heater Cutout & Debounced Auto-Recovery:**
  * Immediate cut on low level or high temp.
  * Re-arming requires `tank_empty == false` continuously for **3000 ms** plus the 3000ms anti-chatter lockout before physical relays re-engage.
* **The Rule:** If `safety_veto` is **True**, physical pins MUST be `ROFF` regardless of the `_en` intent.

## 3. Hardware Constraints (ESP32 DevKit V1)

* **Input-Only Pins:** GPIO 34, 35, 36, and 39 are strictly input-only (no internal pull-ups, no output capability).
* **Relay Logic:** Mixed Active High/Active Low logic is handled via the `RPINS_ROFF` array within the `setrelay()` wrapper.
* **Sensor Mapping:**
  * **Delivery Flow (Delivery Pump):** 
    * `flow_lpm0`, `flow_cnt0` $\rightarrow$ `FSPINS[0]` = **GPIO 25** (Moved from 36, requires external $4.7\text{k}\Omega$ pull-up).
  * **Recovery Flow (Scavenge Pump):** 
    * `flow_lpm1`, `flow_cnt1` $\rightarrow$ `FSPINS[1]` = **GPIO 26** (Moved from 39, requires external $4.7\text{k}\Omega$ pull-up).
  * **Temperature (Control NTC Thermistor):** 
    * `temp1` $\rightarrow$ `TSA2PIN` = **GPIO 39 / VN** (Requires external $15\text{k}\Omega$ series divider to 3.3V, NTC to GND).
  * **Temperature (Display DS18B20 OneWire):** 
    * *Freed/Retired* (frees GPIO 4 for Main Heater Relay 3).
  * **Float Switches (Low & High):** 
    * `LSPINS[0]` (Low) = **GPIO 36 / VP**, `LSPINS[1]` (High) = **GPIO 34**. (Input-only, require external $4.7\text{k}\Omega$ pull-ups).
  * **Pressure Sensor:** 
    * `PSPIN` = **GPIO 35** (Requires external $10\text{k}\Omega / 20\text{k}\Omega$ attenuation divider).
* **Switching Guard:** A 3000ms `htrs_blocked` lockout is required to prevent relay bouncing/chatter.
* **Safety Relay**: The pump power supply must run through a series mechanical relay (`RPINS[5]` / `RPINS[2]`) as a physical gatekeeper in series with the speed controllers to safeguard against MOSFET short-circuit failures.
* **Motorized Ball Valve (`RPBALLVALVE` = 7 on GPIO 5):** 
  * Active-Low relay logic.
  * **Lifecycle / State Automation:** Energized/closed immediately on entering any active state (`STATE_FILL`, `WARM`, `WASH`, `RINSE`, `PAUSE`, `SHUT`).
  * **5-Minute Delayed Reopening:** Upon entering `STATE_OFF`, the valve remains closed for 5 minutes (`300,000 ms`) before de-energizing/opening, avoiding unnecessary valve cycles during back-to-back operations.
  * **Manual GUI Control:** Can be manually toggled via `'v'` command at any time.

## 4. Network, Telemetry & Server Architecture

* **Primary Server:** Raspberry Pi at `10.0.0.9` (512GB SSD). Old Pi (`10.0.0.7`) services are permanently disabled.
* **Host & mDNS:**
  * Hostname: `monsoon` (registered via `WiFi.setHostname("monsoon")` for DHCP logs).
  * mDNS: Active at `monsoon.local` via `ESPmDNS`.
* **MQTT Broker:** `mosquitto` on `10.0.0.9` ports `1883` (TCP) and `9001` (WebSockets). Anonymous access enabled.
* **Web GUI:** Hosted via `nginx` at `http://10.0.0.9/monsoon.html`.
  * Distinct status indicators: **Broker Link** (GUI <-> Pi) vs. **Monsoon Link** (ESP32 <-> Pi with live IP/RSSI).
  * Immediate UI feedback for OTA flashing and rebooting states.
* **Telemetry Datalogger (`monsoon_logger.py`):** 
  * Runs as a user systemd service (`monsoon-logger.service`) under `nicolls@10.0.0.9`.
  * Logs to `/home/nicolls/monsoon/logs/`.
  * Generates per-shower CSV files with relative `Elapsed_ms` timestamps and token parsing (`*M*` temp, `*N*` flows, `*F*`/`*f*` pump PWM, `*I*` IP, `*r*` RSSI).
* **OTA Updates:** OTA server on `10.0.0.9/ota/monsoon.json`. Firmware versions are staged via `deploy_ota.py` (wildcard board matching enabled).

## 5. Coding Style

* Maintain `.ino` file extension in the root for Arduino IDE compatibility.
* Always include `#include <Arduino.h>` (or appropriate core headers) at the top of the main file for PlatformIO.
* Prefer explicit function prototypes to avoid "not declared in scope" errors.

## 6. Manual Control & Auto Tuning

* **General Principle:** The GUI has manual control of everything.
* **Auto Functionality:** When using AUTO, allow real-time adjustment to find the "sweet spot".
* **Implementation:** Set up parameters only when *entering* a state or substate. Do not override manual tuning in the loop unless absolutely necessary.
* **Calibration States:** During long calibration/tuning processes (like `CALIBT`), manual overrides (e.g., for the inlet valve) should be possible unless they fundamentally conflict with the measurement being taken. State-entry logic should explicitly disable any automatic circuits that could interfere with manual GUI control.

## 7. Key State Machine & Flow Rules

* **`STATE_FILL`:**
  * `FILL_PREPARE`: Delivery flow set to 40% with top-up enabled to stabilize flow.
  * `FILL_OVERFILL_PUMP`: Calibrated 8.0 LPM flow limit via `setpump_lpm(RPUMPD, 8.0f)` while recovery is OFF.
* **`STATE_WARM` (Prepare):** Heats to `temp_setpoint + 2.0°C` with low-flow pulsing for mixing and thermal inertia. Does not auto-advance.
* **`STATE_WASH` (Shower):**
  * **PID Speed Control (`TCSPEED`):** Output limits `[3.0, 8.0]` LPM.
  * **Conservative Soft-Start:** Pre-seeded to 4.0 LPM with an 8-second soft-start window (`min(target, 4.0 LPM)` until recovery returns $\ge 2.0\text{ LPM}$).
  * **Mass-Balance Flow Guard:** Delivery flow is dynamically clamped: $\text{Target Delivery} \le \max(3.0\text{ LPM},\; \text{Flow}_{\text{recovery}} - 1.0\text{ LPM})$.
  * **Scavenge Target:** Recovery pump targets $\text{Flow}_{\text{delivery}} + 1.0\text{ LPM}$.
  * **Stabilized Thermal Override:** After 25s post-entry stabilization, if temp $> \text{setpoint} + 1.5^\circ\text{C}$ (e.g. flow clamped due to blocked scavenge), heaters are cut (`ROFF`), re-engaging when temp $\le \text{setpoint} + 0.5^\circ\text{C}$.

## 8. Current Development Focus

* **Single Pump Operation:** We are currently *not* doing anything related to single-pump operations (e.g., `STATE_SETUP1`, `STATE_WARM1`, `STATE_WASH1`, single pump auto-advances). There is no need to modify, refactor, or update any of these functions.
