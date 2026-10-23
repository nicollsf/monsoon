Goal: Maintain interchangeable compatibility across Arduino IDE and PlatformIO for the Monsoon recirculating shower system.

## Primary System Objective: The Monsoon Experience
All engineering in this codebase is directed towards two overarching goals:
1. **Fast Stabilization:** Getting the system into stable operation as quickly as possible upon entering `WASH`.
2. **Maximum Flow at Setpoint:** Keeping the delivered flow rate **as high as possible** (the defining "Monsoon" high-flow drenching experience) at the user's setpoint temperature, without exceeding hydraulic recovery limits.
3. **Simplicity & Robustness:** Keep control algorithms simple, deterministic, and robust. Avoid over-complicated models or estimators that introduce fragile edge cases on the embedded microcontroller.

## 1. Directory & Scoping Rules

* **Active Firmware Code:** Located in `arduino/monsoon_esp4.2/`.
* **Repository Root (`.`):** Top-level architecture, documentation, CAD/tank models (`tanks/`), datasheets (`datasheets/`), and deployment tools (`deploy_ota.py`).
* **Excluded Folders:** Ignore `attic/`, `test/`, `temp/`, and legacy `arduino/monsoon_esp4.1/` for active coding suggestions. These contain legacy or experimental code that conflicts with the current architecture.
* **Environment:** PlatformIO using the ESP32 v3.0 core in the active firmware directory.

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

* **Heaters (Dual Asymmetric Elements - 6 kW Total):**
  * **Main Heater (`RPHEATER` = 4 on GPIO 13):** **4 kW** geyser element (66.7% power).
  * **Main SSR Control (`SSR_MAIN_PIN` = GPIO 33 / D33):** Solid-State Relay (Fotek/HKD SSR-40DA) in series with mechanical relay `RPHEATER` for Slow-PWM burst fire duty modulation (1000ms window timebase). Gated by `safety_veto` and `RPHEATER` relay state. Provides seamless, gap-free proportional power from $0.0\text{ kW}$ to $4.0\text{ kW}$ ($0\dots 100\%$).
  * **Auxiliary Heater (`RPHEATERA` = 3 on GPIO 4):** **2 kW** kettle element (33.3% power) on mechanical relay (ON/OFF). Provides $+2.0\text{ kW}$ base boost, extending continuous modulation to $2.0\text{--}6.0\text{ kW}$ with zero dead zones.
  * **Empirical Thermal Capacity:** $\sim 1\text{ kW}$ per $1\text{ LPM}$ flow ($\text{4 kW} \approx 4\text{--}5\text{ LPM}$, $\text{6 kW} \approx 6\text{--}7\text{ LPM}$).
  * **Continuous Modulation & Staging:**
    * High Power Range ($2.0\dots 6.0\text{ kW}$): Aux ON ($2\text{ kW}$) + Main SSR ($0\dots 100\% \rightarrow 0\dots 4\text{ kW}$).
    * Low Power Range ($0.0\dots 4.0\text{ kW}$): Aux OFF ($0\text{ kW}$) + Main SSR ($0\dots 100\% \rightarrow 0\dots 4\text{ kW}$).
    * Cutout ($0\text{ kW}$): Both OFF.

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
  * Generates per-shower CSV files with relative `Elapsed_ms` timestamps and token parsing (`*M*` temp & power, `*N*` flows, `*W*` aux duty %, `*F*`/`*f*` pump PWM, `*I*` IP, `*r*` RSSI).
* **OTA Updates & Safety Guard:** 
  * OTA server on `10.0.0.9/ota/monsoon.json`. Firmware versions are staged via `deploy_ota.py` (wildcard board matching enabled).
  * **Safety Guard:** OTA updates and flashing are strictly blocked if the system is in an active state (`auto_state != STATE_OFF`) to prevent abrupt de-energization or resets during active heating/showering.

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
  * `FILL_PREPARE`: Delivery flow set to 40% with top-up enabled to stabilize flow (12.5s top-up timeout and repeat guard).
  * `FILL_OVERFILL_PUMP`: Calibrated 8.0 LPM flow limit via `setpump_lpm(RPUMPD, 8.0f)` while recovery is OFF.
* **`STATE_WARM` (Two-Stage Preheating Pipeline):**
  * Auto top-up configured with **120s** inactivity timeout (`auto_wtopup_timeout = 120000;`) and **30s** repeat pulse guard (`auto_wtopup_interval = 30000;`). Closed-loop scavenge handles all normal tank level replenishment without cold mains water intrusion; if level is lost for 120s, top-up fires 2s pulses every 30s until `tank_full` is recovered.
  * **`WARM_RAMP` (Fast Heat-Up):**
    * Continuous forced convection at $\sim 3.5\text{--}3.8\text{ LPM}$ with full $6\text{ kW}$ power (Main 4kW + Aux 2kW).
    * Delivers a linear heating rate of $\sim +4.9^\circ\text{C/min}$ (reaching setpoint in $\sim 5\text{ min}$ from cold) with zero sensor lag or false thermal trips.
    * Automatically advances to `WARM_HOLD` when water temperature is within $0.8^\circ\text{C}$ of `temp_setpoint`.
  * **`WARM_HOLD` (Steady Thermal Soak):**
    * Continuous low-flow circulation at $\sim 2.2\text{--}2.5\text{ LPM}$.
    * Dynamically modulates asymmetric heating stages ($0\text{ kW} / 2\text{ kW} / 4\text{ kW} / 6\text{ kW}$) using tight hysteresis around setpoint.
    * Holds water equilibrium within $\pm 0.5^\circ\text{C}$ indefinitely at $\sim 1.9\text{ kW}$ maintenance power until the user manually triggers `WASH`.
* **`STATE_WASH` (Two-Regime Decoupled Control Architecture):**
  * Auto top-up configured with **120s** inactivity timeout (`auto_wtopup_timeout = 120000;`) and **30s** repeat pulse guard (`auto_wtopup_interval = 30000;`).
  * **Regime 1 (`WASH_FLOW_CTRL` - Flow-Based Temperature Control):**
    * Heaters locked on **FULL 6.0 kW** power (Main 4kW SSR @ 100% + Aux 2kW ON).
    * Damped inner PID (`TCSPEED`, evaluated every 2500ms, $Kp=0.35$, $Ki=0.015$, $Kd=0.15$, limits `[3.6, 8.5]` LPM with mathematically strict $0.12\text{ LPM/s}$ continuous slew rate limiting) hunts for maximum drenching flow ("Monsoon" experience) by modulating delivery pump flow to reject the full 6 kW heat input and clamp water at `temp_setpoint`.
    * **Hydraulic Transition Trigger:** Evaluates recovery pump load (`sc_setperc[RPUMPR]`) and heater headroom. When recovery PWM $> 78\%$ continuously for $\ge 8.0\text{s}$ after $\ge 30\text{s}$ settling in `WASH_FLOW_CTRL`, AND there is verified heater headroom ($T \ge T_{\text{setpoint}} - 0.5^\circ\text{C}$), the system measures and latches `wash_locked_flow = constrain(flow_lpm0, 3.6f, 7.5f)` and executes a bumpless transition to Regime 2.
    * *Winter Mode:* Under high environmental heat loss where recovery never reaches 78% or heater headroom is absent, the system remains stably in `WASH_FLOW_CTRL` for the entire shower.
  * **Regime 2 (`WASH_HEATER_CTRL` - Heater-Based Temperature Control):**
    * **Fixed Recovery Pump:** Recovery pump is fixed at **$80.0\%\text{ PWM}$** (maximum safe, non-cavitating throughput $\sim 9.4\text{ LPM}$).
    * **Delivery Pump Tank Level Seeking (Algebraic Duality):** Tank level control is transferred to the delivery pump using the inverse level relation: $\text{PWM}_{\text{delivery}} = \frac{80.0 - \text{scavenge\_pwm\_trim}}{1.13}$. If `tank_full == false`, delivery gently sheds flow to balance throughput; if `tank_full == true`, delivery creeps up towards maximum drenching flow.
    * **Manual GUI Nudge:** The GUI delivery pump up/down buttons (`'c'` / `'C'`, $\pm 5\%$ PWM) remain fully active and directly shift `scavenge_pwm_trim` by $\mp (1.13 \times 5\%)$, moving the seeking baseline while maintaining level regulation.
    * **Bumpless Transfer:** On transition, pre-seeds Main SSR duty according to excess temp above setpoint ($80\%$ at setpoint, scaled down if hotter).
    * **Heater PID (`TCHEATER`):** Responsive PID loop (sample time 1500ms, DIRECT mode, $Kp=0.25$, $Ki=0.008$, $Kd=0.50$, limits `[0.05, 1.0]`) modulating Main SSR duty cycle with Aux 2kW ON, providing continuous heat from $2.0\text{ kW}$ to $6.0\text{ kW}$.
    * **Thermal Saturation Fallback Guard:** If heaters are saturated at 100% (6.0 kW: Main SSR $\ge 98\%$ + Aux ON) and temperature remains below setpoint ($T < T_{\text{setpoint}} - 0.3^\circ\text{C}$) continuously for $\ge 30\text{s}$, the system executes a bumpless fallback to `WASH_FLOW_CTRL` (with 60s re-transition cooldown) so `TCSPEED` can gently trim flow to restore setpoint temperature.
  * **Dual Ambient Tracking (90-Minute Rule):** Tracks both cold fill water temperature ($T_{\text{water}}$, updated in `FILL_FULL` if $>90$m since last shower) and idle air temperature ($T_{\text{air}}$, updated in `STATE_OFF` if $>90$m). Persisted to NVRAM Preferences (`monsoon_env`).
  * **Unified Dual-Mode Float-Centric Balancing (`loop_autoscavengecontrol`):**
    * Shared state variable `scavenge_pwm_trim` running symmetric seeking dynamics:
      * Continuous ramp: $\pm 0.04\%$ per $500\text{ ms} = \mathbf{\pm 0.08\%/\text{s}}$ ($4.8\%/\text{min}$).
      * Hysteresis step: $\mathbf{\pm 1.5\%\text{ PWM}}$ step on float state flip.
      * Anti-windup midpoint return: rolls back $\mathbf{50\%}$ of the accumulated half-cycle ramp upon float transition.
    * **Mode 1 (`WARM` & `WASH_FLOW_CTRL`):** Recovery tracks delivery: $\text{PWM}_{\text{recovery}} = (1.13 \times \text{PWM}_{\text{delivery}}) + \text{scavenge\_pwm\_trim}$.
    * **Mode 2 (`WASH_HEATER_CTRL`):** Recovery fixed at $80.0\%$, delivery modulates level: $\text{PWM}_{\text{delivery}} = \frac{80.0 - \text{scavenge\_pwm\_trim}}{1.13}$.
    * Stateless across showers (resets only in `STATE_OFF` and `STATE_FILL`), preserved across `WARM` $\rightarrow$ `WASH`.
* **Flow & Temperature Sensor Filtering:**
  * 3-trace telemetry stream (`*N<flow_del>,<flow_rec_raw>,<flow_rec_est>*`) with 5-point piecewise linear recovery calibration.
  * 9-sample ADC median filter in `get_tempsens1()`.
  * 5-point rolling window median filter on temperature readings.

## 8. Current Development Focus

* **Single Pump Operation:** We are currently *not* doing anything related to single-pump operations (e.g., `STATE_SETUP1`, `STATE_WARM1`, `STATE_WASH1`, single pump auto-advances). There is no need to modify, refactor, or update any of these functions.
