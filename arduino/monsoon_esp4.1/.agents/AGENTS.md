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
  * `level_safe`: Physical float switch check (`digitalRead(LSPINS[0]) != LSPINSlv[0]`).
  * `temp_safe`: Thermal cutoff (`current_temp <= MAX_TEMP_LIMIT`).
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

## 4. Coding Style

* Maintain `.ino` file extension in the root for Arduino IDE compatibility.
* Always include `#include <Arduino.h>` (or appropriate core headers) at the top of the main file for PlatformIO.
* Prefer explicit function prototypes to avoid "not declared in scope" errors.

## 5. Manual Control & Auto Tuning

* **General Principle:** The GUI has manual control of everything.
* **Auto Functionality:** When using AUTO, allow real-time adjustment to find the "sweet spot".
* **Implementation:** Set up parameters only when *entering* a state or substate. Do not override manual tuning in the loop unless absolutely necessary.
* **Calibration States:** During long calibration/tuning processes (like `CALIBT`), manual overrides (e.g., for the inlet valve) should be possible unless they fundamentally conflict with the measurement being taken. State-entry logic should explicitly disable any automatic circuits that could interfere with manual GUI control.

## 6. Key State Machine Use Cases

* **`STATE_WARM` (Prepare):** The goal is to heat the water to the user's `temp_setpoint` and hold it there, ready for use. It uses a low-flow pulsing strategy for mixing. It does not auto-advance, prioritizing convenience over energy efficiency if left idle.
* **`STATE_WASH` (Shower):** This is the active showering state. It uses a PI controller on the delivery pump flow (`TCSPEED` mode) to tightly regulate the temperature around the `temp_setpoint`.

## 7. Current Development Focus

* **Single Pump Operation:** We are currently *not* doing anything related to single-pump operations (e.g., `STATE_SETUP1`, `STATE_WARM1`, `STATE_WASH1`, single pump auto-advances). There is no need to modify, refactor, or update any of these functions.
