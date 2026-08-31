# Monsoon System State & DevKit V1 Hardware Migration

This file documents the key findings, hardware constraints, and design decisions made during the transition from the **Wemos D1 R32** to the **ESP32 DevKit V1** module.

---

## 1. Pin Migration Decisions & Motivation

### A. Strapping Pins (Boot Safeguards)
* **Strapping Pins (GPIO 12, 15, 2, 5)** can prevent the ESP32 from booting if pulled high/low externally during startup.
* **Relay Safeguard**: All high-power relay outputs were moved off critical strapping pins to ensure the board always boots cleanly.
* **Bespoke Exception**: GPIO 15 (MTDO) is used for the Level Range TX serial pin. Since UART TX idles High, this aligns naturally with the default boot requirements.

### B. Input-Only Pin Constraints (GPIO 34, 35, 36, 39)
* **No Internal Pull-ups**: These pins do not have internal software pull-up/pull-down resistors. External physical pull-ups are mandatory for dry contacts (like float switches).
* **No Outputs**: These pins have no output drivers and can only be used as inputs.

### C. Flow Sensor Migration (ADC1 Crosstalk Isolation)
* **Finding**: The flow sensors were previously connected to GPIO 36 and 39. During ADC1 operations (reading the pressure sensor or NTC thermistor), internal multiplexer charging caused high-frequency crosstalk spikes on GPIO 36 and 39. This triggered false interrupts on the flow sensors.
* **Action**: Moved the Flow Sensors to **GPIO 25 and 26** (general bidirectional digital pins). These pins possess internal Schmitt triggers and are electrically isolated from ADC1 crosstalk, resolving the counting noise completely.
* **Resistor Strength**: Internal pull-ups ($30\text{k}\Omega-80\text{k}\Omega$) are too weak to fight cable capacitance and electromagnetic noise from pumps. External **$4.7\text{k}\Omega$** pull-ups are added to GPIO 25 and 26.

---

## 2. External Resistor Specifications

### Float Switches (GPIO 36 & 34)
* **Pull-up Value**: **$4.7\text{k}\Omega$ to $10\text{k}\Omega$** connected to **3.3V**.
* **Reason**: Input-only pins lack internal pull-ups. Stronger pull-ups ($4.7\text{k}\Omega$) improve noise immunity for long wire runs.

### NTC Thermistor Temperature Sensor (GPIO 39)
* **Divider Connection**: 
  * A **$15\text{k}\Omega$** (nominal, measured precisely as **$14.93\text{k}\Omega$** in code) metal-film resistor connected from **GPIO 39** to **3.3V**.
  * The NTC thermistor connected from **GPIO 39** to **GND**.
* **Math Match**: This corresponds to the code ratiometric formula: $R_{NTC} = 14.93\text{k}\Omega \times \frac{ADC}{4095 - ADC}$.

### Pressure Sensor (GPIO 35)
* **Divider Connection**: **$10\text{k}\Omega$** resistor from sensor output to **GPIO 35**, and **$20\text{k}\Omega$** resistor from **GPIO 35** to **GND** (attenuates $5\text{V}$ output down to safe $3.3\text{V}$ range).

---

## 3. Power Architecture & Pump Gatekeeper Safety
* **Series Safety Relay**: The pumps are powered via a physical mechanical relay in series with the PWM H-bridge speed controllers.
* **Why**: Silicon motor drivers (MOSFETs) typically fail as a **short-circuit** (forcing the pumps permanently ON). The series relay provides an independent, physical air-gap cutoff that the ESP32 safety vetoes can open to kill power, ensuring fail-safe protection.
