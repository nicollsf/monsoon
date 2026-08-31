# Complete Pin Mapping & Hardware Guide: ESP32 DevKit V1

This document outlines the optimized pin layout and external resistor requirements for your new **ESP32 DevKit V1** build. 

To maximize future-proofing, we have retired unused features (One-Wire temperature and capacitive level sensors), packed all dedicated inputs onto input-only pins, and routed relays to safe, non-strapping general-purpose GPIOs.

---

## 1. Physical Pin-by-Pin Layout (Top to Bottom)

| LEFT HEADER (Top to Bottom) | Monsoon Usage | | | RIGHT HEADER (Top to Bottom) | Monsoon Usage |
| :--- | :--- | :---: | :---: | :--- | :--- |
| **1. EN** (Reset Button) | Hardware Reset | | | **16. D23** (GPIO 23) | `RPINS[2]` (Relay 2 - Delivery) |
| **2. VP** (GPIO 36) | `LSPINS[0]` (Float Switch Low) | | | **17. D22** (GPIO 22) | `RPINS[6]` (Relay 6 - Burp) |
| **3. VN** (GPIO 39) | `TSA2PIN` (Analog NTC Temp) | | | **18. TX0** (GPIO 1) | Console UART TX (via USB) |
| **4. D34** (GPIO 34) | `LSPINS[1]` (Float Switch High) | | | **19. RX0** (GPIO 3) | Console UART RX (via USB) |
| **5. D35** (GPIO 35) | `PSPIN` (Pressure Analog) | | | **20. D21** (GPIO 21) | `RPINS[5]` (Relay 5 - Pump) |
| **6. D32** (GPIO 32) | `RLEVRXPIN` (Range RX) | | | **21. D19** (GPIO 19) | `RPINS[1]` (Relay 1 - Drain) |
| **7. D33** (GPIO 33) | **FREE** *(General GPIO)* | | | **22. D18** (GPIO 18) | `RPINS[0]` (Relay 0 - Inlet) |
| **8. D25** (GPIO 25) | `FSPINS[0]` (Delivery Flow) | | | **23. D5** (GPIO 5) | **FREE** *(General GPIO)* |
| **9. D26** (GPIO 26) | `FSPINS[1]` (Recovery Flow) | | | **24. TX2** (GPIO 17) | `TXD2` (Bluetooth TX) |
| **10. D27** (GPIO 27) | `SC_PWM1` (Speed Control 1) | | | **25. RX2** (GPIO 16) | `RXD2` (Bluetooth RX) |
| **11. D14** (GPIO 14) | `SC_PWM0` (Speed Control 0) | | | **26. D4** (GPIO 4) | `RPINS[3]` (Relay 3 - Heater A) |
| **12. D12** (GPIO 12) | **FREE** *(General GPIO)* | | | **27. D2** (GPIO 2) | **FREE** *(General GPIO / On-board LED)* |
| **13. D13** (GPIO 13) | `RPINS[4]` (Relay 4 - Heater) | | | **28. D15** (GPIO 15) | `RLEVTXPIN` (Range TX) |
| **14. GND** | Ground Reference | | | **29. GND** | Ground Reference |
| **15. VIN** (5V In) | 5V Power from Buck | | | **30. 3V3** (3.3V Out) | 3.3V Power Out (Pull-up Rail) |

---

## 2. External Resistor Requirements

Because the ESP32 input-only pins (GPIO 34, 35, 36, 39) lack internal software-configurable pull-up resistors, you must add external resistors to the following circuits:

### A. Flow Sensors (GPIO 25 & 26)
* **Type**: Pull-up Resistors.
* **Connection**: One resistor connected from **GPIO 25** to **3.3V**, and one from **GPIO 26** to **3.3V**.
* **Value**: **$2.2\text{k}\Omega$ to $4.7\text{k}\Omega$**. 
* *Rationale*: Hall-effect flow sensors have open-drain outputs. Using a stronger pull-up (like $2.2\text{k}\Omega$ or $4.7\text{k}\Omega$ instead of the generic $10\text{k}\Omega$) ensures sharper logic transitions and limits susceptibility to high-frequency electromagnetic noise.

### B. Float Switches (GPIO 36 & 34)
* **Type**: Pull-up Resistors.
* **Connection**: One resistor from **GPIO 36** to **3.3V**, and one from **GPIO 34** to **3.3V**.
* **Value**: **$4.7\text{k}\Omega$ to $10\text{k}\Omega$**.
* *Rationale*: Float switches are simple dry-contact mechanical switches to GND. Since these pins are input-only and have no internal pull-ups, external resistors are required to hold the inputs High (`State=1`) when the float switches are open.

### C. NTC Thermistor Temperature Sensor (GPIO 39)
* **Type**: Voltage Divider (Ratiometric).
* **Connection**: 
  * Connect a **$15\text{k}\Omega$ (measured at $14.93\text{k}\Omega$ in code)** precision metal-film resistor from **GPIO 39** to **3.3V**.
  * Connect the NTC thermistor from **GPIO 39** to **GND**.
* *Rationale*: This matches the calculation formula in `sensors.cpp`: $R_{NTC} = 14.93\text{k}\Omega \times \frac{ADC}{4095 - ADC}$.

### D. Analog Pressure Sensor (GPIO 35)
* **Type**: Voltage Divider (Attenuation).
* **Connection**: Connect a **$10\text{k}\Omega$ resistor** from the sensor output to **GPIO 35**, and a **$20\text{k}\Omega$ resistor** from **GPIO 35** to **GND**.
* *Rationale*: Scales the $0-5\text{V}$ sensor output down to a safe $0-3.3\text{V}$ range to prevent over-voltage damage to the ESP32 ADC input.

---

## 3. Updated Code Configuration for `system.h`

```cpp
// ----------------------------------------------------------------------
//   Code Pin Definitions (ESP32 DevKit V1)
// ----------------------------------------------------------------------

// Relay Output Pins (all on safe, non-strapping bidirectional pins)
const int RPINS[7] = { 
  18, // RPINS[0] = Inlet Valve (D18)
  19, // RPINS[1] = Drain Valve (D19)
  23, // RPINS[2] = Delivery Valve (D23)
  4,  // RPINS[3] = Heater A (D4 - moved from 5, safe from boot strap)
  13, // RPINS[4] = Main Heater (D13)
  21, // RPINS[5] = Main Pump Power (D21 - moved from 12)
  22  // RPINS[6] = Burp Valve (D22 - moved from 2)
};  
const int RPINS_ROFF[7] = { HIGH, HIGH, HIGH, HIGH, LOW, LOW, LOW };  

// Float Switch Inputs (Lsw low high - on input-only pins)
const int LSPINS[2] = { 
  36, // LSPINS[0] = Float Switch Low (VP - input-only, needs external pull-up)
  34  // LSPINS[1] = Float Switch High (D34 - input-only, needs external pull-up)
};       
const int LSPINSlv[2] = { LOW, HIGH };  

// Capacitive Level Sensor Inputs (Retired / Disabled)
// const int LSCPINS[2] = { 25, 34 };       
// const int LSCPINSlv[2] = { LOW, LOW };  

// Temperature Sensor Pins
// const int TSAPIN = 4;   // OneWire Digital (Retired / Disabled - freed GPIO 4 for Relay 3)
const int TSA2PIN = 39;   // Analog NTC 15k thermistor (VN - moved from 33 to input-only pin)

// Flow Sensor Inputs (Interrupts on native digital pins with Schmitt triggers)
const int FSPINS[2] = { 
  25, // FSPINS[0] = Delivery Flow (D25 - moved from 36)
  26  // FSPINS[1] = Recovery Flow (D26 - moved from 39)
};       

// Speed Controller (H-Bridge PWM)
const int SC_PWM0 = 14;   // D14
const int SC_PWM1 = 27;   // D27   

// Analog Pressure Sensor
const int PSPIN = 35;     // D35 (input-only)

// Level Range Sensor (UART)
const int RLEVTXPIN = 15; // Level Range TX (D15)
const int RLEVRXPIN = 32; // Level Range RX (D32)
```

## 4. Wemos to ESP32 DevKit V1 Wire Translation Guide (Physical Layout)

This table corresponds directly to the physical left and right headers of the **ESP32 DevKit V1** module, showing you exactly which Wemos-labeled wire to plug into each pin.

| LEFT HEADER (Pins 1–15) | Wemos Wire Label | | | RIGHT HEADER (Pins 16–30) | Wemos Wire Label |
| :--- | :--- | :---: | :---: | :--- | :--- |
| **1. EN** (Reset Button) | *(No wire - onboard reset)* | | | **16. D23** (GPIO 23) | **23** (Delivery Valve Relay) |
| **2. VP** (GPIO 36) | **21** (Float Switch Low) `[pullup]` | | | **17. D22** (GPIO 22) | **2** (Burp Valve Relay) |
| **3. VN** (GPIO 39) | **33** (Analog NTC Temp) `[divider]` | | | **18. TX0** (GPIO 1) | *(No wire - USB Console TX)* |
| **4. D34** (GPIO 34) | **22** (Float Switch High) `[pullup]` | | | **19. RX0** (GPIO 3) | *(No wire - USB Console RX)* |
| **5. D35** (GPIO 35) | **35** (Pressure Sensor) `[divider]` | | | **20. D21** (GPIO 21) | **12** (Main Pump Power Relay) |
| **6. D32** (GPIO 32) | **32** (Range Sensor RX) | | | **21. D19** (GPIO 19) | **19** (Drain Valve Relay) |
| **7. D33** (GPIO 33) | **FREE** *(Was Analog Temp)* | | | **22. D18** (GPIO 18) | **18** (Inlet Valve Relay) |
| **8. D25** (GPIO 25) | **36** (Delivery Flow) `[pullup]` | | | **23. D5** (GPIO 5) | **FREE** *(Was Heater A Relay)* |
| **9. D26** (GPIO 26) | **39** (Recovery Flow) `[pullup]` | | | **24. TX2** (GPIO 17) | **17** (Bluetooth TX) |
| **10. D27** (GPIO 27) | **27** (Speed Control PWM1) | | | **25. RX2** (GPIO 16) | **16** (Bluetooth RX) |
| **11. D14** (GPIO 14) | **14** (Speed Control PWM0) | | | **26. D4** (GPIO 4) | **5** (Heater A) / **4** *(Retired - tape off)* |
| **12. D12** (GPIO 12) | **FREE** *(Was Scavenge Relay)* | | | **27. D2** (GPIO 2) | **FREE** *(Was Burp Relay)* |
| **13. D13** (GPIO 13) | **13** (Main Heater Relay) | | | **28. D15** (GPIO 15) | **15** (Range Sensor TX) |
| **14. GND** | **GND** (Ground Reference) | | | **29. GND** | **GND** (Ground Reference) |
| **15. VIN** (5V In) | **5V / VIN** (Buck Power) | | | **30. 3V3** (3.3V Out) | **3V3** (Pull-up Rail Supply) |

---

### Legend & Wiring Notes:
* `[pullup]`: External $4.7\text{k}\Omega$ pull-up resistor required from this pin to the **3.3V rail (Pin 30)**.
* `[divider]`: Requires voltage divider circuitry (see Section 2 for exact resistor specs).
* **Wire Label 4 (Old OneWire Temp)**: Fully retired. Do not connect it to Pin 26 (D4). Connect **Wire Label 5 (Heater A)** to Pin 26 instead.

