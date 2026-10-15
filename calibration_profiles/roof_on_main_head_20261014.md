# Thermal Calibration Profile: "shower with roof on and through main shower head"
Date of Calibration: 2026-10-14 17:57:29
Raw Data Source: `calibt_roof_on_main_head_20261014.csv` (originating from `20261014_175729_shower.csv`)
Target Flow Rate: 5.0 LPM (Nominal) | Measured Average Flow: 4.76 ± 0.05 LPM

---

## 1. Physical Enclosure & Hardware Configuration
* **Enclosure:** Shower stall with roof/ceiling panel installed.
* **Shower Head:** Main overhead high-flow shower head.
* **Tank Geometry:** Cylindrical tank, 18 cm diameter (~2.55 L per 10 cm height), ~15–18 Litres loop volume.
* **Sensor & Element Placement:** Heating elements (4 kW geyser + 2 kW kettle) are located at the bottom of the tank. The control NTC thermistor is mounted virtually right next to the elements.
* **Pumps:** Dual-pump closed-loop configuration (Delivery = RPUMPD / Recovery = RPUMPR with closed-loop dynamic scavenge).
* **Heaters:** Dual asymmetric elements (Main SSR 4 kW + Aux mechanical 2 kW = 6 kW maximum).

---

## 2. Identified First-Order Plus Dead Time (FOPDT) Parameters

$$G(s) = \frac{K}{\tau s + 1} e^{-\tau_{\text{dead}} s}$$

| Parameter | Calibrated Value | Physics & Description |
| :--- | :---: | :--- |
| **`label`** | `"shower with roof on and through main shower head"` | Human-readable identifier stored in ESP32 NVRAM (`therm_models` namespace). |
| **Apparent Dead Time ($\tau_{\text{dead}}$)** | **$19.44\text{ seconds}$** | Internal thermal conduction lag: delay for heat to conduct through element ceramic insulation/metal sheath into water, convective mixing in the 18 cm tank base, and conduction through the thermowell/potting into the NTC bead. |
| **Time Constant ($\tau$)** | **$232.8\text{ seconds}$ ($3.88\text{ min}$)** | Rate at which bulk circulating water and internal plumbing reach thermal equilibrium ($R^2 = 0.9955$). |
| **Thermal Conductance ($\text{UA}$)** | **$206.9\text{ W/}^\circ\text{C}$** | Steady-state thermal heat loss conductance to ambient at $50^\circ\text{C}$. |
| **Process Gain ($K$)** | **$4.83^\circ\text{C / kW}$** | Asymptotic temperature rise per kW applied ($K = 1 / \text{UA}$). |
| **Active Thermal Capacity ($C_{\text{th}}$)** | **$48.2\text{ kJ/}^\circ\text{C}$** | Effective active circulating thermal mass ($C_{\text{th}} = \tau \times \text{UA}$). |
| **Equivalent Water Volume ($V_{\text{eff}}$)** | **$11.5\text{ Litres}$** | Equivalent circulating water mass absorbing heat during transient step. |
| **Reference Delivery Flow ($Q_{\text{ref}}$)** | **$4.76\text{ LPM}$** | Calibrated reference delivery flow rate. |

---

## 3. Step Test Phase Progression

1. **Phase 1: Baseline Idle (`CALIBT_BASELINE`, $t = 143.3\text{s} \to 232.9\text{s}$)**
   * Power: $0\text{ kW}$ (Heaters OFF).
   * Delivery Flow: $4.71\text{ LPM}$.
   * Equilibrium Ambient Temperature: $T_{\text{amb}} = \mathbf{28.11^\circ\text{C}}$.

2. **Phase 2: 6.0 kW Step Heating (`CALIBT_STEP_HEAT`, $t = 232.9\text{s} \to 580.6\text{s}$)**
   * Power Step: Instantaneous step from $0.0\text{ kW} \to 6.0\text{ kW}$ (Main 100% SSR + Aux 2 kW ON).
   * Temperature Rise: $28.10^\circ\text{C} \to 50.00^\circ\text{C}$ ($\Delta T = +21.9^\circ\text{C}$).
   * Pure Transport Delay Observed: $19.44\text{ s}$ before thermistor moved above $28.10^\circ\text{C}$.
   * Mean Flow: $4.77\text{ LPM}$.

3. **Phase 3: Stabilize Hold (`CALIBT_STABILIZE`, $t = 580.6\text{s} \to 640.9\text{s}$)**
   * Power Step: Stepped down from $6.0\text{ kW} \to 4.80\text{ kW}$ (Main 70% SSR + Aux 2 kW ON).
   * Stable Equilibrium Temperature: $T_{\text{hold}} = \mathbf{51.31^\circ\text{C}}$ ($\Delta T_{\text{hold}} = 23.20^\circ\text{C}$ above ambient).
   * Steady-State Heat Loss: $\text{UA} = 4800\text{ W} / 23.20^\circ\text{C} = \mathbf{206.9\text{ W/}^\circ\text{C}}$.

4. **Phase 4: Extended Cooling Decay (`CALIBT_COOL_DECAY`, $t = 640.9\text{s} \to 1241.2\text{s}$)**
   * Power: $0\text{ kW}$ (Heaters OFF).
   * Duration: $600.3\text{ s}$ ($10.0\text{ minutes}$).
   * Cooling Decay: $51.50^\circ\text{C} \to 34.60^\circ\text{C}$ (temperature drop of $-16.90^\circ\text{C}$ towards $28.11^\circ\text{C}$ ambient).
   * Mean Flow: $4.76\text{ LPM}$.
