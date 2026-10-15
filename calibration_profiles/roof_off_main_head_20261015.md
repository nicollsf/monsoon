# Thermal Calibration Profile: "shower with roof off and through main shower head"
Date of Calibration: 2026-10-15 19:43:40
Raw Data Source: `calibt_roof_off_main_head_20261015.csv` (originating from Pi `20261015_194340_shower.csv`)
Target Flow Rate: 5.0 LPM (Nominal) | Measured Average Flow: 4.77 ± 0.05 LPM
Systematic Condition: Stall roof removed; recovery filter freshly cleaned.

---

## 1. Physical Enclosure & Hardware Configuration
* **Enclosure:** Shower stall with roof/ceiling panel **REMOVED** (open top).
* **Shower Head:** Main overhead high-flow shower head.
* **Filter State:** Freshly cleaned recovery lint/hair filter.
* **Ambient Equilibrium:** $T_{\text{amb}} = \mathbf{28.32^\circ\text{C}}$ (vs. $28.11^\circ\text{C}$ yesterday with roof on).
* **Tank Geometry:** Cylindrical tank, 18 cm diameter (~2.55 L per 10 cm height).
* **Sensors & Heaters:** Bottom-mounted NTC thermistor adjacent to dual asymmetric elements (Main SSR 4 kW + Aux mechanical 2 kW = 6 kW).

---

## 2. Identified First-Order Plus Dead Time (FOPDT) Parameters

$$G(s) = \frac{K}{\tau s + 1} e^{-\tau_{\text{dead}} s}$$

| Parameter | Roof OFF (Clean Filter) | Roof ON (Dirty Filter) | Direct Difference | Physical Interpretation |
| :--- | :---: | :---: | :---: | :--- |
| **`label`** | `"roof off main head"` | `"roof on main head"` | — | Enclosure configuration identifier |
| **Apparent Dead Time ($\tau_{\text{dead}}$)** | **$22.61\text{ s}$** | $19.44\text{ s}$ | $+3.17\text{ s}$ | Internal conduction lag through sheath/thermowell |
| **Heating Time Constant ($\tau_{\text{heat}}$)** | **$266.2\text{ s}$ ($4.44\text{ m}$)** | $232.8\text{ s}$ ($3.88\text{ m}$)** | $+33.4\text{ s}$ ($+14.3\%$) | Slower thermal saturation due to higher droplet heat shedding |
| **Process Gain ($K$)** | **$4.85^\circ\text{C / kW}$** | $4.83^\circ\text{C / kW}$ | $+0.02^\circ\text{C / kW}$ | Net asymptotic steady-state gain |
| **Steady-State Loss ($\text{UA}_{\text{ss}}$)** | **$219.4\text{ W/}^\circ\text{C}$** | **$206.9\text{ W/}^\circ\text{C}$** | **$+12.5\text{ W/}^\circ\text{C}$ ($+6.0\%$)** | Active steady-state heat loss conductance at ~50°C hold |
| **Cooling Decay Constant ($\tau_{\text{cool}}$)** | **$348.4\text{ s}$ ($5.81\text{ m}$)** | **$457.5\text{ s}$ ($7.62\text{ m}$)** | **$-109.1\text{ s}$ ($-23.8\%$)** | Time to decay to $1/e$ of temperature elevation |
| **Cooling Half-Life ($t_{1/2}$)** | **$241.5\text{ s}$ ($4.02\text{ m}$)** | **$317.1\text{ s}$ ($5.29\text{ m}$)** | **$-75.6\text{ s}$ ($-23.8\%$)** | Time to shed 50% of temperature elevation |
| **Relative Cooling Rate** | **$-3.73^\circ\text{C/min}$** | **$-2.87^\circ\text{C/min}$** | **$+30.1\%$ Faster Loss** | Evaluated at 50°C during unheated decay |
| **Filter Recovery Load (`PWM1`)** | **$51.9\%$** | **$56.0\%$** | **$-4.1\%$ PWM relief** | Hydraulic benefit of freshly cleaned recovery filter |

---

## 3. Physical Mechanisms of Heat Loss: Roof ON vs. Roof OFF

1. **Suppression of Evaporation with Roof ON:**
   With the ceiling panel installed, the enclosure quickly becomes saturated with hot water vapor (relative humidity approaches 100%). In a saturated environment, the vapor pressure gradient between falling water droplets and surrounding air is small, dramatically suppressing droplet evaporation. The enclosed steam blanket insulates the stream.

2. **Open-Plume Evaporative Siphon with Roof OFF:**
   With the roof off, natural buoyancy drives a continuous warm updraft (convective plume) carrying steam out of the top into the room. Cool, relatively dry room air is drawn in from the lower edges. The falling droplets experience vigorous mass transfer / evaporative cooling ($\Delta h_{\text{vap}} \approx 2260\text{ kJ/kg}$). Even losing a fraction of a percent of circulating water mass through steam carries away enormous latent heat, driving the **$+31.3\%$ acceleration in cooling decay**.

3. **Steady-State Impact on Power Budget:**
   At typical showering conditions ($48^\circ\text{C}$ setpoint with $28^\circ\text{C}$ ambient, $\Delta T = 20^\circ\text{C}$):
   * With Roof ON: Steady-state thermal loss $\approx 206.9\text{ W/}^\circ\text{C} \times 20^\circ\text{C} = \mathbf{4.14\text{ kW}}$.
   * With Roof OFF: Steady-state thermal loss $\approx 219.4\text{ W/}^\circ\text{C} \times 20^\circ\text{C} = \mathbf{4.39\text{ kW}}$.
   * Additional continuous heater power required to sustain identical temperature: $\approx \mathbf{+250\text{ W}}$.

---

## 4. Hydraulic Impact: Cleaned Recovery Filter

At the exact same calibrated delivery flow ($4.76\text{ LPM}$):
* **Dirty Filter (Yesterday):** Recovery pump required **$56.0\%\text{ PWM}$** to maintain scavenge balance.
* **Clean Filter (Today):** Recovery pump required only **$51.9\%\text{ PWM}$**.
* **Net Benefit:** Cleaning the filter restored **$4.1\%\text{ PWM}$ of hydraulic headroom**, shifting the recovery pump further away from the $80\%$ limit and allowing delivery flow to reach higher setpoints before hitting hydraulic limits.
