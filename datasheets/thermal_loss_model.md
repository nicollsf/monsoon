# Monsoon Recirculating Shower — Thermal Loss & System Dynamics Model

This document records the empirical thermal loss parameters, heat transfer coefficients, and dynamic response characteristics of the Monsoon recirculating shower system, estimated from unheated circulation data during actual shower operations.

---

## 1. Empirical Parameter Summary

| Parameter | Symbol | Estimated Value | Units | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Ambient / Drain Temp** | $T_{amb}$ | **22.9** | $^\circ\text{C}$ | Baseline room / unheated water reference |
| **Active Loop Volume** | $V$ | **15.0** | $\text{L}$ | Water volume in sump, plumbing, and pan |
| **Lumped Thermal Mass** | $C_{th}$ | **62.76** | $\text{kJ/}^\circ\text{C}$ | $M \times C_p = 15\text{ kg} \times 4.184\text{ kJ/(kg}\cdot^\circ\text{C)}$ |
| **Decay Rate Constant** | $k$ | **$0.003024$** | $\text{s}^{-1}$ | Exponential cooling rate ($0.1814\text{ min}^{-1}$) |
| **Thermal Time Constant** | $\tau$ | **$330.7$** | $\text{s}$ | **5.51 minutes** ($\tau = 1/k$) |
| **Thermal Half-Life** | $t_{1/2}$ | **$229.2$** | $\text{s}$ | **3.82 minutes** |
| **Overall Loss Coefficient** | $UA$ | **$189.77$** | $\text{W/}^\circ\text{C}$ | Combined heat transfer coefficient ($C_{th} / \tau$) |

---

## 2. Mathematical Model

### Lumped Capacitance Energy Balance
The temperature $T(t)$ of the circulating water during unheated operation evolves according to:

$$\frac{dT}{dt} = - \frac{1}{\tau} (T(t) - T_{amb}) = - \frac{UA}{M C_p} (T(t) - T_{amb})$$

Integrating over time yields the analytic decay curve:

$$T(t) = T_{amb} + (T_0 - T_{amb}) e^{-t / \tau}$$

### Heat Loss Power as a Function of Water Temperature
The total thermal power lost to the environment (evaporative droplet cooling + spray convection + pan and plumbing conduction) is:

$$P_{loss}(T) = UA \cdot (T - T_{amb})$$

---

## 3. Heat Loss & Cooling Rates at Operating Temperatures ($T_{amb} = 22.9^\circ\text{C}$)

| Water Temp ($T$) | Temperature Delta ($\Delta T$) | Total Heat Loss ($P_{loss}$) | Unheated Cooling Rate ($dT/dt$) | Net Balance @ 4 kW | Net Balance @ 6 kW |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **$40.0^\circ\text{C}$** | $17.1^\circ\text{C}$ | **$3.25\text{ kW}$** | $-3.10^\circ\text{C/min}$ | $+0.75\text{ kW}$ (Warming) | $+2.75\text{ kW}$ (Fast heat) |
| **$42.0^\circ\text{C}$** | $19.1^\circ\text{C}$ | **$3.62\text{ kW}$** | $-3.47^\circ\text{C/min}$ | $+0.38\text{ kW}$ (Slow heat) | $+2.38\text{ kW}$ (Fast heat) |
| **$45.0^\circ\text{C}$** | $22.1^\circ\text{C}$ | **$4.19\text{ kW}$** | $-4.01^\circ\text{C/min}$ | **$-0.19\text{ kW}$ (Deficit!)** | $+1.81\text{ kW}$ (Stable heat) |
| **$48.0^\circ\text{C}$** | $25.1^\circ\text{C}$ | **$4.76\text{ kW}$** | $-4.55^\circ\text{C/min}$ | **$-0.76\text{ kW}$ (Deficit!)** | **$+1.24\text{ kW}$ (Ideal)** |
| **$50.0^\circ\text{C}$** | $27.1^\circ\text{C}$ | **$5.14\text{ kW}$** | $-4.92^\circ\text{C/min}$ | **$-1.14\text{ kW}$ (Deficit!)** | $+0.86\text{ kW}$ (Stable heat) |

---

## 4. Key Engineering Insights

1. **Why 4 kW Alone Cannot Maintain $48^\circ\text{C}$ at Normal Flow:**
   * At $5.5\text{ LPM}$ and $48^\circ\text{C}$, the shower loses **$4.76\text{ kW}$** to the room and drain pan.
   * A single $4.0\text{ kW}$ element running at 100% duty suffers a **$0.76\text{ kW}$ deficit**, causing the water to slowly cool down unless flow is throttled below $\sim 4.0\text{ LPM}$.
2. **Why 6 kW (Dual Elements) is Required for "Monsoon" Flow:**
   * With full $6.0\text{ kW}$ (4 kW Main + 2 kW Aux), there is a **$+1.24\text{ kW}$ net surplus** at $48^\circ\text{C}$, enabling the flow PID to push delivery flow up to $\sim 6.0\dots 7.0\text{ LPM}$ before equilibrium is reached.
3. **Thermal Inertia vs. Controller Timebases:**
   * The physical time constant of the water loop is $\tau \approx 5.5\text{ minutes}$ ($330\text{s}$).
   * Any supervisory heater modulation loop MUST operate on a slow timebase ($\ge 30\text{s}$) with small steps ($\pm 3\dots 5\%$) to avoid ringing or over-controlling against this thermal mass.
