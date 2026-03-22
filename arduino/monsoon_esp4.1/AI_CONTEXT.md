AI\_CONTEXT.md
==============

Project: Monsoon (ESP32 Recirculating Shower)
---------------------------------------------

**Goal:** Transitioning from Arduino IDE to PlatformIO while maintaining interchangeable compatibility.

1\. Directory & Scoping Rules
-----------------------------

*   **Active Code:** Located strictly in the project root directory (.).
    
*   **Excluded Folders:** Ignore attic/, test/, and temp/ for all coding suggestions. These contain legacy or experimental code that conflicts with the current architecture.
    
*   **Environment:** PlatformIO using the ESP32 v3.0 core.
    

2\. Core Architecture: State vs. Intent
---------------------------------------

The project uses a decoupled "Gatekeeper" model to manage high-power hardware safely:

*   **The Intent Layer (\_en):** Control logic (state machines/thermostats) sets the _ideal_ state using setrelay\_en(pin, state). It does not check safety; it only expresses what the system _wants_ to do.
    
*   **The Physical Layer (loop\_heaters):** The hardware gatekeeper. It evaluates the safety\_veto and synchronizes the physical pins to the intent only if safe.
    
*   **Safety Veto Logic:**
    
    *   htrs\_forcedisable: GUI/User override (Manual Veto).
        
    *   level\_safe: Physical float switch check (digitalRead(LSPINS\[0\]) != LSPINSlv\[0\]).
        
    *   temp\_safe: Thermal cutoff (current\_temp <= MAX\_TEMP\_LIMIT).
        
*   **The Rule:** If safety\_veto is **True**, physical pins MUST be ROFF regardless of the \_en intent.
    

3\. Hardware Constraints
------------------------

*   **Input-Only Pins:** Pins 34, 35, 36, and 39 are input-only (no internal pull-ups, no output capability).
    
*   **Relay Logic:** Mixed Active High/Active Low logic is handled via the RPINS\_ROFF array within the setrelay() wrapper.
    
*   **Switching Guard:** A 3000ms htrs\_blocked lockout is required to prevent relay bouncing/chatter.
    

4\. Coding Style
----------------

*   Maintain .ino file extension in the root for Arduino IDE compatibility.

*   Always include #include at the top of the main file for PlatformIO.

*   Prefer explicit function prototypes to avoid "not declared in scope" errors.


5\. Manual Control & Auto Tuning
-------------------------------

*   **General Principle:** The GUI has manual control of everything.

*   **Auto Functionality:** When using AUTO, allow real-time adjustment to find the "sweet spot".

*   **Implementation:** Set up parameters only when *entering* a state or substate. Do not override manual tuning in the loop unless absolutely necessary.
