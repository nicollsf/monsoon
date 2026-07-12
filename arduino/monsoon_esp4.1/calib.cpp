#include <Arduino.h>
#include <vector>
#include <Preferences.h>
#include "calib.h"
#include "auto.h"
#include "system.h"
#include "sensors.h"
#include "control.h"
#include "gui.h"

// ----------------------------------------------------------------------
//   Calibrations
// ----------------------------------------------------------------------

int calib_cind;
unsigned int calib_lastsettime;
unsigned int calib_laststagetime;


// Auto calibrate pumps
enum autocalibp_states {
  CALIBP_NONE = 0,
  CALIBP_FILL,            // Fill tank to full
  CALIBP_PREPARE,         // Prime and stabilise circuit
  CALIBP_CALIB_DELIVERY,  // Calibrate delivery pump
  CALIBP_SCAVENGE_PREP,   // Flood pan (scavenge off, delivery 90% until tank level low)
  CALIBP_CALIB_SCAVENGE,  // Calibrate scavenge pump (with dynamic delivery)
  CALIBP_DONE
};

std::vector<float> calib_pumpperc = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
std::vector<float> calib_dpumpmap, calib_rpumpmap;

const char *autocalibp_statestrs[] = {"NONE", "FILL", "PREPARE", "CALIB_DEL", "SCAV_PREP", "CALIB_SCAV", "DONE", "WTF"};
std::vector<float> pdpercv, trespv;
void loop_autocalibp(void)
{
  static int last_substate = -1;
  float flow_low, flow_high, current_target_lpm;

  if( auto_state!=STATE_CALIBP ) return;
  auto_substatestrs = autocalibp_statestrs;
  
  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  switch( auto_substate ) {
    case CALIBP_NONE:
      if( just_entered ) {
        auto_wtopupenable = 0;
        htrs_enable = 0;
        temp_controlmode = TCOFF;
        btLog("Entering CALIBP_NONE");
        calib_cind = -1;
        calib_dpumpmap.clear();
        calib_rpumpmap.clear();
      }
      Serial.println("Switching substate to CALIBP_FILL");
      auto_switchsubstate(CALIBP_FILL);
      break;

    case CALIBP_FILL:
      if( just_entered ) {
        btLog("CALIBP: Filling tank from mains inlet.");
        setrelay_en(RPINLET, RON);
      }

      if( tank_full ) {
        btLog("CALIBP: Tank is full. Moving to prepare/prime stage.");
        setrelay_en(RPINLET, ROFF);
        auto_switchsubstate(CALIBP_PREPARE);
      }
      break;

    case CALIBP_PREPARE:
      if( just_entered ) {
        btLog("CALIBP: Priming/stabilising circuit. SCAVENGE 100%, DELIVERY 50%, top-up ON.");
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPR, RON);
        setpump_en(RPUMPD, RON);
        setpump_perc(RPUMPR, 100);
        setpump_perc(RPUMPD, 50); // delivery ~50%
        auto_wtopupenable = 1;
      }

      // Wait for tank to be full and flow to be stable for 20s
      if( tank_full && (millis() - flow_lastlt1 > 20000) ) {
        btLog("CALIBP: Flow stable. Starting delivery pump calibration.");
        calib_cind = -1;
        auto_switchsubstate(CALIBP_CALIB_DELIVERY);
      }
      break;

    case CALIBP_CALIB_DELIVERY:
      if( just_entered ) {
        btLog("CALIBP: Starting delivery pump calibration step series.");
        calib_dpumpmap.clear();
        calib_cind = 0;
        setrelay_en(RPDELIVER, RON);
        setpump_en(RPUMPR, RON);
        setpump_perc(RPUMPR, 100); // Scavenge full to clear pan
        setpump_en(RPUMPD, RON);
        setpump_perc(RPUMPD, calib_pumpperc[calib_cind]);
        auto_wtopupenable = 1; // Keep tank full during delivery calibration
        calib_lastsettime = millis();
      }

      // Safeguard check
      if( tank_empty ) {
        btLog("ERROR: Tank level low during Delivery Calib. Aborting.");
        pumpsen_reset();
        auto_switchstate(STATE_OFF);
        return;
      }

      if( millis() - calib_lastsettime >= 10000 ) {
        float measured_flow = flow_lpm0;
        calib_dpumpmap.push_back(measured_flow);
        btLog("CALIBP: Delivery " + String(calib_pumpperc[calib_cind], 0) + "% -> Flow: " + String(measured_flow, 2) + " LPM");

        if( calib_cind < (int)calib_pumpperc.size() - 1 ) {
          calib_cind++;
          setpump_perc(RPUMPD, calib_pumpperc[calib_cind]);
          calib_lastsettime = millis();
        } else {
          // Delivery calibration complete
          std::vector<float> dpump_pwms;
          std::vector<float> dpump_flows;
          for (size_t i = 0; i < calib_pumpperc.size(); i++) {
            if (calib_pumpperc[i] <= 90.0f) {
              dpump_pwms.push_back(calib_pumpperc[i]);
              dpump_flows.push_back(calib_dpumpmap[i]);
            }
          }
          modeld = pumpcalib_fit(dpump_pwms, dpump_flows);
          pumpcalib_savemodels();
          btLog("CALIBP: Delivery pump calibration finished. Model fitted and saved.");
          auto_switchsubstate(CALIBP_SCAVENGE_PREP);
        }
      }
      break;

    case CALIBP_SCAVENGE_PREP:
      if( just_entered ) {
        btLog("CALIBP: Flooding pan. Scavenge off, top-up off, Delivery 90% until level low.");
        setpump_en(RPUMPR, ROFF);
        auto_wtopupenable = 0;
        setrelay_en(RPINLET, ROFF);
        setrelay_en(RPDELIVER, RON);
        setpump_perc(RPUMPD, 90);
        setpump_en(RPUMPD, RON);
      }

      // Wait until bottom level sensor hits low (which means pan is flooded/full)
      if( tank_empty ) {
        btLog("CALIBP: Tank level hit low (pan is full). Starting scavenge calibration steps.");
        calib_cind = -1;
        auto_switchsubstate(CALIBP_CALIB_SCAVENGE);
      }
      break;

    case CALIBP_CALIB_SCAVENGE:
      if( just_entered ) {
        btLog("CALIBP: Starting scavenge pump calibration step series with dynamic delivery.");
        calib_rpumpmap.clear();
        calib_cind = 0;
        setrelay_en(RPDELIVER, RON);
        setpump_perc(RPUMPR, calib_pumpperc[calib_cind]);
        setpump_en(RPUMPR, RON);
        calib_lastsettime = millis();
      }

      // Dynamic Delivery pump control:
      // drive the DELIVERY pump full (90%) when the tank level is high and have it off when the level is low.
      if( tank_full ) {
        setpump_perc(RPUMPD, 90);
        setpump_en(RPUMPD, RON);
      } else {
        setpump_en(RPUMPD, ROFF);
      }

      if( millis() - calib_lastsettime >= 10000 ) {
        float measured_flow = flow_lpm1;
        calib_rpumpmap.push_back(measured_flow);
        btLog("CALIBP: Scavenge " + String(calib_pumpperc[calib_cind], 0) + "% -> Flow: " + String(measured_flow, 2) + " LPM");

        if( calib_cind < (int)calib_pumpperc.size() - 1 ) {
          calib_cind++;
          setpump_perc(RPUMPR, calib_pumpperc[calib_cind]);
          calib_lastsettime = millis();
        } else {
          // Scavenge calibration finished
          modelr = pumpcalib_fit(calib_pumpperc, calib_rpumpmap);
          pumpcalib_savemodels();
          btLog("CALIBP: Scavenge pump calibration finished. Model fitted and saved.");
          auto_switchsubstate(CALIBP_DONE);
        }
      }
      break;

    case CALIBP_DONE:
      if( just_entered ) {
        auto_woverflowstopenable = 0;
        pumpsen_reset();
      }
      auto_switchstate(STATE_CALIBT);
      break;
  }
}


// Auto calibrate temperature
enum autocalibt_states {
  CALIBT_NONE = 0,
  CALIBT_OVERSCAVENGE,
  CALIBT_PREWARM,
  CALIBT_TEMPRESP,
  CALIBT_DONE
};

struct TempLogPoint {
  uint16_t time_sec;
  float temp_c;
  float flow0;
  float flow1;
  uint8_t target_lpm_x10;
} __attribute__((packed));
std::vector<TempLogPoint> temp_response_log;

const char *autocalibt_statestrs[] = {"NONE", "OVERSCAVENGE", "PREWARM", "TEMPRESP", "DONE", "WTF"};
void loop_autocalibt(void)
{
  static int last_substate = -1;
  float target_lpm_low, target_lpm_high, current_target_lpm;

  const float CALIB_TARGET_TEMP = temp_setpoint; // The desired center-point for calibration based on user preference.
  const float CALIB_DEVIATION = 3.0f;    // The +/- range to explore around the target.

  if( auto_state!=STATE_CALIBT ) return;
  auto_substatestrs = autocalibt_statestrs;

  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;
  
  switch( auto_substate ) {
    case CALIBT_NONE:
      if( just_entered ) {
        auto_wtopupenable = 0;
        htrs_enable = 1;
        temp_controlmode = TCON;
        auto_woverflowstopenable = 1;
        btLog("Entering CALIBT_NONE");
        calib_cind = -1;
      }
      Serial.println("Switching substate to CALIBT_OVERSCAVENGE");
      auto_switchsubstate(CALIBT_OVERSCAVENGE);
      break;

    case CALIBT_OVERSCAVENGE:
      target_lpm_high = 0.666f * modeld.maxflow;

      if( just_entered ) {
        setrelay_en(RPDELIVER, RON);  
        setpump_en(RPUMPR, RON);  
        setpump_en(RPUMPD, RON); 
        
        // Phase 1: Over-scavenge to ensure shower base is empty (minimal charge)
        setpump_perc(RPUMPR, 100);  
        setpump_lpm(RPUMPD, target_lpm_high);

        // Enable occasional top-up to prevent running dry, with a longer interval
        auto_wtopup_interval = 20000; // 20 seconds
        auto_wtopupenable = 1;
      }

      // Wait for pan to clear and tank to fill
      if( tank_full && (millis() - auto_substatestime > 15000) ) {
        Serial.println("Switching substate to CALIBT_PREWARM");
        calib_cind = -1;
        auto_switchsubstate(CALIBT_PREWARM); 
      }
      break;

    case CALIBT_PREWARM: {
      const float PREWARM_TARGET = CALIB_TARGET_TEMP; 
      const float TEMP_UPPER = CALIB_TARGET_TEMP + CALIB_DEVIATION; // Upper limit of investigation range
      float base_lpm = 0.10f * modeld.maxflow; // Low flow to minimise circuit heat loss
      float pulse_lpm = 0.70f * modeld.maxflow; // Periodic pulse to guarantee mixing
      unsigned long cycle_time = (millis() - auto_substatestime) % 30000; // 30s cycle
      const unsigned long PREWARM_MAX_DURATION = 600000; // 10 minutes

      if( just_entered ) {
        auto_wtopupenable = 0; // Disable top-up from previous state
        auto_wtopup_interval = 10000; // Reset to default

        setrelay_en(RPDELIVER, RON);  
        setpump_en(RPUMPR, RON);  
        setpump_en(RPUMPD, RON); 
        setpump_perc(RPUMPR, 100);
        btLog("Starting PREWARM to " + String(PREWARM_TARGET) + "C.");
      }

      // Pulse flow for 5 seconds every 30 seconds for good mixing
      if( cycle_time < 5000 ) {
        setpump_lpm(RPUMPD, pulse_lpm);
      } else {
        setpump_lpm(RPUMPD, base_lpm);
      }

      if( temp1 >= TEMP_UPPER ) {
        btLog("Tank above investigation range (" + String(temp1) + "C). Switching to TEMPRESP.");
        auto_switchsubstate(CALIBT_TEMPRESP);
      } else if( temp1 >= PREWARM_TARGET ) {
        btLog("Prewarm target reached (" + String(temp1) + "C). Switching to TEMPRESP.");
        auto_switchsubstate(CALIBT_TEMPRESP);
      } else if( millis() - auto_substatestime >= PREWARM_MAX_DURATION ) {
        btLog("ERROR: Prewarm timeout (10 min). Could not reach " + String(PREWARM_TARGET) + "C. Aborting.");
        auto_switchsubstate(CALIBT_DONE);
      }
      break;
    }

    case CALIBT_TEMPRESP: {
      static unsigned long last_sample_time = 0;
      static unsigned long last_step_change_time = 0;
      static bool is_high_step = false;
      const float TEMP_LOWER = CALIB_TARGET_TEMP - CALIB_DEVIATION;
      const float TEMP_UPPER = CALIB_TARGET_TEMP + CALIB_DEVIATION;
      const float TEMP_MAX = 58.0; // Graceful exit before safety limits trigger
      const unsigned long CALIB_MAX_DURATION = 1200000; // 20 minutes

      target_lpm_low = 0.333f * modeld.maxflow;
      target_lpm_high = 0.666f * modeld.maxflow;

      if( just_entered ) {
        auto_wtopupenable = 0; // Explicitly disable auto-topup to ensure manual inlet control.
        auto_woverflowstopenable = 1;
        setrelay_en(RPDELIVER, RON);  setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON);  

        temp_response_log.clear();
        temp_response_log.reserve(1000); 
        calib_lastsettime = millis();
        last_sample_time = millis();
        last_step_change_time = millis();
        is_high_step = (temp1 >= TEMP_UPPER); // Set initial state based on current temp
        Serial.printf("Starting Temp Response. Targets: %.2f/%.2f LPM\n", target_lpm_low, target_lpm_high);
        
        // Clear NVRAM log
        Preferences p;
        p.begin("calibt", false);
        p.clear();
        p.end();

        // Overscavenge using recovery pump, rely on auto_woverflowstopenable for limits
        setpump_perc(RPUMPR, 100);
        setpump_lpm(RPUMPD, is_high_step ? target_lpm_high : target_lpm_low);
      }

      current_target_lpm = is_high_step ? target_lpm_high : target_lpm_low;      

      if( tank_empty ) {
        btLog("ERROR: Tank level low during Temp Calib. Aborting.");
        pumpsen_reset();
        calib_archive_log();
        auto_switchstate(STATE_CALIBDUMP); // Go directly to dump
        return;
      }

      if( millis()-auto_substatestime < 20000 ) {
        return;
      }

      if( millis()-last_sample_time >= 5000 ) {
        last_sample_time = millis();
        TempLogPoint p;
        p.time_sec = (uint16_t)((millis() - auto_substatestime) / 1000);
        p.temp_c = temp1; // Use the fast NTC sensor the PID actually looks at
        p.flow0 = flow_lpm0; // Record actual measured flows
        p.flow1 = flow_lpm1;
        p.target_lpm_x10 = (uint8_t)(current_target_lpm * 10);
        temp_response_log.push_back(p);
      }

      // Determine if the temperature has hit an asymptote (stabilised)
      bool stable = false;
      // Must be in the current step for at least 90 seconds to avoid falsely triggering on the turnaround peak
      if (millis() - last_step_change_time >= 90000 && temp_response_log.size() >= 12) {
        // Compare the current temperature with the temperature recorded 12 samples (60 seconds) ago at 5s intervals
        float temp_60s_ago = temp_response_log[temp_response_log.size() - 12].temp_c;
        if (abs(temp1 - temp_60s_ago) <= 0.2) {
          stable = true;
        }
      }

      // Switching logic based on temperature thresholds OR stabilisation
      if (!is_high_step && (temp1 >= TEMP_UPPER || stable)) {
        is_high_step = true;
        setpump_lpm(RPUMPD, target_lpm_high);
        last_step_change_time = millis();
        
        if (stable) btLog("Heating stabilised at " + String(temp1) + "C. Flow now HIGH (" + String(target_lpm_high) + " LPM)");
        else btLog("Temp crossed upper boundary (" + String(TEMP_UPPER) + "C). Flow now HIGH (" + String(target_lpm_high) + " LPM)");
      } else if (is_high_step && (temp1 <= TEMP_LOWER || stable)) {
        is_high_step = false;
        setpump_lpm(RPUMPD, target_lpm_low);
        last_step_change_time = millis();
        
        if (stable) btLog("Cooling stabilised at " + String(temp1) + "C. Flow now LOW (" + String(target_lpm_low) + " LPM)");
        else btLog("Temp crossed lower boundary (" + String(TEMP_LOWER) + "C). Flow now LOW (" + String(target_lpm_low) + " LPM)");
      }

      // Exit successfully if we hit the max time limit or the maximum safety temperature
      bool timeout = (millis() - auto_substatestime >= CALIB_MAX_DURATION);
      bool temp_exceeded = (temp1 >= TEMP_MAX);
      
      if( timeout || temp_exceeded ) {
        if (timeout) btLog("Temp calibration 20-minute duration reached.");
        if (temp_exceeded) btLog("Temp calibration max temp limit (" + String(TEMP_MAX) + "C) reached.");
        calib_archive_log();
        auto_switchsubstate(CALIBT_DONE);
        return;
      }
      break;
    }

    case CALIBT_DONE:
      if( just_entered ) {
        auto_woverflowstopenable = 0;
        //htrs_disable = 0;
        temp_controlmode = TCNONE;
        pumpsen_reset();
      }
      auto_switchstate(STATE_CALIBDUMP); // Automatically dump CSV on success
      break;
  }
}


// Auto calibrate dump
enum autocalibdump_states {
  CALIBDUMP_NONE = 0,
  CALIBDUMP_DUMP,
  CALIBDUMP_DONE
};
const char *autocalibdump_statestrs[] = {"NONE", "DUMP", "DONE", "WTF"};
void loop_autocalibdump(void)
{
  static int last_substate = -1;
  if( auto_state!=STATE_CALIBDUMP ) return;
  auto_substatestrs = autocalibdump_statestrs;
  
  bool just_entered = (last_substate != auto_substate);
  last_substate = auto_substate;

  switch( auto_substate ) {
    case CALIBDUMP_NONE:
      if( just_entered ) btLog("Entering CALIBDUMP_NONE");
      auto_switchsubstate(CALIBDUMP_DUMP);
      break;

    case CALIBDUMP_DUMP:
      if( just_entered ) {
        btLog("Dumping Calibration Info from NVRAM");
        
        Serial.println("\n--- START STORED PUMP MODELS ---");
        Serial.printf("Model R: offset=%.2f, a=%.4f, b=%.4f, maxflow=%.2f\n", modelr.offset, modelr.a, modelr.b, modelr.maxflow);
        Serial.printf("Model D: offset=%.2f, a=%.4f, b=%.4f, maxflow=%.2f\n", modeld.offset, modeld.a, modeld.b, modeld.maxflow);
        Serial.println("--- END STORED PUMP MODELS ---\n");

        {
          Preferences p;
          p.begin("calibt", true);
          size_t len = p.getBytesLength("log");
          if (len > 0 && (len % sizeof(TempLogPoint) == 0)) {
             uint8_t* buf = new uint8_t[len];
             p.getBytes("log", buf, len);
             int count = len / sizeof(TempLogPoint);
             TempLogPoint* pts = (TempLogPoint*)buf;
             Serial.println("Time_sec, Target_LPM, Temp_C, Flow_LPM0, Flow_LPM1");
             for(int i=0; i<count; i++) {
               Serial.printf("%d, %.1f, %.2f, %.2f, %.2f\n", pts[i].time_sec, pts[i].target_lpm_x10/10.0f, pts[i].temp_c, pts[i].flow0, pts[i].flow1);
             }
             delete[] buf;
          } else {
             Serial.println("No NVRAM temp log found.");
          }
          p.end();
        }

        Serial.println("========================================\n");
      }

      auto_switchsubstate(CALIBDUMP_DONE);
      break;

    case CALIBDUMP_DONE:
      if( just_entered ) auto_switchstate(STATE_OFF);
      break;
  }
}

void loop_calib(void)
{
  loop_autocalibp();
  loop_autocalibt();
  loop_autocalibdump();
}

void calib_archive_log(void) {
    if (temp_response_log.empty()) {
        return; // Nothing to save
    }

    Preferences prefs;
    prefs.begin("calibt", false);
    if (prefs.putBytes("log", temp_response_log.data(), temp_response_log.size() * sizeof(TempLogPoint))) {
        btLog("CALIBT log (" + String(temp_response_log.size()) + " pts) archived to NVRAM.");
    } else {
        btLog("ERROR: Failed to archive CALIBT log to NVRAM.");
    }
    prefs.end();

    temp_response_log.clear(); // Free up memory
}