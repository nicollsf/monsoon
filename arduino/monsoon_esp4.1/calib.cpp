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
  CALIBP_STABILISE,
  CALIBP_PUMPSPEED,
  CALIBP_DONE
};

std::vector<float> calib_pumpperc = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
std::vector<float> calib_dpumpmap, calib_rpumpmap;

const char *autocalibp_statestrs[] = {"NONE", "STABILISE", "PUMPSPEED", "DONE", "WTF"};
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
      }
      Serial.println("Switching substate to CALIB_STABILISE");
      auto_switchsubstate(CALIBP_STABILISE);
      break;

    case CALIBP_STABILISE:
      if( just_entered ) {
        setrelay_en(RPDELIVER, RON);  setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON); 
        setpump_perc(RPUMPR, 100);  setpump_perc(RPUMPD, 100);
      }
      if( millis()-auto_substatestime<15000 ) {
        break;
      }

      Serial.println("Switching substate to CALIBP_PUMPSPEED");
      calib_cind = -1;
      auto_switchsubstate(CALIBP_PUMPSPEED); 
      break;
      
    case CALIBP_PUMPSPEED:
      if( just_entered ) {
        setrelay_en(RPDELIVER, RON);  setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON);  
      }

      if( calib_cind<0 ) {
        calib_rpumpmap.clear();
        calib_dpumpmap.clear();
        calib_cind = 0;
        Serial.println("Setting pumps to " + String(calib_pumpperc[calib_cind]));
        setpump_perc(RPUMPR, calib_pumpperc[calib_cind]);
        setpump_perc(RPUMPD, calib_pumpperc[calib_cind]);
        calib_lastsettime = millis();
        break;
      }

      if( millis()-calib_lastsettime<10000 ) break;

      calib_rpumpmap.push_back(flow_lpm0);
      calib_dpumpmap.push_back(flow_lpm1);

      if( calib_cind < (int)calib_pumpperc.size()-1 ) {
        calib_cind++;
        Serial.println("Setting pumps to " + String(calib_pumpperc[calib_cind]));
        setpump_perc(RPUMPR, calib_pumpperc[calib_cind]);
        setpump_perc(RPUMPD, calib_pumpperc[calib_cind]);
        calib_lastsettime = millis();
        break;
      }
        
      modelr = pumpcalib_fit(calib_pumpperc, calib_rpumpmap);
      modeld = pumpcalib_fit(calib_pumpperc, calib_dpumpmap);
      pumpcalib_savemodels(); 

      auto_switchsubstate(CALIBP_DONE); 
      break;

    case CALIBP_DONE:
      if( just_entered ) {
        auto_woverflowstopenable = 0;
      }
      //htrs_disable = 0;
      auto_switchstate(STATE_CALIBT);
      break;
  }
}


// Auto calibrate temperature
enum autocalibt_states {
  CALIBT_NONE = 0,
  CALIBT_OVERSCAVENGE,
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

const char *autocalibt_statestrs[] = {"NONE", "OVERSCAVENGE", "TEMPRESP", "DONE", "WTF"};
void loop_autocalibt(void)
{
  static int last_substate = -1;
  float target_lpm_low, target_lpm_high, current_target_lpm;

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
      }

      // Wait for pan to clear and tank to fill
      if( level_high1 && (millis() - auto_substatestime > 15000) ) {
        Serial.println("Switching substate to CALIBT_TEMPRESP");
        calib_cind = -1;
        auto_switchsubstate(CALIBT_TEMPRESP); 
      }
      break;

    case CALIBT_TEMPRESP:
      static int step_repeat = 0;
      static unsigned long last_sample_time = 0;
      static bool is_high_step = false;

      target_lpm_low = 0.333f * modeld.maxflow;
      target_lpm_high = 0.666f * modeld.maxflow;

      if( just_entered ) {
        auto_woverflowstopenable = 1;
        setrelay_en(RPDELIVER, RON);  setpump_en(RPUMPR, RON);  setpump_en(RPUMPD, RON);  

        temp_response_log.clear();
        temp_response_log.reserve(1000); 
        calib_lastsettime = millis();
        last_sample_time = millis();
        step_repeat = 0;
        is_high_step = false;
        Serial.printf("Starting Temp Response. Targets: %.2f/%.2f LPM\n", target_lpm_low, target_lpm_high);
        
        // Clear NVRAM log
        Preferences p;
        p.begin("calibt", false);
        p.remove("log");
        p.end();

        // Overscavenge using recovery pump, rely on auto_woverflowstopenable for limits
        setpump_perc(RPUMPR, 100);
        setpump_lpm(RPUMPD, target_lpm_low);
      }

      current_target_lpm = is_high_step ? target_lpm_high : target_lpm_low;      

      if( !level_high0 ) {
        btLog("ERROR: Tank level low during Temp Calib. Aborting.");
        // Save partial data before aborting!
        Preferences prefs;
        prefs.begin("calibt", false);
        prefs.putBytes("log", temp_response_log.data(), temp_response_log.size() * sizeof(TempLogPoint));
        prefs.end();
        auto_switchstate(STATE_CALIBDUMP);
        return;
      }

      if( millis()-auto_substatestime < 20000 ) {
        return;
      }

      if( millis()-last_sample_time >= 1000 ) {
        last_sample_time = millis();
        TempLogPoint p;
        p.time_sec = (uint16_t)((millis() - auto_substatestime) / 1000);
        p.temp_c = temp1; // Use the fast NTC sensor the PID actually looks at
        p.flow0 = flow_lpm0; // Record actual measured flows
        p.flow1 = flow_lpm1;
        p.target_lpm_x10 = (uint8_t)(current_target_lpm * 10);
        temp_response_log.push_back(p);

        // Save to NVRAM every 10 seconds
        if (temp_response_log.size() % 10 == 0) {
           Preferences prefs;
           prefs.begin("calibt", false);
           prefs.putBytes("log", temp_response_log.data(), temp_response_log.size() * sizeof(TempLogPoint));
           prefs.end();
        }
      }

      if( millis()-calib_lastsettime >= 120000 ) {
        calib_lastsettime = millis();
        is_high_step = !is_high_step;
        if (!is_high_step) step_repeat++; 
        setpump_lpm(RPUMPD, is_high_step ? target_lpm_high : target_lpm_low);
        btLog("Temp Step: Delivery Flow now " + String(is_high_step ? target_lpm_high : target_lpm_low) + " LPM");
      }

      if( step_repeat >= 4 ) {
        // Final save
        Preferences prefs;
        prefs.begin("calibt", false);
        prefs.putBytes("log", temp_response_log.data(), temp_response_log.size() * sizeof(TempLogPoint));
        prefs.end();

        btLog("Temp calibration recording completed and archived.");
        temp_response_log.clear(); 
        auto_switchsubstate(CALIBT_DONE);
      }
      break;

    case CALIBT_DONE:
      if( just_entered ) {
        auto_woverflowstopenable = 0;
        //htrs_disable = 0;
        temp_controlmode = TCNONE;
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