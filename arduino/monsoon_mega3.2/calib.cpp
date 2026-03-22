#include "auto.h"
//#include "control.h"
#include "system.h"
#include "gui.h"
#include "sensors.h"


// ----------------------------------------------------------------------
//   Main calibration functionality
// ----------------------------------------------------------------------

const int hstnsamp = 240;  // heater step test number of samples
const float hstdmsec = 5000;  // heater step test sampling period
float hstsampf[hstnsamp];  // heater step test sample values
unsigned long hststarts;  // heater step test start time
int htlnexti = 0;  // heater step test next sample index
  
// Auto calibration
enum calib_states {
  CALIB_NONE = 0,
  CALIB_SETUPFILL,
  CALIB_SETUPWARM,
  CALIB_SETUPWARMC,
  CALIB_SETUPWARMD,
  CALIB_START,
  CALIB_HEATERONSTEP,
  CALIB_HEATEROFFSTEP,
  CALIB_DONE
};
const char *autocalib_statestrs[] = {"NONE", "SETUPFILL", "SETUPWARM", "SETUPWARMC", "SETUPWARMD", "START", "HEATERONSTEP", "HEATEROFFSTEP", "DONE", "WTF"};
//void loop_calib(void)
//{
//  if( auto_state!=STATE_CALIB ) return;
// 
//  // Code to run on transition into substate
//  if( auto_substatechange ) {
//    triac_Pp = 90;
//    
//    switch( auto_substate ) {
//      case CALIB_SETUPFILL:
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        break;
//
//      case CALIB_SETUPWARM:
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        break;
//
//      case CALIB_SETUPWARMC:
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        RPINS1_EN[2] = 1;  //RPINS1_EN[0] = 1;  
//        break;
//
//      case CALIB_SETUPWARMD:
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        RPINS0_EN[5] = 1;  RPINS1_EN[2] = 1;  
//        break;
//
//      case CALIB_START:
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        break;
//        
//      case CALIB_HEATERONSTEP:
//        triac_Pp = 90;
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        RPINS0_EN[0] = RPINS0_EN[1] = 1;
//        break;
//        
//       case CALIB_HEATEROFFSTEP:
//        RPINS1_EN[2] = 1;
//        RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
//        auto_topupenable = 1;
//        break;
//        
//      case CALIB_DONE:
//        btLog("Entered CALIB_DONE");
//        Serial.println("Calibration complete:");
//        Serial.print("hstsampf = [ ");
//        for( int i=0; i<hstnsamp; i++ ) Serial.print(String(hstsampf[i]) + " ");
//        Serial.println("]");
//        break;
//    }
//  }
//
//  // Automatic transitions between substates
//  switch( auto_substate ) {
//    case CALIB_NONE:
//      auto_switchsubstate(CALIB_SETUPFILL);
//      break;
//
//    case CALIB_SETUPFILL:
//      if( millis()-auto_substatestime>=10000 && millis()-auto_topuplastlowtime>=10000 ) {
//        btLog("Switching from CALIB_SETUPFILL to CALIB_SETUPWARM");
//        auto_switchsubstate(CALIB_SETUPWARM);
//      }
//      break;
//
//    case CALIB_SETUPWARM:
//      if( temp0>temp_setpoint-5 ) {
//        btLog("Switching from CALIB_SETUPWARM to CALIB_START because " + String(temp0) + ">" + String(temp_setpoint-5));
//        auto_switchsubstate(CALIB_START);
//      }
//      if( auto_substate==CALIB_SETUPWARM && millis()-auto_substatestime>=autowarm_cycleinterval ) auto_switchsubstate(CALIB_SETUPWARMC);
//      break;
//
//    case CALIB_SETUPWARMC:
//      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(CALIB_SETUPWARMD);
//      break;
//
//    case CALIB_SETUPWARMD:
//      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(CALIB_SETUPWARM);
//      break;
//
//    case CALIB_START:
//      if( millis()-auto_substatestime>=30000 && millis()-auto_topuplastlowtime>=30000 ) {  // stabilise
//        btLog("Switching from CALIB_SETUP to CALIB_HEATERONSTEP");
//        hststarts = millis();  htlnexti = 0;  // start capture
//        auto_switchsubstate(CALIB_HEATERONSTEP);
//      }
//      break;
//
//    case CALIB_HEATERONSTEP:  
//      btLog("In CALIB_HEATERONSTEP(temp=" + String(temp0) + "): htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
//      if( htlnexti>=hstnsamp ) {  auto_switchsubstate(CALIB_DONE);  return; }
//      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
//      if( temp0>temp_setpoint+5 ) auto_switchsubstate(CALIB_HEATEROFFSTEP);
//      break;
//
//    case CALIB_HEATEROFFSTEP:  
//      btLog("In CALIB_HEATEROFFSTEP:  htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
//      if( htlnexti>=hstnsamp ) { auto_switchsubstate(CALIB_DONE);  return; }
//      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
//      if( temp0<temp_setpoint-5 ) auto_switchsubstate(CALIB_HEATERONSTEP);
//      break;
//  }
//
//  return;
//}


void loop_calib(void)
{
  if( auto_state!=STATE_CALIB ) return;
 
  switch( auto_substate ) {
    
    case CALIB_NONE:
      auto_switchsubstate(CALIB_SETUPFILL);
      break;

    case CALIB_SETUPFILL:
      RPINS1_EN[2] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      //auto_wtopupenable = 1;
        
//      if( millis()-auto_substatestime>=10000 && millis()-auto_wtopuplastlowtime>=10000 ) {
//        btLog("Switching from CALIB_SETUPFILL to CALIB_SETUPWARM");
//        auto_switchsubstate(CALIB_SETUPWARM);
//      }
      break;

    case CALIB_SETUPWARM:
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
   
      if( temp0>temp_setpoint-5 ) {
        btLog("Switching from CALIB_SETUPWARM to CALIB_START because " + String(temp0) + ">" + String(temp_setpoint-5));
        auto_switchsubstate(CALIB_START);
      }
//      if( auto_substate==CALIB_SETUPWARM && millis()-auto_substatestime>=autowarm_cycleinterval ) auto_switchsubstate(CALIB_SETUPWARMC);
      break;

    case CALIB_SETUPWARMC:
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      RPINS1_EN[2] = 1;  //RPINS1_EN[0] = 1;  
      
//      if( millis()-auto_substatestime>=autowarm_cycleperiod ) auto_switchsubstate(CALIB_SETUPWARMD);
      break;

    case CALIB_SETUPWARMD:
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
      RPINS0_EN[5] = 1;  RPINS1_EN[2] = 1;  
      
//      if( millis()-auto_substatestime>=autowarm_drainperiod ) auto_switchsubstate(CALIB_SETUPWARM);
      break;

    case CALIB_START:
      RPINS1_EN[2] = 1;  
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_wtopupenable = 1;
      
//      if( millis()-auto_substatestime>=30000 && millis()-auto_wtopuplastlowtime>=30000 ) {  // stabilise
//        btLog("Switching from CALIB_SETUP to CALIB_HEATERONSTEP");
//        hststarts = millis();  htlnexti = 0;  // start capture
//        auto_switchsubstate(CALIB_HEATERONSTEP);
//      }
      break;

    case CALIB_HEATERONSTEP:  
//      triac_Pp = 90;
      RPINS1_EN[2] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_wtopupenable = 1;
      RPINS0_EN[0] = RPINS0_EN[1] = 1;
              
      btLog("In CALIB_HEATERONSTEP(temp=" + String(temp0) + "): htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
      if( htlnexti>=hstnsamp ) {  auto_switchsubstate(CALIB_DONE);  return; }
      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
      if( temp0>temp_setpoint+5 ) auto_switchsubstate(CALIB_HEATEROFFSTEP);
      break;

    case CALIB_HEATEROFFSTEP:  
      RPINS1_EN[2] = 1;
      RPINS0_EN[4] = 1;  RPINS0_EN[5] = 1;
      auto_wtopupenable = 1;
          
      btLog("In CALIB_HEATEROFFSTEP:  htlnexti=" + String(htlnexti) + "/" + String(hstnsamp));
      if( htlnexti>=hstnsamp ) { auto_switchsubstate(CALIB_DONE);  return; }
      if( millis()-hststarts>=hstdmsec*htlnexti ) hstsampf[htlnexti++] = temp0;
      if( temp0<temp_setpoint-5 ) auto_switchsubstate(CALIB_HEATERONSTEP);
      break;

    case CALIB_DONE:
//      if( auto_substatechange ) {
//        btLog("Calibration completed:");
//        Serial.print("hstsampf = [ ");
//        for( int i=0; i<hstnsamp; i++ ) Serial.print(String(hstsampf[i]) + " ");
//        Serial.println("]");
//      }
      break;

      
  }

  return;
}
