//// Auto wash
//enum autowash_states {
//  WASH_NONE = 0,
//  WASH_CYCLE
//};
//String autowash_statestrs[] = {"NONE", "CYCLE", "WTF"};
//void loop_autowash(void)
//{
//  if( auto_state!=STATE_WASH ) return;
//
//  // Code to run on transition into main state
//  if( auto_laststate!=auto_state ) {
//    btLog("Entering state " + auto_statestrs[auto_state]);
//    auto_nextstate = STATE_SHUT;
//    auto_substate = auto_lastsubstate = 0;
//    auto_substatestr = autowash_statestrs[auto_substate];
//    pins_off();  // all off
//    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
//    auto_laststate = auto_state;  
//    auto_statestime = millis();
//  }
//
//  // Code to run on transition into substate
//  if( auto_substate!=auto_lastsubstate ) {
//    auto_lastsubstate = auto_substate;
//    auto_substatestr = autowash_statestrs[auto_substate];
//    auto_substatestime = millis();
//    pins_off();
//
//    switch( auto_substate ) {
//      case WASH_NONE:
//        break;
//
//      case WASH_CYCLE:
//        digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[3], RELAYON);
//        htrs_en[0] = htrs_en[1] = 1;
//        break;
//    }
//        
//    btLog("Entered substate " + auto_substatestr);
//  }
//
//
//
//
//  
//
//  return;
//}    
//
//// Auto shut
//enum autoshut_states {
//  SHUT_NONE = 0,
//  SHUT_DRAIN,   // empty working tank to drain
//  SHUT_RINSE,  // rinse working tank to drain
//};
//String autoshut_statestrs[] = {"NONE", "DRAIN", "RINSE", "WTF"};
//int autoshut_rinseperiod = 2500;
//int autoshut_numrinse;
//void loop_autoshut(void)
//{
//  if( auto_state!=STATE_SHUT ) return;
//
//  // Code to run on transition into main state
//  if( auto_laststate!=auto_state ) {
//    btLog("Entering state " + auto_statestrs[auto_state]);
//    auto_nextstate = STATE_OFF;
//    auto_substate = auto_lastsubstate = 0;
//    auto_substatestr = autoshut_statestrs[auto_substate];
//    autoshut_numrinse = 0;
//    pins_off();  // all off
//    htrs_en[0] = htrs_en[1] = htrs_en[2] = htrs_en[3] = 0;
//    auto_laststate = auto_state;
//    auto_statestime = millis();
//  }
//
//  // Code to run on transition into substate
//  if( auto_substate!=auto_lastsubstate ) {
//    auto_lastsubstate = auto_substate;
//    auto_substatestr = autoshut_statestrs[auto_substate];
//    auto_substatestime = millis();
//    pins_off();
//
//    switch( auto_substate ) {
//      case SHUT_NONE:
//        break;
//
//      case SHUT_DRAIN:
//        digitalWrite(RPINS1[0], RELAYON);  digitalWrite(RPINS1[3], RELAYON);
//        digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
//        break;
//
//      case SHUT_RINSE:
//        digitalWrite(RPINS1[1], RELAYON);  digitalWrite(RPINS1[2], RELAYON);
//        digitalWrite(RPINS0[4], RELAYON);  digitalWrite(RPINS0[5], RELAYON);
//        autoshut_numrinse += 1;
//        break;
//    }
//        
//    btLog("Entered substate " + auto_substatestr);
//  }
//
//  // Automatic transitions between substates
//  switch( auto_substate ) {
//    case SHUT_NONE:
//      auto_substate = SHUT_DRAIN;
//      break;
//      
//    case SHUT_DRAIN:
//      btLog("Checking for no primary flow");
//      if( flow_lasts1==0 && millis()-flow_lastch1>=2000 ) auto_substate = SHUT_RINSE;
//      break;
//
//    case SHUT_RINSE:
//      btLog("Checking for rinse complete");
//      if( millis()-auto_substatestime>=autoshut_rinseperiod ) {
//        if( autoshut_numrinse>=2 ) auto_substate = SHUT_DRAIN;
//        else auto_substate = auto_nextstate;
//      }
//      break;
//  }
//
//  return;
//}

