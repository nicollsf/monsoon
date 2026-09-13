#pragma once

enum autocalibp_states {
  CALIBP_NONE = 0,
  CALIBP_FILL,            // Fill tank to full
  CALIBP_PREPARE,         // Prime and stabilise circuit
  CALIBP_CALIB_DELIVERY,  // Calibrate delivery pump
  CALIBP_SCAVENGE_PREP,   // Flood pan (scavenge off, delivery 90% until tank level low)
  CALIBP_CALIB_SCAVENGE,  // Calibrate scavenge pump (with dynamic delivery)
  CALIBP_DONE
};

enum autocalibf_states {
  CALIBF_NONE = 0,
  CALIBF_INIT_FILL,       // Fill tank to top float with mains inlet if needed
  CALIBF_DRAIN_DELIVERY,  // Drain tank from Top Float to Bottom Float using Delivery Pump
  CALIBF_FILL_RECOVERY,   // Fill tank from Bottom Float to Top Float using Recovery Pump
  CALIBF_CALCULATE,       // Compute K_rec ratio and display results
  CALIBF_DONE
};

void loop_calib(void);
void loop_autocalibf(void);
void calib_archive_log(void);