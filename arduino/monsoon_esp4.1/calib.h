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

void loop_calib(void);
void calib_archive_log(void);