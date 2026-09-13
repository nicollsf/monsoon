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
  CALIBF_PREPARE,         // Prime circuit at 5.0 delivery / 6.0 recovery
  CALIBF_RUN,             // Multi-point delivery & delta flow sweep
  CALIBF_DONE
};

void loop_calib(void);
void loop_autocalibf(void);
void calib_archive_log(void);