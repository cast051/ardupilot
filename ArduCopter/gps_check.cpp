#include "Copter.h"


/**
 *
 * Detects failures of the gps status for the RTK system, trigger an to the pilot and take action.
 *
 */
#ifndef GPS_CHECK_ITERATION_MAX
 #define GPS_CHECK_ITERATION_MAX 10 // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

static struct {
	uint8_t fail_count;
	uint8_t bad_variance : 1;
} gps_check_state;

void Copter::gps_check_armed()
{

	if (gps_overthrottle()) {
		if (!gps_check_state.bad_variance) {
			gps_check_state.fail_count++;
			if (gps_check_state.fail_count > GPS_CHECK_ITERATION_MAX)
			{
				gcs_send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch detected");
				gps_check_state.bad_variance = true;
				if (control_mode == AUTO) {
					if (set_mode(LOITER, MODE_REASON_GPS_GLITCH)) {
						Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_FAILSAFE_OCCURRED);
						gcs_send_text(MAV_SEVERITY_CRITICAL,"GPS Glitch Occurred Process done");
						failsafe.gps = true;
					}
				}
			}
		}
	}
}
bool Copter::gps_overthrottle()
{
	if (!motors.armed()) {
		return false;
	}
	if (gps.status(1) < AP_GPS::GPS_OK_FIX_3D_RTK && gps.status(0) >= AP_GPS::GPS_OK_FIX_3D && g.gps_fail_check) {
		return true;
	} else {
		return false;
	}
}