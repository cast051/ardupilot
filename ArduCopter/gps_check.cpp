#include "Copter.h"


/**
 *
 * Detects failures of the gps status for the RTK system, trigger an to the pilot and take action.
 *
 */
#ifndef GPS_CHECK_FAIL_ITERATION_MAX
 #define GPS_CHECK_FAIL_ITERATION_MAX 1 // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef GPS_CHECK_BACK_ITERATION_MAX
 #define GPS_CHECK_BACK_ITERATION_MAX 100 // 10 seconds
#endif

static struct {
	uint8_t fail_count;
	uint8_t bad_variance : 1;
	uint8_t back_count;
	uint8_t rtk_gone : 1;
} gps_check_state;

void Copter::gps_check_armed()
{

	if (gps_overthrottle()) {
		if (!gps_check_state.bad_variance) {
			gps_check_state.fail_count++;
			if (gps_check_state.fail_count > GPS_CHECK_FAIL_ITERATION_MAX)
			{
				gcs_send_text(MAV_SEVERITY_CRITICAL,"GPS_CHECK:GPS Glitch detected and Processing");
				gps_check_state.bad_variance = true;
				mission.stop(); //stop mission runing.
				if (control_mode == AUTO) {
					if (auto_mode == Auto_Loiter)
					{
						 wp_nav.init_loiter_target();

					} else {
						auto_brake_start();
					}
					failsafe.gps = true;
					// auto_loiter_start(); //start auto loiter
					Log_Write_Error(ERROR_SUBSYSTEM_GPS, ERROR_CODE_FAILSAFE_OCCURRED);
					//set EK2_ALT_SOURCE as 0, use baro for alt control.
					enum ap_var_type var_type;
					// set parameter
					AP_Param *vp;
					char key[AP_MAX_NAME_SIZE+1] = "EK2_ALT_SOURCE";
					key[AP_MAX_NAME_SIZE] = 0;
					// find existing param so we can get the old value
					vp = AP_Param::find(key, &var_type);
					if (vp == NULL) {
						return;
					}
					// set the value
					vp->set_float(0, var_type);
				}
			}
		}
	} else {
		gps_check_state.fail_count = 0;
	}

	if (gps_check_state.bad_variance && !gps_check_state.rtk_gone) {
		if (gps_backnormal()) {
			gcs_send_text(MAV_SEVERITY_CRITICAL,"GPS_CHECK:RTK back resume mission");
			mission.resume();
			enum ap_var_type var_type;
			// set parameter
			AP_Param *vp;
			char key[AP_MAX_NAME_SIZE+1] = "EK2_ALT_SOURCE";
			key[AP_MAX_NAME_SIZE] = 0;
			// find existing param so we can get the old value
			vp = AP_Param::find(key, &var_type);
			if (vp == NULL) {
				return;
			}
			// set the value
			vp->set_float(2, var_type);
			gps_check_state.bad_variance = false;
			gps_check_state.fail_count = 0;
			gps_check_state.back_count = 0;
		} else {
			gps_check_state.back_count++;
			if (gps_check_state.back_count > GPS_CHECK_BACK_ITERATION_MAX) {
				gcs_send_text(MAV_SEVERITY_CRITICAL,"GPS_CHECK:RTK Gone");
				gps_check_state.rtk_gone = true;
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

bool Copter::gps_backnormal()
{
	if (!motors.armed()) {
		return true;
	}
	if (gps.status(1) == AP_GPS::GPS_OK_FIX_3D_RTK) {
		return true;
	} else {
		return false;
	}
}