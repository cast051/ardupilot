/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_PX4.cpp - Arduino Library for PX4 magnetometer
 *
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_Compass_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_device.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>
#include "../../ArduCopter/Copter.h"
extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

// constructor
AP_Compass_PX4::AP_Compass_PX4(Compass &compass):
    AP_Compass_Backend(compass),
    _num_sensors(0)
{
}

// detect the sensor
AP_Compass_Backend *AP_Compass_PX4::detect(Compass &compass)
{
    AP_Compass_PX4 *sensor = new AP_Compass_PX4(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_Compass_PX4::init(void)
{
	_mag_fd[0] = open(MAG_BASE_DEVICE_PATH"0", O_RDONLY);
	/*_mag_fd[1] = open(MAG_BASE_DEVICE_PATH"1", O_RDONLY);
	_mag_fd[2] = open(MAG_BASE_DEVICE_PATH"2", O_RDONLY);
*/
    _num_sensors = 0;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (_mag_fd[i] >= 0) {
            _num_sensors = i+1;
        }
    }    
	if (_num_sensors == 0) {
        hal.console->printf("Unable to open " MAG_BASE_DEVICE_PATH"0" "\n");
        return false;
	}

    for (uint8_t i=0; i<_num_sensors; i++) {
        _instance[i] = register_compass();

        // get device id
        set_dev_id(_instance[i], ioctl(_mag_fd[i], DEVIOCGDEVICEID, 0));

        // average over up to 20 samples
        if (ioctl(_mag_fd[i], SENSORIOCSQUEUEDEPTH, 20) != 0) {
            hal.console->printf("Failed to setup compass queue\n");
            return false;                
        }

        // remember if the compass is external
        set_external(_instance[i], ioctl(_mag_fd[i], MAGIOCGEXTERNAL, 0) > 0);
        _count[i] = 0;
        _sum[i].zero();

    }

    // give the driver a chance to run, and gather one sample
    hal.scheduler->delay(40);
    accumulate();
    if (_count[0] == 0) {
        hal.console->printf("Failed initial compass accumulate\n");        
    }

    return true;
}

void AP_Compass_PX4::read(void)
{
    // try to accumulate one more sample, so we have the latest data
    		accumulate();
// avoid division by zero if we haven't received any mag reports
	    	
		if (copter.rtkstatuok&&copter.gps.status() >= 4)
		{
			rtkyawDeg = ToRad(copter.curyaw);
			Vector3f mag_ef(300, 0, 0);
			float roll, pitch, yaw;
			copter.ahrs._body_dcm_matrix.to_euler(&roll, &pitch, &yaw);
			rtkR.from_euler(roll, pitch, rtkyawDeg);
			// Rotate into body frame
			mag_ef = rtkR.transposed() * mag_ef;
			_sum[0] = mag_ef;
			publish_filtered_field(_sum[0], _instance[0]);

			
			if (!copter.compassok)
			{
				copter.compassok = true;
				inintyaw = true;
				
				//copter.ahrs.reset();
			}
		}
		else
		{
			if (!inintyaw)
			{
				rtkyawDeg = ToRad(0);
				Vector3f mag_ef(300, 0, 0);
				rtkR.from_euler(0, 0, 0.0f);
				mag_ef = rtkR.transposed() * mag_ef;
				_sum[0] = mag_ef;//žøÖµ
				publish_filtered_field(_sum[0], _instance[0]);

			}
			else
			{
				Vector3f mag_ef(300, 0, 0);
				float roll, pitch, yaw;
				copter.ahrs._body_dcm_matrix.to_euler(&roll, &pitch, &yaw);

				rtkR.from_euler(roll, pitch, copter.ahrs.yaw);
				mag_ef = rtkR.transposed() * mag_ef;
				_sum[0] = mag_ef;//žøÖµ
				publish_filtered_field(_sum[0], _instance[0]);

			}
		}

}

void AP_Compass_PX4::accumulate(void)
{
    struct mag_report mag_report;
    for (uint8_t i=0; i<_num_sensors; i++) {
        uint8_t frontend_instance = _instance[i];
        while (::read(_mag_fd[i], &mag_report, sizeof(mag_report)) == sizeof(mag_report) &&
               mag_report.timestamp != _last_timestamp[i]) {

            uint32_t time_us = (uint32_t)mag_report.timestamp;
            // get raw_field - sensor frame, uncorrected
            Vector3f raw_field = Vector3f(mag_report.x, mag_report.y, mag_report.z)*1.0e3f;

            // rotate raw_field from sensor frame to body frame
            rotate_field(raw_field, frontend_instance);

            // publish raw_field (uncorrected point sample) for calibration use
            publish_raw_field(raw_field, time_us, frontend_instance);

            // correct raw_field for known errors
            correct_field(raw_field, frontend_instance);

            // accumulate into averaging filter
            _sum[i] += raw_field;
            _count[i]++;

            _last_timestamp[i] = mag_report.timestamp;
        }
    }
}

#endif // CONFIG_HAL_BOARD
