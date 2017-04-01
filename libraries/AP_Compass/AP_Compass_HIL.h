#pragma once

#include "AP_Compass.h"

#define HIL_NUM_COMPASSES 1

class AP_Compass_HIL : public AP_Compass_Backend
{
public:
    AP_Compass_HIL(Compass &compass);
    void read(void);
    bool init(void);
	//添加变量
	float rtkyawDeg;
	Vector3f rtkmagfiled;
	Matrix3f rtkR;
	bool inintyaw;
	int badgpsyaw;//gps.yaw变坏次数
    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

private:
    uint8_t     _compass_instance[HIL_NUM_COMPASSES];
};
