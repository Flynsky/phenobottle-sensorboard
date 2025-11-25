#ifndef IMU_H
#define IMU_H

#include "ICM42670P.h"
#include <Arduino.h>

extern char init_imu;
const int ICM_SDA = 36;
const int ICM_SCL = 37;
extern ICM42670 IMU;

void setup_imu();

#endif
