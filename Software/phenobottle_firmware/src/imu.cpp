#include "imu.h"
#include "pins.h"

char init_imu = 0;
ICM42670 IMU(Wire1, 0);

void setup_imu() {
  // setup IMU
  Wire1.begin(PIN_I2C2_SDA , PIN_I2C2_SCL ,400000);
  char ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM42670 initialization failed: ");
    Serial.println(ret);
    while (1)
      ;
  } else {
    // Accel ODR = 100 Hz and Full Scale Range = 16G
    IMU.startAccel(100, 16);
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro(100, 2000);
    init_imu = 1;

    // TODO use IRQ for IMU data
  }
}
