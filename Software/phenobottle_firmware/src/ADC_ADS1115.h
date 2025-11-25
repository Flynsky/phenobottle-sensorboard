#ifndef ADC_ADS1115_H
#define ADC_ADS1115_H

#include <ADS1115_WE.h>
#include <Arduino.h>
#include <Wire.h>

const int I2C_ADDRESS = 0x48;
const int SDA_1 = 34;
const int SCL_1 = 33;

void setup_adc();
float readChannel(ADS1115_MUX channel,ADS1115_RANGE = ADS1115_RANGE_6144);
float readPH(const float Temp_C=25.0);
float readTIA();
float readNTC();
#endif
