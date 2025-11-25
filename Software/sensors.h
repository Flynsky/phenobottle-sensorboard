#ifndef SENSORS_H
# define SENSORS_H

extern volatile char adc_init = 0;

void adc_setup();

uint8_t temp_isconnected();
float temp_read_one();
float ph_read(const float Temp_C);
float tia_read();

# endif