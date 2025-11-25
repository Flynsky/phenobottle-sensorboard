#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <Arduino.h>

void plotBuffer(float* buffer, uint32_t length, Stream* stream);

#endif
