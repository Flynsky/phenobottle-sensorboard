#ifndef OPT_H
#define OPT_H

#include "ClosedCube_OPT3002.h"
#include <Arduino.h>
#include <Wire.h>

#include <queue>

extern std::queue<OPT3002> opt3002_left_queue;
extern std::queue<OPT3002> opt3002_right_queue;

void setup_opt();

#endif // OPT_H
