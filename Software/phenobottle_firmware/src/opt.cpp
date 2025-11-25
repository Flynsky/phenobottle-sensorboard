#include "ClosedCube_OPT3002.h"
#include <Arduino.h>
#include <Wire.h>

#include <queue>

#include "opt.h"
#include "pins.h"

#define OPT_ERR_CHECK(x)    \
    do { \
        OPT3002_ErrorCode err = (x); \
        if (err != OPT3002_ErrorCode::NO_ERROR) { \
            Serial.printf("OPT3002 Error: %d (Line %d)\n", err, __LINE__); \
            Serial.flush(); \
            return; \
        } \
    } while (0)

#define OPT3002_ADDRESS_BASE 0x45

#define OPT_CONFIG_REG 0x0E00
#define OPT_LL_CONFIG_INT 0xC000

const OPT3002_Config config = {
    .rawData = OPT_CONFIG_REG
};

// Sensor "left" is closest to the short board edge
ClosedCube_OPT3002 opt3002_left(Wire1);
ClosedCube_OPT3002 opt3002_right(Wire1);

std::queue<OPT3002> opt3002_left_queue;
std::queue<OPT3002> opt3002_right_queue;

void IRAM_ATTR opt3002_left_isr() {
    OPT3002 result = opt3002_left.readResult();
    OPT_ERR_CHECK(opt3002_left.writeConfig(config));
    opt3002_left_queue.push(result);
}

void IRAM_ATTR opt3002_right_isr() {
    OPT3002 result = opt3002_right.readResult();
    OPT_ERR_CHECK(opt3002_right.writeConfig(config));
    opt3002_right_queue.push(result);
}

void setup_opt() {
    // Initialize the left sensor
    OPT_ERR_CHECK(opt3002_left.begin(OPT3002_ADDRESS_BASE));
    // Initialize the right sensor
    OPT_ERR_CHECK(opt3002_right.begin(OPT3002_ADDRESS_BASE));

    if(opt3002_left.readManufacturerID() != 0x5449) {
        Serial.println("Error: Left OPT3002 Manufacturer ID mismatch");
        return;
    }

    if(opt3002_right.readManufacturerID() != 0x5449) {
        Serial.println("Error: Right OPT3002 Manufacturer ID mismatch");
        return;
    }

    attachInterrupt(digitalPinToInterrupt(PIN_OPT_INT_1), opt3002_left_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_OPT_INT_2), opt3002_right_isr, FALLING);

    // write LL config register
    OPT3002_ER opt_ll_config;
    opt_ll_config.rawData = OPT_LL_CONFIG_INT;
    OPT_ERR_CHECK(opt3002_left.writeLowLimit(opt_ll_config));
    OPT_ERR_CHECK(opt3002_right.writeLowLimit(opt_ll_config));

    // write config register
    OPT_ERR_CHECK(opt3002_left.writeConfig(config));
    OPT_ERR_CHECK(opt3002_right.writeConfig(config));
}
