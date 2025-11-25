/*

Arduino library for Texas Instruments OPT3002 Light to Digital Sensor
Written by AA for ClosedCube
---

The MIT License (MIT)

Copyright (c) 2016 ClosedCube Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#ifndef CLOSEDCUBE_OPT3002
#define CLOSEDCUBE_OPT3002

#include <Arduino.h>
#include <Wire.h>

typedef enum {
	RESULT = 0x00,
	CONFIG = 0x01,
	LOW_LIMIT = 0x02,
	HIGH_LIMIT = 0x03,

	MANUFACTURER_ID = 0x7E,
	DEVICE_ID = 0x7F,
} OPT3002_Commands;


typedef enum {
	NO_ERROR = 0,
	TIMEOUT_ERROR = -100,

	WIRE_I2C_DATA_TOO_LOG = -10,
	WIRE_I2C_RECEIVED_NACK_ON_ADDRESS = -20,
	WIRE_I2C_RECEIVED_NACK_ON_DATA = -30,
	WIRE_I2C_UNKNOW_ERROR = -40
} OPT3002_ErrorCode;

typedef union {
	uint16_t rawData;
	struct {
		uint16_t Result : 12;
		uint8_t Exponent : 4;
	};
} OPT3002_ER;


typedef union {
	struct {
		uint8_t FaultCount : 2;
		uint8_t MaskExponent : 1;
		uint8_t Polarity : 1;
		uint8_t Latch : 1;
		uint8_t FlagLow : 1;
		uint8_t FlagHigh : 1;
		uint8_t ConversionReady : 1;
		uint8_t OverflowFlag : 1;
		uint8_t ModeOfConversionOperation : 2;
		uint8_t ConvertionTime : 1;
		uint8_t RangeNumber : 4;
	};
	uint16_t rawData;
} OPT3002_Config;

struct OPT3002 {
	float lux;
	OPT3002_ER raw;
	OPT3002_ErrorCode error;
};

class ClosedCube_OPT3002 {
public:
	ClosedCube_OPT3002(TwoWire &i2c);

	OPT3002_ErrorCode begin(uint8_t address);

	uint16_t readManufacturerID();
	uint16_t readDeviceID();

	OPT3002 readResult();
	OPT3002 readHighLimit();
	OPT3002 readLowLimit();

	OPT3002_Config readConfig();
	OPT3002_ErrorCode writeConfig(OPT3002_Config config);
	OPT3002_ErrorCode writeLowLimit(OPT3002_ER low_limit);

private:
	uint8_t _address;
	TwoWire* i2c;

	OPT3002_ErrorCode writeData(OPT3002_Commands command);
	OPT3002_ErrorCode readData(uint16_t* data);

	OPT3002 readRegister(OPT3002_Commands command);
	OPT3002 returnError(OPT3002_ErrorCode error);

	float calculateLux(OPT3002_ER er);
};


#endif
