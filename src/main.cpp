/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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

#include <i2cscan.h>

#include "GlobalVars.h"
#include "Wire.h"
#include "globals.h"

Logger logger(Serial,"SlimeVR");
IMUObj imu1;
#ifdef SECOND_IMU != IMU
IMUObj imu2; // define second
#endif
//SlimeVR::Logging::Logger logger("SlimeVR");
void setup() {
	pinMode(LED_PIN, OUTPUT); // set up LED
	digitalWrite(LED_PIN, LED__ON); // turn ON LED
	Serial.begin(serialBaudRate);
	// Make sure the bus isn't stuck when resetting ESP without powering it down
	// Fixes I2C issues for certain IMUs. Previously this feature was enabled for
	// selected IMUs, now it's enabled for all. If some IMU turned out to be broken by
	// this, check needs to be re-added.
	auto clearResult = I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL);
	if (clearResult != 0) {
		logger.error("Can't clear I2C bus, error %d", clearResult);
	}

	// join I2C bus

#if ESP32
	// For some unknown reason the I2C seem to be open on ESP32-C3 by default. Let's
	// just close it before opening it again. (The ESP32-C3 only has 1 I2C.)
	Wire.end();
#endif

	// using `static_cast` here seems to be better, because there are 2 similar function
	// signatures
	Wire.begin(static_cast<int>(PIN_IMU_SDA), static_cast<int>(PIN_IMU_SCL));

#ifdef ESP8266
	Wire.setClockStretchLimit(150000L);  // Default stretch limit 150mS
#endif
#ifdef ESP32  // Counterpart on ESP32 to ClockStretchLimit
	Wire.setTimeOut(150);
#endif
	Wire.setClock(I2C_SPEED);

	// Wait for IMU to boot
	delay(500);
	digitalWrite(LED_PIN, LED__OFF);
}


void loop() {
	digitalWrite(LED_PIN, LED__ON); // turn ON LED
	logger.print("LED ON");
	delay(1000); // wait for a second
	digitalWrite(LED_PIN, LED__OFF); // turn OFF LED
	logger.print("LED OFF");
	delay(1000); // wait for a second
}
