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
#include <map>

/// TASKS
#include "tasks/calibration.h"

//#include "tasks/calibration.h"
//#include <ArduinoJson.hpp>
/// TODO: add a second imu. this can be a dynamic system later.
SlimeVR::Logger logger(Serial,"SlimeVR");
SlimeVR::SerialManager SMNGR;
std::vector<byte> i2c_addresses;
SlimeVR::FS filesystem;
SlimeVR::TaskManager taskmng; // Task manager for the tasks.
SlimeVR::JsonDocument tracker_config; // settings for the tracker.
// IMU Declarations 
#ifdef IMU==IMU_ICM42688 
    #include "IMU/ICM42688_drv.h"
    SlimeVR::ICM42688_DRIVER imu1;
#endif

std::optional<SlimeVR::ICM42688_DRIVER> imu2 = std::nullopt; // optional IMU2, if present.
//SlimeVR::FSConfig fsConfig;
//ArduinoJson::JsonDocument tracker_config;
void setup() {
	//if (!fsConfig.fileExists("/config.json")) {
	//	logger.warn("Config file not found. Creating default config.");
	//	fsConfig.SaveJSON("/config.json", tracker_config);
	//}

	pinMode(LED_PIN, OUTPUT); // set up LED
	SlimeVR::led_on();
	Serial.begin(serialBaudRate);
	logger.print("Booting up");
	//if (!filesystem.exists("/config.json")) {
	//	logger.error("Config file not found. Creating default config.");
	//	filesystem.createFile("/config.json");
	//} else {
	//	if (!filesystem.readJSON("/config.json", tracker_config)) {
	//		logger.error("Failed to read config file.");
	//	}
	//}
	// check if calibration.imu1.accelthres exists, and if so, set the threshold to imu1 rn
	//if ((tracker_config.containsKey("calibration")) && 
	//	(tracker_config["calibration"].containsKey("imu1")) &&
	//	(tracker_config["calibration"]["imu1"].containsKey("accelthres"))) {
	//	logger.print("Calibration data found.");
	//	auto accelthres = tracker_config["calibration"]["imu1"]["accelthres"];
	//	logger.print("Setting acceleration threshold to %f %f %f", accelthres[0], accelthres[1], accelthres[2]);
	//	imu1.acceleration_threshold = SlimeVR::Vector3(accelthres[0], accelthres[1], accelthres[2]);
	//} else {
	//	logger.print("No calibration data found. Flip over the tracker and reboot the tracker, keep it upside down to start the calibration process.");
	//}
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

	if (!imu1.imu_init()) {
		logger.error("Failed to initialize the primary IMU.");
	} else {
		logger.print("Successfully initialized primary IMU %s.", imu1.name);
		//imu1.VQF_init();
	}


	// Wait for IMU to boot
	delay(500);
	logger.print("Scanning I2C bus for devices...");
	// scan for i2c devices upon boot.
	byte error,address;
	for(address = 1; address < 127; address++ ) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0)
		{
			i2c_addresses.push_back(address);
		}
		else if (error==4)
		{
			logger.error("Device did not acknowledge the address: 0x%d",address);
		}
	}
	logger.print("Found %d I2C devices.",i2c_addresses.size());
	for ( auto &i : i2c_addresses ) {
		logger.info("Device found at address 0x%x",i);
	}
	SlimeVR::CalibrationTask* CalTask = new SlimeVR::CalibrationTask(); // Dynamically create to extend lifetime. If you don't do this for any new task, you risk `use-after-free` (using a variable after its been freed) which causes crashes on the tracker.
	CalTask->state["imu1"] = static_cast<SlimeVR::IMUObj*>(&imu1); // set the imu1 in the state of the task.
	CalTask->state["tracker_config"] = &tracker_config; // set the tracker config in the state of the task.
	CalTask->state["filesystem"] = &filesystem; // set the filesystem in the state of the task.
	CalTask->init();
	taskmng.addTask(*CalTask); // add the calibration task to the task manager.

	//logger.print("Calibrating IMU.");
	//imu1.position_estimation_filter.calibrateBiases(imu1.acceleration_threshold, imu1.gyro_threshold); // calibrate the biases of the IMU.
	SMNGR.SetupCMDs();
}	
void loop() {
	imu1.update();
	#if USE_POSITION_ESTIMATION_FILTER
	imu1.position_estimation();
	logger.print("position: %f %f %f", imu1.position.x(), imu1.position.y(), imu1.position.z());
	logger.print("velocity: %f %f %f", imu1.velocity.x(), imu1.velocity.y(), imu1.velocity.z());
	logger.print("gyro threshold: %f %f %f", imu1.gyro_threshold.x(), imu1.gyro_threshold.y(), imu1.gyro_threshold.z());
	logger.print("accel threshold: %f %f %f", imu1.acceleration_threshold.x(), imu1.acceleration_threshold.y(), imu1.acceleration_threshold.z());
	#endif
	taskmng.runTasks(); // run the tasks.
	SMNGR.ReadSerial();
}
