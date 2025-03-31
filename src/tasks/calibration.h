#ifndef __CALIBRATION_TASK_H__
#define __CALIBRATION_TASK_H__
#include "GlobalVars.h"
#include <any>
namespace SlimeVR {
    struct CalibrationTask : Task {
        SlimeVR::Logger logger = SlimeVR::Logger(Serial,"SlimeVR","CalibrationTask");
        void init() override {
            type = RUN_ONCE; // runs once
            func = [this](SlimeVR::TaskState &state) { // the function to run.
                logger.debug("Running."); delay(100);
                auto* imu1 = std::any_cast<IMUObj*>(state["imu1"]); // get the imu1 from the state.
                logger.debug("After cast from imu1->"); delay(100);
                auto* tracker_config = std::any_cast<ArduinoJson::JsonDocument*>(state["tracker_config"]); // get the tracker config from the state.
                auto* filesystem = std::any_cast<SlimeVR::FS*>(state["filesystem"]); // get the filesystem from the state.

                // This is tricky.
                // state['imu1'] is actually a driver/separate class deriving from a base class. But that means that the class must have functions implemented in the base class.
                // So just simply casting it to the base class is sufficient enough, but how? Without erroring?

                // Apparently, doing variables like ArduinoJson::JsonDocument tracker_config state['tracker_config'] gives errors
                // so we need to cast to std::any and then to the type we want. 

                // All variables passed in state are referenced/borrowed. i.e state['imu1'] is &imu1->
                


                imu1->update();
                if (imu1->acceleration.z() < 0.f) {
                    logger.print("Calibrating IMU. Keep the tracker still and flat for around 10 seconds."); 
                    delay(2000); // give user 2 seconds to read
                    delay(10000);
                    logger.print("Creating Acceleration Threshold.");
                    SlimeVR::Vector3 lastaccelthres;
                    SlimeVR::Vector3 gyrothres;
                    for (int i = 0; i < 11; i++) {
                        if (i==0) {imu1->update(); lastaccelthres=imu1->acceleration; gyrothres=imu1->gyro; continue; } // skip the first iteration to get the first sample.
                        delay(1000);
                        imu1->update();
                        auto deductible = (imu1->acceleration-lastaccelthres).array().abs().maxCoeff();
                        if (deductible >= 0.001f) {
                            lastaccelthres*= deductible; // multiply the last acceleration threshold by the deductible.
                        }
                        auto gyrdeductible = (imu1->gyro-gyrothres).array().abs().maxCoeff();
                        if (gyrdeductible >= 0.001f) {
                            gyrothres*= gyrdeductible; // multiply the last gyro threshold by the deductible.
                        }
                        SlimeVR::led_blink(500);
                    }
                    imu1->acceleration_threshold=lastaccelthres;
                    imu1->gyro_threshold=gyrothres;
                    logger.print("Acceleration Threshold: %f %f %f", lastaccelthres.x(), lastaccelthres.y(), lastaccelthres.z()); delay(100);
                    imu1->position_estimation_filter.calibrateBiases(lastaccelthres, gyrothres);

                    JsonArray accelThresArray = (*tracker_config)["calibration"]["imu1"]["accelthres"].createNestedArray();
                    accelThresArray.add(lastaccelthres.x());accelThresArray.add(lastaccelthres.y());accelThresArray.add(lastaccelthres.z());

                    JsonArray gyroThresArray = (*tracker_config)["calibration"]["imu1"]["gyrothres"].createNestedArray();
                    gyroThresArray.add(gyrothres.x());gyroThresArray.add(gyrothres.y());gyroThresArray.add(gyrothres.z());
                    if (!filesystem->saveJSON("/config.json", *tracker_config)) {
                        logger.error("Failed to save configuration. You may have a corrupted configuration file now.");
                    } else {
                        logger.print("Saved calibration data.");
                    }
                } else {
                    logger.print("Z axis is normal: %f", imu1->acceleration.z());
                }
            };
        };
    };
}

//void virtual init();

#endif 