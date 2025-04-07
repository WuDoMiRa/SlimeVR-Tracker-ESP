#ifndef __CALIBRATION_TASK_H__
#define __CALIBRATION_TASK_H__
#include "GlobalVars.h"
#include <Arduino.h>
#include <any>
namespace SlimeVR {
    struct CalibrationTask : public Task {
        SlimeVR::Logger logger = SlimeVR::Logger(Serial,"SlimeVR","CalibrationTask");
        enum State { IDLE, PHASE1, PHASE2, COMPLETE };
        State currentState = IDLE;
        unsigned long phaseStartTime = 0;
        int sampleCount = 0;
        SlimeVR::Vector3 lastaccelthres;
        SlimeVR::Vector3 gyrothres;
        SlimeVR::Vector3 biasSum;
        bool shownPhase2Message;
        
        bool initialUpsideDown = false;
        bool calibrationStarted = false;
        
        void init() override {
            // Get IMU reference early to check initial orientation
            auto* imu1 = std::any_cast<IMUObj*>(state["imu1"]);
            imu1->update();
            initialUpsideDown = (imu1->acceleration.z() < 0.f);
            
            type = RUN_EVERY_CYCLE;
            
            func = [this](SlimeVR::TaskState &state) {
                // Skip if device wasn't upside down at power-on
                if(!initialUpsideDown && !calibrationStarted) {
                    return;
                }
                
                // Critical section protection
                noInterrupts();
                
                // More conservative memory check
                if(ESP.getFreeHeap() < 25000) {
                    logger.error("CRITICAL: Low heap %d", ESP.getFreeHeap());
                    interrupts();
                    return;
                }
                
                // Stack guard
                volatile uint32_t stackCanary;
                __asm__ __volatile__ ("mov %0, sp" : "=r" (stackCanary));
                if(stackCanary < 0x3FFE8000 || stackCanary > 0x3FFFFFFF) {
                    logger.error("Stack corruption in calibration!");
                    ESP.reset();
                }

                auto* imu1 = std::any_cast<IMUObj*>(state["imu1"]);
                auto* tracker_config = std::any_cast<ArduinoJson::JsonDocument*>(state["tracker_config"]);
                auto* filesystem = std::any_cast<SlimeVR::FS*>(state["filesystem"]);

                imu1->update();
                
                switch(currentState) {
                    case IDLE:
                        if(imu1->acceleration.z() < 0.f) {
                            logger.print("Starting calibration - keep tracker upside down");
                            filesystem->deleteFile("/config.json");
                            tracker_config->clear();
                            ArduinoJson::JsonObject calibration = tracker_config->createNestedObject("calibration");
                            calibration.createNestedObject("imu1");
                            currentState = PHASE1;
                            phaseStartTime = millis();
                        }
                        break;
                        
                    case PHASE1:
                        ESP.wdtFeed();
                        handlePhase1(state, imu1, tracker_config, filesystem);
                        //yield();
                        break;
                        
                    case PHASE2:
                        //ESP.wdtFeed(); // Feed before any FS operations
                        //ESP.wdtDisable();
                        handlePhase2(state, imu1, tracker_config, filesystem);
                        break;
                        
                    case COMPLETE:
                        // Calibration complete
                        break;
                }
            };
        };

    private:
        void handlePhase1(SlimeVR::TaskState& state, IMUObj* imu1, ArduinoJson::JsonDocument* tracker_config, SlimeVR::FS* filesystem) {
            unsigned long elapsed = millis() - phaseStartTime;
            
            // Show initial message once
            static bool shownInitialMessage = false;
            if(elapsed < 2000 && !shownInitialMessage) {
                //ESP.wdtFeed();
                logger.print("Calibrating IMU. Keep tracker still and flat for 10s total");
                shownInitialMessage = true;
                return;
            }
            
            // Main sampling phase
            if(elapsed >= 2000 && elapsed < 12000) {
                
                
                imu1->update();
                
                if(sampleCount == 0) {
                    lastaccelthres = imu1->acceleration;
                    gyrothres = imu1->gyro;
                } else {
                    auto deductible = (imu1->acceleration-lastaccelthres).array().abs().maxCoeff();
                    if (deductible >= 0.001f) {
                        lastaccelthres *= deductible;
                    }
                    auto gyrdeductible = (imu1->gyro-gyrothres).array().abs().maxCoeff();
                    if (gyrdeductible >= 0.001f) {
                        gyrothres *= gyrdeductible;
                    }
                }
                
                sampleCount++;
                SlimeVR::led_blink(500);
                
                
            }

            if(elapsed >= 12000) {
                imu1->acceleration_threshold = lastaccelthres;
                imu1->gyro_threshold = gyrothres;
                
                ArduinoJson::JsonObject imu1Cal = (*tracker_config)["calibration"]["imu1"];
                JsonArray accelThresArray = imu1Cal.createNestedArray("accelthres");
                accelThresArray.add(lastaccelthres.x());
                accelThresArray.add(lastaccelthres.y());
                accelThresArray.add(lastaccelthres.z());
                
                JsonArray gyroThresArray = imu1Cal.createNestedArray("gyrothres");
                gyroThresArray.add(gyrothres.x());
                gyroThresArray.add(gyrothres.y());
                gyroThresArray.add(gyrothres.z());
                logger.print("Phase 1 complete - calculated thresholds");
                currentState = PHASE2;
                phaseStartTime = millis();
                sampleCount = 0;
                biasSum = SlimeVR::Vector3::Zero();
                shownPhase2Message = false;
                shownInitialMessage = false;
                biasSum = SlimeVR::Vector3::Zero();
            }
            return;
        }

        void handlePhase2(SlimeVR::TaskState& state, IMUObj* imu1, ArduinoJson::JsonDocument* tracker_config, SlimeVR::FS* filesystem) {
            unsigned long elapsed = millis() - phaseStartTime;
            
            // Show phase 2 instructions once
            if(elapsed < 2000 && !shownPhase2Message) {
                //ESP.wdtDisable();
                logger.print("Phase 2: Move device slowly in any direction for 10 seconds");
                shownPhase2Message = true;
                //ESP.wdtEnable(100);
                return;
            }
            
            // Phase 2 sampling
            if(elapsed >= 2000 && elapsed < 12000) {
                //ESP.wdtDisable();
                noInterrupts();
                
                // Validate memory and stack
                if(ESP.getFreeHeap() < 20000) {
                    logger.error("Low heap in phase 2: %d bytes", ESP.getFreeHeap());
                    interrupts();
                    ESP.wdtEnable(100);
                    return;
                }
                
                volatile uint32_t stackCanary;
                __asm__ __volatile__ ("mov %0, sp" : "=r" (stackCanary));
                if(stackCanary < 0x3FFE8000 || stackCanary > 0x3FFFFFFF) {
                    logger.error("Stack corruption in phase 2!");
                    ESP.reset();
                }
                
                imu1->update();
                
                // Calculate moving average of bias
                Vector3 bias = Vector3::Zero();
                
                Vector3 newBias = (imu1->acceleration.cwiseAbs() - imu1->acceleration_threshold.cwiseAbs()) / 2.0f;
                biasSum += newBias;
                sampleCount++;
                
                // Update bias estimate every 100ms
                if(sampleCount % 5 == 0) {
                    bias = biasSum / sampleCount;
                }
                
                SlimeVR::led_blink(500);
                interrupts();
                ESP.wdtEnable(100);
                return;
            }
            
            // Phase 2 completion
            if(elapsed >= 12000) {
                //ESP.wdtDisable();
                noInterrupts();
                
                // Final bias calculation
                Vector3 finalBias = biasSum / sampleCount;
                
                // Store calibration data
                unsigned long time_to_accumulate_bias = 10000; // 10s active sampling
                (*tracker_config)["time_to_accumulate_bias"] = time_to_accumulate_bias;
                
                JsonArray BDMArray = (*tracker_config).createNestedArray("bias_during_movement");
                BDMArray.add(finalBias.x());
                BDMArray.add(finalBias.y());
                BDMArray.add(finalBias.z());
                
                // Save configuration
                bool saveSuccess = filesystem->saveJSON("/config.json", *tracker_config);
                
                interrupts();
                ESP.wdtEnable(100);
                
                if (!saveSuccess) {
                    logger.error("Failed to save configuration");
                } else {
                    logger.print("Calibration complete. Saved config to /config.json");
                }
                
                currentState = COMPLETE;
                state["complete"] = true;
                imu1->position = Vector3::Zero();
            }
        }
    };
}

//void virtual init();

#endif