#ifndef __UPKEEP_WIFI_TASK_H__
#define __UPKEEP_WIFI_TASK_H__

#include "GlobalVars.h"
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include "utils.h"

namespace SlimeVR {
    class UpkeepWifiTask : public Task {
    public:
        UpkeepWifiTask() : 
            logger(Serial, "SlimeVR", "UpkeepWifi"),
            lastConnectionState(false),
            lastBlinkTime(0),
            lastPrintTime(0) {}

        void init() override {
            type = RUN_EVERY_CYCLE;
            unsigned long lastProvCheck = 0;
            
            func = [this, lastProvCheck](SlimeVR::TaskState &state) mutable {
                bool currentConnected = (WiFi.status() == WL_CONNECTED);
                unsigned long now = millis();
                
                // Handle state changes
                if(currentConnected != lastConnectionState) {
                    if(currentConnected) {
                        logger.print("Successfully connected to WiFi (SSID: %s, IP: %s)",
                            WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
                        SlimeVR::led_on(); // Solid LED when connected
                        if(provisioning) {
                            stopprov();
                        }
                    } else {
                        logger.print("Disconnected from WiFi. Reason: %d - %s",
                            WiFi.status(), getDisconnectReason(WiFi.status()));
                        lastBlinkTime = millis(); // Start blinking
                    }
                    lastConnectionState = currentConnected;
                }

                // Handle LED blinking when disconnected
                if(!currentConnected) {
                    static bool ledState = false;
                    if(now - lastBlinkTime > 500) { // 500ms blink interval
                        if(ledState) {
                            SlimeVR::led_off();
                        } else {
                            SlimeVR::led_on();
                        }
                        ledState = !ledState;
                        lastBlinkTime = now;
                    }
                }

                // Check provisioning every 5 seconds if not connected
                if(!currentConnected && now - lastProvCheck >= 5000) {
                    lastProvCheck = now;
                    int wifiStatus = WiFi.status();
                    bool isConnecting = (wifiStatus == WL_IDLE_STATUS || wifiStatus == WL_DISCONNECTED);
                    
                    if(!provisioning && !isConnecting) {
                        beginprov();
                    } else if(provisioning && isConnecting) {
                        stopprov();
                    }
                }
            };
        }

    private:
        Logger logger;
        bool lastConnectionState;
        unsigned long lastBlinkTime;
        unsigned long lastPrintTime;
        bool provisioning = false; // Whether or not we are provisioning the device.

        const char* getDisconnectReason(int status) {
            switch(status) {
                case WL_IDLE_STATUS: return "Idle";
                case WL_NO_SSID_AVAIL: return "SSID not available";
                case WL_CONNECT_FAILED: return "Connection failed";
                case WL_CONNECTION_LOST: return "Connection lost";
                case WL_DISCONNECTED: return "Disconnected";
                default: return "Unknown reason";
            }
        }

        void beginprov(){
            if (WiFi.beginSmartConfig()) {
                logger.debug("Provisioning started.");
                provisioning=true;
            }
        }
        void stopprov(){
            WiFi.stopSmartConfig();
            logger.debug("Provisioning stopped.");
            provisioning=false;
        }
    };
}

#endif // __UPKEEP_WIFI_TASK_H__