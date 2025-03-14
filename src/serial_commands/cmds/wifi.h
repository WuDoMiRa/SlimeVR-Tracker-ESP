#include <string>
#include <Arduino.h>
#include <vector>
#include "globals.h"
#include <LittleFS.h>
/// TODO: for some reason, the IDE shows no errors about importing ESP8266WiFi.h, and
/// I can see definitions about the 'WiFi' class, however, when it comes to actually building,
/// compiling will show that 'ESP8266WiFi.h' doesn't exist. Tried a reinstall of platformio.ini (deleting the folder/libraries and having it reinstall)
/// only for it to not work again. Maybe don't develop on Windows next time, future me?

/// The problem wasn't because I was using windows but because for some reason developing with platformio,
/// you need both .c and .h if you want to use something like the wifi library.
/// literally tried searching up for possible reasons as for why, even the error message i was getting while compiling,
/// and absolutely nothing.
namespace SlimeVR {
    struct WifiSerialCommand {
        /// NOTE: the blow function is commented out because of rst.
        //WifiSerialCommand(){
            /// NOTE: IF THE TRACKER BEGINS TO CRASH/THROW STACK ERRORS,
            /// THIS FUNCTION IS WHY!
            // stuff copy and pasted from the main slimevr branch
        //    WiFi.persistent(true);
        //    WiFi.mode(WIFI_STA);
        //    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
        //    WiFi.hostname("SlimeVR FBT Tracker");
        //    WiFi.begin();  // Should connect to last used access point, see
            // https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/station-class.html#begin
        //}
        SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "Wifi"); // the logger.
        String name="wifi"; // The name of the command.
        String description="This command is used to configure the wifi settings of the device."; // Description of command
        /// The run function.
        void run(std::vector<String> &arguments, bool fallback_11g=false, bool fallback_11b=false){
            if(arguments.size() < 1){
                logger.print("Usage: wifi <ssid> <password>(optional)");
                return;
            }
            // if we're on the 2nd fallback (11g)
            if(fallback_11g && (!fallback_11b)){
                WiFi.setPhyMode(WIFI_PHY_MODE_11G);
            }
            // if we're on the 2nd fallback (11b)
            if (fallback_11b & (!fallback_11g)){
                WiFi.setPhyMode(WIFI_PHY_MODE_11B);
            }
            WiFi.begin(arguments[0].c_str(), arguments[1].c_str());
            logger.print("Connecting to WiFi: %s", arguments[0].c_str());
            int attempts=0;
            while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                attempts++;
                if(attempts > 40){
                    switch (WiFi.status()) {
                        case WL_IDLE_STATUS:
                            logger.error("WiFi is in idle state");
                            break;
                        case WL_NO_SSID_AVAIL:
                            logger.error("SSID not found");
                            break;
                        case WL_WRONG_PASSWORD:
                            logger.error("Wrong password.");
                            break;
                        case WL_CONNECT_FAILED:
                            logger.error("Connection failed");
                            break;
                        case WL_DISCONNECTED:
                            logger.error("WiFi is disconnected");
                            break;
                        default:
                            logger.error("Failed to connect to WiFi. Unknown error: %d", WiFi.status());
                            break;
                    }
                    delay(100);
                    // first fallback
                    if ((!fallback_11g) && (!fallback_11b)){
                        logger.print("Trying to re-connect with 11G.");
                        return run(arguments,true); // recursion for fallback
                    }
                    // second fallback
                    if (!fallback_11b) {
                        logger.print("Trying to re-connect with 11B.");
                        return run(arguments,false,true); // recursion for fallback
                    }
                    // we ran out of fallbacks.
                    WiFi.setPhyMode(WIFI_PHY_MODE_11N); // reset back to 11n, could call the init again but
                    // that would just make the module connect back to the last used access point.
                    return;
                }
            }
            logger.print("Connected to WiFi: %s", arguments[0].c_str());
        }
    };
}
