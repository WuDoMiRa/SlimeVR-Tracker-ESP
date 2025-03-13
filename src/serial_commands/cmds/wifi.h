#include <string>
#include <Arduino.h>
#include <vector>
#include "globals.h"
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
        SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "Wifi"); // the logger.
        String name="wifi"; // The name of the command.
        String description="This command is used to configure the wifi settings of the device."; // Description of command
        /// The run function.
        void run(std::vector<String> arguments){
            if(arguments.size() < 1){
                logger.print("Usage: wifi <ssid> <password>");
                return;
            }
            WiFi.begin(arguments[0].c_str(), arguments[1].c_str());
            logger.print("Connecting to WiFi: %s", arguments[0].c_str());
            Serial.println(arguments[0].c_str());
            int attempts=0;
            while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                Serial.print(".");
                attempts++;
                if(attempts > 10){
                    logger.error("Failed to connect to WiFi. This may be because of incorrect credentials or maybe the tracker is too far away.");
                    return;
                }
            }
            logger.print("Connected to WiFi: %s", arguments[0].c_str());
        }
    };
}
