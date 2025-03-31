#include <Arduino.h>
#include <vector>
#include "globals.h"
#include <LittleFS.h>
#include <WiFiUdp.h>
#include "basecmd.hpp"
/// TODO: for some reason, the IDE shows no errors about importing ESP8266WiFi.h, and
/// I can see definitions about the 'WiFi' class, however, when it comes to actually building,
/// compiling will show that 'ESP8266WiFi.h' doesn't exist. Tried a reinstall of platformio.ini (deleting the folder/libraries and having it reinstall)
/// only for it to not work again. Maybe don't develop on Windows next time, future me?

/// The problem wasn't because I was using windows but because for some reason developing with platformio,
/// you need both .c and .h if you want to use something like the wifi library.
/// literally tried searching up for possible reasons as for why, even the error message i was getting while compiling,
/// and absolutely nothing.

struct WifiSerialCommand : BaseCommand {
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
    String usage="wifi <ssid>(string) <password>(string,optional)";

    String server_ip="";
    WiFiUDP udpSock; // Listening/sending packets

    bool fallback_11g=false;
    bool fallback_11b=false;

    void setup() override {
        WiFi.setOutputPower(0); logger.debug("Wifi output power set to 0."); delay(100);
        WiFi.persistent(true); logger.debug("Wifi persistent set to true."); delay(100);
        WiFi.mode(WIFI_STA); logger.debug("Wifi mode set to station."); delay(100);
        WiFi.setPhyMode(WIFI_PHY_MODE_11N); logger.debug("Wifi phy mode set to 11n."); delay(100);
        WiFi.hostname("SlimeVR FBT Tracker"); logger.debug("Wifi hostname set to SlimeVR FBT Tracker."); delay(100);
        WiFi.begin(); logger.debug("Wifi begin."); // Should connect to last used access point, see
        // https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/station-class.html#begin
    }

    /// Begins listening on a port.
    void begin_listening() {
        udpSock.begin(3000);  // Standard SlimeVR broadcast port
    }

    /// The run function.
    void run(std::vector<String> &arguments) {

        if(arguments.size() < 1){
            logger.print("Usage: wifi <ssid> <password>(optional)");
            return;
        }
        // if we're on the 1st fallback (11g)
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
                    fallback_11g=true;
                    return run(arguments); // recursion for fallback
                }
                // second fallback
                if (!fallback_11b) {
                    logger.print("Trying to re-connect with 11B.");
                    fallback_11g=false; // reset the 11g fallback
                    fallback_11b=true;
                    return run(arguments); // recursion for fallback
                }
                // we ran out of fallbacks.
                WiFi.setPhyMode(WIFI_PHY_MODE_11N); // reset back to 11n, could call the init again but
                // that would just make the module connect back to the last used access point.
                return;
            }
        }
        logger.print("Connected to WiFi: %s", arguments[0].c_str());
    }
    
    ///listen for broadcast
    void listen_for_broadcast() {
        char packet[255];
        while (server_ip.length() == 0) {
            int packetSize = udpSock.parsePacket();
            if (packetSize) {
                int len = udpSock.read(packet, 255);
                if (len > 0) {
                    packet[len] = '\0';
                    if (String(packet) == "SLIMEVR_DISCOVERY") {
                        server_ip = udpSock.remoteIP().toString();
                        logger.info("Server found at: %s", server_ip.c_str());
                        break;
                    }
                }
            }
            delay(100);
        }
    }
};

