#ifndef __SERVER_CONNECTION_TASK_H__
#define __SERVER_CONNECTION_TASK_H__

#include "GlobalVars.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "utils.h"

namespace SlimeVR {
    class ServerConnectionTask : public Task {
    public:
        static const uint16_t BROADCAST_PORT = 6969;

        ServerConnectionTask() :
            logger(Serial, "SlimeVR", "ServerConnection"),
            udpInitialized(false) {}

        void init() override {
            type = RUN_EVERY_CYCLE;
            unsigned long lastKeepalive = 0;
            
            func = [this, lastKeepalive](SlimeVR::TaskState &state) mutable {
                auto* imu1 = std::any_cast<IMUObj*>(state["imu1"]);
                unsigned long now = millis();
                
                // Initialize UDP socket if WiFi connected
                if(WiFi.status() == WL_CONNECTED && !udpInitialized) {
                    if(udp.begin(BROADCAST_PORT)) {
                        udpInitialized = true;
                        logger.print("Listening for broadcasts on port %d", BROADCAST_PORT);
                    }
                }

                // Process incoming packets
                if(udpInitialized) {
                    processPackets();
                }

                // Send IMU data if connected
                if(state.contains("shouldSendIMUData") && std::any_cast<bool>(state["shouldSendIMUData"])) {
                    //logger.debug("max coeff: %f", imu1->acceleration.maxCoeff());
                    // calculate the magnitude of the acceleration vector
                    float magnitude = sqrt(pow(imu1->acceleration.x(), 2) + pow(imu1->acceleration.y(), 2) + pow(imu1->acceleration.z(), 2));
                    // create a check where magnitude is greater than being still
                    if(magnitude > 1.05f) {
                        // Send actual IMU data if moving
                        sendIMUData(
                            imu1->acceleration.x(),
                            imu1->acceleration.y(),
                            imu1->acceleration.z(),
                            imu1->gyro.x(),
                            imu1->gyro.y(),
                            imu1->gyro.z(),
                            imu1->quat.w(),
                            imu1->quat.x(),
                            imu1->quat.y(),
                            imu1->quat.z()
                        );
                        lastKeepalive = now; // Reset keepalive timer
                    }
                    else if (now - lastKeepalive >= 2000) { // Send keepalive every 1 second
                        // Send empty keepalive packet
                        uint8_t emptyPacket[] = {};
                        udp.beginPacket(serverAddress, 5700);
                        udp.write(emptyPacket, sizeof(emptyPacket));
                        udp.endPacket();
                        lastKeepalive = now;
                       // logger.debug("Sent keepalive packet");
                    }
                }
            };
        }

    private:
        Logger logger;
        WiFiUDP udp;
        bool udpInitialized;
        IPAddress serverAddress;
        bool connected = false;

        void processPackets() {
            int packetSize = udp.parsePacket();
            if(packetSize <= 0) {
                if(WiFi.status() != WL_CONNECTED) {
                    udpInitialized = false;
                    connected = false;
                    serverAddress = INADDR_NONE; // Clear stored IP
                    state["shouldSendIMUData"] = false;
                }
                return;
            }

            char packetBuffer[256];
            int len = udp.read(packetBuffer, sizeof(packetBuffer)-1);
            if(len > 0) {
                packetBuffer[len] = 0;
                
                if(strcmp(packetBuffer, "DISCOVER_SLIMEVR") == 0) {
                    IPAddress remote = udp.remoteIP();
                    logger.debug("Discovery request from %s", remote.toString().c_str());
                    if(respondToDiscovery(remote)) {
                        serverAddress = remote;
                        connected = true;
                        logger.print("Connected to server at %s", serverAddress.toString().c_str());
                    }
                }
            }
        }

        bool respondToDiscovery(IPAddress server) {
            udp.beginPacket(server, 5700);
            udp.write("TRACKER_RESPONSE");
            bool success = udp.endPacket();
            if (success) {
                // Start sending IMU data immediately after successful response
                state["shouldSendIMUData"] = true;
            }
            return success;
        }

        void sendIMUData(float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ, float quatW, float quatX, float quatY, float quatZ) {
            if(!connected) return;

            //static unsigned long lastSendTime = 0;
            //static unsigned long packetCount = 0;
            //static unsigned long lastRateCalc = 0;
            //unsigned long now = millis();
            //unsigned long interval = imu1.get_hrtz_ms();
            
            // Rate limit sends
            //if(now - lastSendTime < interval) {
            //    return;
            //}
            //lastSendTime = now;

            // Create packet with IMU data + quaternion (40 bytes total)
            uint8_t packet[40];
            
            // First 24 bytes: accel + gyro
            memcpy(packet, &accelX, 4);
            memcpy(packet+4, &accelY, 4);
            memcpy(packet+8, &accelZ, 4);
            memcpy(packet+12, &gyroX, 4);
            memcpy(packet+16, &gyroY, 4);
            memcpy(packet+20, &gyroZ, 4);
            
            // Next 16 bytes: quaternion (w, x, y, z)
            memcpy(packet+24, &quatW, 4);
            memcpy(packet+28, &quatX, 4);
            memcpy(packet+32, &quatY, 4);
            memcpy(packet+36, &quatZ, 4);

            udp.beginPacket(serverAddress, 5700);
            udp.write(packet, sizeof(packet));
            udp.endPacket();
            
            // Calculate actual send rate
            //packetCount++;
            //if(now - lastRateCalc >= 1000) {
            //    float actualRate = (packetCount * 1000.0) / (now - lastRateCalc);
            //    logger.debug("Actual send rate: %.1f Hz", actualRate);
            //    packetCount = 0;
            //    lastRateCalc = now;
            //}
        }
    };
}

#endif // __SERVER_CONNECTION_TASK_H__