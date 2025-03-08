#ifndef IMUOBJECT_H
#define IMUOBJECT_H
#include <Wire.h>
#include <Arduino.h>
#include <GlobalVars.h>
#include <globals.h>
/// The IMU object for any IMUs.
/// This will handle handling things specifically tailored towards the IMU, and aims to be as abstract as possible.
/// It will NOT have configurable TPS values as it will NOT be sending ANY kind of data in of itself,
/// and instead data would be sent from main.cpp, as 'main.cpp' would be the 'tracker' itself, and not 'IMU.h'.
/// This localizes things and makes things easier for developers, this rewrite was necessary. in fact, you could get even less packets a cycle
/// by batching all packets in-of itself, instead of sending things seperately like before, like imu rot, accel, and sometimes battery.
/// These classes/structs SHOULD NOT BE SENDING DATA! THEY SHOULD NOT BE IN CHARGE OF SENDING DATA!
struct IMUObj {
    int name;
    int id;
    int address;

    /// @brief Initializes the IMU for usage.
    /// Also initializes other things like VQF if enabled. 
    void virtual init();
};
#endif