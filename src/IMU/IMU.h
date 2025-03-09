#ifndef IMUOBJECT_H
#define IMUOBJECT_H
#include <Wire.h>
#include <Arduino.h>
#include "../GlobalVars.h"
#include "../globals.h"
#include <memory>
#include "../math/vector3.h"
#include "../math/quat.h"
#include "../vqf/vqf/cpp/vqf.hpp"
namespace SlimeVR {
    /// The IMU object for any IMUs.
    /// ALL IMU OBJECTS THAT INHERIT FROM THIS CLASS MUST INCLUDE '_DRIVER' as the suffix in their name.

    /// This will handle handling things specifically tailored towards the IMU, and aims to be as abstract as possible.
    /// It will NOT have configurable TPS values as it will NOT be sending ANY kind of data in of itself,
    /// and instead data would be sent from main.cpp, as 'main.cpp' would be the 'tracker' itself, and not 'IMU.h'.
    /// This localizes things and makes things easier for developers, this rewrite was necessary. in fact, you could get even less packets a cycle
    /// by batching all packets in-of itself, instead of sending things seperately like before, like imu rot, accel, and sometimes battery.
    /// These classes/structs SHOULD NOT BE SENDING DATA! THEY SHOULD NOT BE IN CHARGE OF SENDING DATA!
    template<class T>
    struct IMUObj {
        IMUObj() = default;
        SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "IMU");
        std::unique_ptr<T> imu_drv; // The underlying IMU driver that the IMU uses.
        Vector3 acceleration;
        Vector3 gyro;
        char* name;
        int id;
        int address;
        int fifo_size = 64;

        Vector3 position;
        Vector3 rotation;
        Quat quat;

        // TODO: have a dynamic filter, although
        // instead of implementing these filters in the drivers,
        // implement them in here,
        // but you'd have to constantly rewrite functions for different VQF
        // filters, but that is code. you cannot have it the easy way here.

        VQFParams vqf_parameters;
        VQF vqf_filter = VQF(vqf_parameters, 0.01);
        //VQF vqf_filter;
        /// @brief Initializes the VQF filter.
        //void VQF_init() {
        //    vqf_filter = VQF(vqf_parameters, 0.01);
            //vqf_init();
        //}

        /// @brief Updates the VQF filter, and updates quaternion.
        void VQF_update() {
            // TODO: replace the VQF library EDIT: done
            vqf_real_t vqfgyr[3] = {rotation.x, rotation.y, rotation.z};
            vqf_real_t vqfacc[3] = {acceleration.x, acceleration.y, acceleration.z};
            vqf_real_t vqfquat[4] = {0,0,0,0};
            vqf_filter.update(vqfgyr, vqfacc);
            vqf_filter.getQuat6D(vqfquat);
            //logger.print("VQF Quat: %f %f %f %f", vqfquat[0], vqfquat[1], vqfquat[2], vqfquat[3]);
            quat = {vqfquat[0], vqfquat[1], vqfquat[2], vqfquat[3]};

        }
        /// @brief Initializes the IMU for usage.
        /// Also initializes other things like VQF if enabled. Returns 'true' if working.
        bool imu_init();

        /// @brief Updates acceleration and gyro.
        void update();
    };
}
#endif