#ifndef IMUOBJECT_H
#define IMUOBJECT_H
#include <Wire.h>
#include <Arduino.h>
#include "../GlobalVars.h"
#include "../globals.h"
#include <memory>
#include "ArduinoEigen.h"
#include "../vqf/vqf/cpp/vqf.hpp"

namespace SlimeVR {
    typedef Eigen::Vector3f Vector3;
    typedef Eigen::Quaternion<float> Quaternion;
    /// The IMU object for any IMUs.
    /// ALL IMU OBJECTS THAT INHERIT FROM THIS CLASS MUST INCLUDE '_DRIVER' as the suffix in their name.

    /// This will handle handling things specifically tailored towards the IMU, and aims to be as abstract as possible.
    /// It will NOT have configurable TPS values as it will NOT be sending ANY kind of data in of itself,
    /// and instead data would be sent from main.cpp, as 'main.cpp' would be the 'tracker' itself, and not 'IMU.h'.
    /// This localizes things and makes things easier for developers, this rewrite was necessary. in fact, you could get even less packets a cycle
    /// by batching all packets in-of itself, instead of sending things seperately like before, like imu rot, accel, and sometimes battery.
    /// These classes/structs SHOULD NOT BE SENDING DATA! THEY SHOULD NOT BE IN CHARGE OF SENDING DATA!
    //template<class T>
    struct IMUObj {
        SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "IMU");
        //std::unique_ptr<T> imu_drv; // The underlying IMU driver that the IMU uses.
        u8 update_hrtz = 120;
        /// @brief Converts the `update_hrtz` into it's `ms` equivalent to equal 120 hertz a second for example.
        double get_hrtz_ms(){return (1.0/update_hrtz)*1000.0;}
        Vector3 acceleration; // X is the first element, Y is the third element, Z is the second element. (X,Z,Y).
        Vector3 gyro; // rotation of the IMU.
        char* name;
        int id;
        int address;
        int fifo_size = 64;

        Vector3 position; // Position of the tracker.
        Vector3 velocity; // Speed of the tracker.
        Vector3 rotation; // rotation is in RADIANS.
        Quaternion quat; // a quaternion only has 4 values so therefore vec4

        /// These directions will end up having 1 on a specific axis for their respective directions, the rest 0.
        /// To make these directions useful you can do multiplication or check which direction the IMU is facing with these directions.
        Vector3 UpDir; // Format is XZY.
        Vector3 DownDir; // Format is XZY.
        Vector3 LeftDir; // Format is XZY.
        Vector3 RightDir; // Format is XZY.
        Vector3 FrontDir; // Format is XZY.
        Vector3 BackDir; // Format is XZY.

        Vector3 position_threshold; // Threshold of the position.
        Vector3 acceleration_threshold; // Threshold of the acceleration.
        Vector3 gyro_threshold; // Threshold of the gyro.
        Vector3 velocity_threshold; // Threshold of the velocity.

        /// TODO: have a dynamic filter, although
        /// instead of implementing these filters in the drivers,
        /// implement them in here,
        /// but you'd have to constantly rewrite functions for different VQF
        /// filters, but that is code. you cannot have it the easy way here.

        VQFParams vqf_parameters;
        VQF vqf_filter = VQF(vqf_parameters, 0.01);
        //VQF vqf_filter;
        /// @brief Initializes the VQF filter.
        //void VQF_init() {
        //    vqf_filter = VQF(vqf_parameters, 0.01);
            //vqf_init();
        //}

        /// @brief Updates the VQF filter, and updates `quat` (quaternion).
        void VQF_update() {
            // TODO: replace the VQF library EDIT: done
            vqf_real_t vqfgyr[3] = {gyro.x(), gyro.y(), gyro.z()};
            vqf_real_t vqfacc[3] = {acceleration.x(), acceleration.y(), acceleration.z()};
            vqf_real_t vqfquat[4] = {0,0,0,0};
            vqf_filter.update(vqfgyr, vqfacc);
            vqf_filter.getQuat6D(vqfquat);
            //logger.print("VQF Quat: %f %f %f %f", vqfquat[0], vqfquat[1], vqfquat[2], vqfquat[3]);
            //quat = {vqfquat[0], vqfquat[1], vqfquat[2], vqfquat[3]}; -- for gmath
            quat = Quaternion(vqfquat[3], vqfquat[0], vqfquat[1], vqfquat[2]); // -- for eigen
        }
        /// @brief Initializes the IMU for usage.
        /// Also initializes other things like VQF if enabled. Returns 'true' if working.
        virtual bool imu_init(){ return false; };

        /// @brief Updates acceleration and gyro.
        virtual void update(){};

        /// @brief This should apply any configurations given to this struct to the IMU.
        void configure();

        double PEF_dt;
        // code to use the kalman filter
        #if USE_POSITION_ESTIMATION_FILTER 
            #include "../deadreckoning.h"
            /// TODO: make this apart of calibration instead of hard-coded.
            DeadReckoning position_estimation_filter;
            IMUObj(){
                Eigen::Vector3f initialAccel(0.0f, 0.0f, 9.81f); 
                position_estimation_filter.initialize(initialAccel,micros());
                PEF_dt=get_hrtz_ms();
            };
            /// @brief Call this after VQF update.
            void position_estimation() {
                //Eigen::Vector3d acc_global = to_global_frame(acceleration.cast<double>(), quat.cast<double>());
                //acc_global[2] -= CONST_EARTH_GRAVITY;

                //position_estimation_filter.predict(PEF_dt);
                //position_estimation_filter.update(acc_global);
                //Eigen::Vector3d PEF_position=position_estimation_filter.get_position();
                //Eigen::Vector3d PEF_velocity=position_estimation_filter.get_velocity();
                //position=PEF_position.cast<float>();
                //velocity=PEF_velocity.cast<float>();
                position_estimation_filter.update(acceleration,gyro, micros());
                position=position_estimation_filter.getPosition();
                quat = position_estimation_filter.getOrientation();
                velocity=position_estimation_filter.getVelocity();
                //position=position_estimation_filter.get_position().cast<float>();
                //velocity=position_estimation_filter.get_velocity().cast<float>();
                //auto PEF_position=position_estimation_filter.get_position();
                //auto PEF_velocity=position_estimation_filter.get_velocity();
                //position=Vector3(PEF_position.x(),PEF_position.y(),PEF_position.z());
                //velocity=Vector3(PEF_velocity.x(),PEF_velocity.y(),PEF_velocity.z());
            };
        #endif
        //virtual ~IMUBase() = default;
    };
}
#endif
