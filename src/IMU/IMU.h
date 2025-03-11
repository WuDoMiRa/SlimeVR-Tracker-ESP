#ifndef IMUOBJECT_H
#define IMUOBJECT_H
#include <Wire.h>
#include <Arduino.h>
#include "../GlobalVars.h"
#include "../globals.h"
#include <memory>
#include "../gmath/src/Quaternion.hpp"
#include "../gmath/src/Vector3.hpp"
#include "../vqf/vqf/cpp/vqf.hpp"
#include "../3DKalmanFilter/src/KalmanFilter3D.h"

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
        Vector3 velocity;
        Vector3 rotation; // rotation is in RADIANS.
        Quaternion quat; // a quaternion only has 4 values so therefore vec4

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

        /// @brief Position estimation. TODO: come back later to possibly initialize one of these parameters when doing calibration or something.
        KalmanFilter3D position_estimation_filter = KalmanFilter3D(0.01, 0.5, 0.1);

        /// @brief Updates the VQF filter, and updates quaternion.
        void VQF_update() {
            // TODO: replace the VQF library EDIT: done
            vqf_real_t vqfgyr[3] = {gyro.X, gyro.Y, gyro.Z};
            vqf_real_t vqfacc[3] = {acceleration.X, acceleration.Y, acceleration.Z};
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

        /// @brief This should apply any configurations given to this struct to the IMU.
        void configure();

        
        Vector3 rotateVectorByQuaternion(const Quaternion& q, const Vector3& v) {
            // Implement quaternion vector rotation manually
            Vector3 q_vec(q.X, q.Y, q.Z);
            float q_w = q.W;
            
            Vector3 uv(q_vec.Y*v.Z - q_vec.Z*v.Y,
                    q_vec.Z*v.X - q_vec.X*v.Z,
                    q_vec.X*v.Y - q_vec.Y*v.X);
                    
            Vector3 uuv(q_vec.Y*uv.Z - q_vec.Z*uv.Y,
                        q_vec.Z*uv.X - q_vec.X*uv.Z,
                        q_vec.X*uv.Y - q_vec.Y*uv.X);

            return Vector3(
                v.X + 2.0f*(q_w*uv.X + uuv.X),
                v.Y + 2.0f*(q_w*uv.Y + uuv.Y),
                v.Z + 2.0f*(q_w*uv.Z + uuv.Z)
            );
        }
        double vectorLength(const Vector3& v) {
            return std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }
        
        /// @brief This will estimate the position of the IMU based off of the calibration data, and acceleration data..
        void position_estimation() {
            constexpr float GRAVITY = 9.80665f;
            static uint64_t last_update = 0;
            
            // Calculate dt
            uint64_t now = micros();
            //float dt = (last_update > 0) ? (now - last_update) / 1e6f : 0;
            //last_update = now;
            //if(dt <= 0) dt = 0.01f;  // Default to 100Hz
        
            // Get orientation-compensated acceleration
            Quaternion q = quat.Normalized(quat);
            Vector3 accel_global = rotateVectorByQuaternion(q, acceleration);
            
            // Improved gravity removal with tilt compensation
            Vector3 gravity = Vector3(0, 0, GRAVITY);
            Vector3 linear_accel = accel_global - gravity;
        
            // Kalman prediction with proper acceleration integration
            double accel_arr[3] = {linear_accel.X, linear_accel.Y, linear_accel.Z};
            position_estimation_filter.predict(accel_arr);  // Modified predict()
        
            // Enhanced ZUPT with velocity measurement
            if(isStationary()) {
                double zero_vel[3] = {0, 0, 0};
                position_estimation_filter.update(zero_vel, true);  // Velocity measurement
            }
        
            // Get state with stability checks
            double state[6];
            position_estimation_filter.getState(state);
            position = Vector3(state[0], state[1], state[2]);
            velocity = Vector3(state[3], state[4], state[5]);
        
            // Reset on excessive drift
            if(position.Z < -5.0) {  // Threshold based on your application
                position_estimation_filter.resetState();
            }
        };

        
        /// @brief ZUPT implementation, suggestion by Deepseek.
        bool isStationary() {
            constexpr float GYRO_THRESH = 0.15f;   // ±15°/s
            constexpr float ACCEL_THRESH = 0.3f;   // ±0.3 m/s²
            constexpr float GRAVITY = 9.80665f;
            
            return (vectorLength(gyro) < GYRO_THRESH) && 
                   (abs(vectorLength(acceleration) - GRAVITY) < ACCEL_THRESH);
        }
    };
}
#endif