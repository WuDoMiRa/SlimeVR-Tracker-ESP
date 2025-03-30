#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "ArduinoEigen.h"

class DeadReckoning {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    DeadReckoning(float gravity = 9.81f) : 
        m_gravity(Eigen::Vector3f(0, 0, gravity)),
        m_position(Eigen::Vector3f::Zero()),
        m_velocity(Eigen::Vector3f::Zero()),
        m_orientation(Eigen::Quaternionf::Identity()),
        m_accelBias(Eigen::Vector3f::Zero()),
        m_gyroBias(Eigen::Vector3f::Zero()),
        m_prevTimeUs(0) {}

    void initialize(const Eigen::Vector3f& initialAccel, uint64_t timestampUs) {
        // Assume initial orientation from accelerometer (gravity vector)
        Eigen::Vector3f up = -initialAccel.normalized();
        Eigen::Vector3f east = up.cross(Eigen::Vector3f::UnitY()).normalized();
        Eigen::Vector3f north = up.cross(east);
        m_orientation = Eigen::Quaternionf().setFromTwoVectors(Eigen::Vector3f::UnitZ(), up);
        
        m_prevTimeUs = timestampUs;
    }

    void update(const Eigen::Vector3f& accel, 
                const Eigen::Vector3f& gyro,
                uint64_t timestampUs) {
        // Calculate time delta in seconds
        float dt = (timestampUs - m_prevTimeUs) / 1e6f;
        if(dt <= 0 || dt > 0.1f) {  // Handle invalid/initial delta
            dt = 0.01f;  // Default to 100Hz
        }
        
        // 1. Update orientation using gyro (integrating angular velocity)
        Eigen::Vector3f angularVelocity = gyro - m_gyroBias;
        Eigen::Quaternionf deltaQ;
        Eigen::Vector3f deltaAngle = angularVelocity * dt;
        deltaQ.w() = 1.0f - 0.125f * deltaAngle.squaredNorm();
        deltaQ.vec() = 0.5f * deltaAngle;
        m_orientation = (m_orientation * deltaQ).normalized();

        // 2. Rotate acceleration to world frame
        Eigen::Vector3f accelWorld = m_orientation * (accel - m_accelBias);

        // 3. Remove gravity and integrate
        Eigen::Vector3f linearAccel = accelWorld - m_gravity;
        m_velocity += linearAccel * dt;
        m_position += m_velocity * dt + 0.5f * linearAccel * dt * dt;

        // Store time for next update
        m_prevTimeUs = timestampUs;
    }

    void resetPose() {
        m_position.setZero();
        m_velocity.setZero();
    }

    void calibrateBiases(const Eigen::Vector3f& accelBias, 
                        const Eigen::Vector3f& gyroBias) {
        m_accelBias = accelBias;
        m_gyroBias = gyroBias;
    }

    // Getters
    Eigen::Vector3f getPosition() const { return m_position; }
    Eigen::Vector3f getVelocity() const { return m_velocity; }
    Eigen::Quaternionf getOrientation() const { return m_orientation; }

private:
    Eigen::Vector3f m_gravity;
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_velocity;
    Eigen::Quaternionf m_orientation;
    Eigen::Vector3f m_accelBias;
    Eigen::Vector3f m_gyroBias;
    uint64_t m_prevTimeUs;
};

#endif // DEAD_RECKONING_H