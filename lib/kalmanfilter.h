#include "ArduinoEigen.h"

// Fixed-size type aliases
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 3, 12> Matrix3x12d;
typedef Eigen::Matrix<double, 12, 3> Matrix12x3d;

class IMUKalmanFilter {
public:
    IMUKalmanFilter() {
        // Initialize with proper fixed-size matrices
        x_.setZero();
        P_ = Matrix12d::Identity() * 0.1;
        
        // Configure measurement matrix (3x12)
        H_.setZero();
        H_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();  // Acceleration part
        H_.block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();   // Bias part
        
        Q_ = Matrix12d::Identity() * 0.01;    // Process noise
        R_ = Eigen::Matrix3d::Identity() * 0.1; // Measurement noise
    }

    void initialize(const Eigen::Vector3d& initial_position, const Eigen::Vector3d& initial_acc_bias) {
        x_.segment<3>(0) = initial_position;
        x_.segment<3>(9) = initial_acc_bias;
    }

    void predict(double dt) {
        update_transition_matrix(dt); // Update F_ in-place
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const Eigen::Vector3d& linear_acceleration) {
        S_ = H_ * P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S_.inverse();
        y_ = linear_acceleration - H_ * x_;
        x_ += K_ * y_;
        P_ = (Matrix12d::Identity() - K_ * H_) * P_;
    }

    Eigen::Vector3d get_position() const { return x_.segment<3>(0); }
    Eigen::Vector3d get_velocity() const { return x_.segment<3>(3); }
    Eigen::Vector3d get_acceleration() const { return x_.segment<3>(6); }
    Eigen::Vector3d get_acc_bias() const { return x_.segment<3>(9); }

private:
    Vector12d x_;        // State
    Matrix12d P_;        // Covariance
    Matrix3x12d H_;      // Measurement
    Matrix12d Q_;        // Process noise
    Eigen::Matrix3d R_;  // Measurement noise

    // Reusable buffers
    Matrix12d F_;        // State transition
    Eigen::Matrix3d S_;  // Innovation
    Matrix12x3d K_;      // Kalman gain
    Eigen::Vector3d y_;  // Residual

    void update_transition_matrix(double dt) {
        F_.setIdentity();
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * 0.5 * dt * dt;
        F_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    }
};

// Helper function with fixed-size types
Eigen::Vector3d to_global_frame(const Eigen::Vector3d& acc_body, const Eigen::Quaterniond& orientation) {
    return orientation * acc_body;
};