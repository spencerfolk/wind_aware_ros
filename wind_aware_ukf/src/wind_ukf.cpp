#include <wind_aware_ukf/wind_ukf.hpp>

WindUKF::WindUKF() 
{
    // Initialize UKF parameters
    // ukf.Q = Eigen::MatrixXd::Identity(16, 16) * 0.1; // Process noise covariance
    // ukf.R = Eigen::MatrixXd::Identity(16, 16) * 0.1; // Measurement noise covariance
    // ukf.P = Eigen::MatrixXd::Identity(16, 16) * 1.0; // Initial state covariance
    
}

void WindUKF::predict() 
{
    // ukf.iterate();
}

void WindUKF::update(const Eigen::VectorXd& measurement) 
{
    // // Update UKF with new measurement
    // ukf.new_observation(0, measurement);
    // ukf.iterate();
}