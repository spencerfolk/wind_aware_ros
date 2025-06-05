#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <cmath>
#include <algorithm>

enum class MotorSpeedUnits {
    RPM,
    RAD_PER_SEC
};

// Converts PWM values to motor speeds (rad/s or rpm) using a polynomial mapping
// Defaults taken from thrust stand testing of the CFBL motor/prop at 3.7V
inline Eigen::VectorXd pwmToMotorSpeeds(const Eigen::VectorXd& pwm,
                                        const Eigen::Vector2d& coeffs = Eigen::Vector2d(0.51212, -1719),
                                     MotorSpeedUnits output = MotorSpeedUnits::RAD_PER_SEC) {
    Eigen::VectorXd rpm = coeffs[0] * pwm.array() + coeffs[1];

    if (output == MotorSpeedUnits::RPM) {
        return rpm;
    } else if (output == MotorSpeedUnits::RAD_PER_SEC) {
        return rpm * 0.10472;  // Conversion from RPM to rad/s
    } else {
        throw std::invalid_argument("Motor speed mapping: must specify either 'rpm' or 'rad/s'.");
    }
}

// Converts PWM values to motor speeds with battery compensation
// Default values from thrust stand testing of the CFBL motor/props at a range of voltages.
inline Eigen::VectorXd pwmToMotorSpeedsBatCompensated(const Eigen::VectorXd& pwm_input,
                                                      double vbat,
                                                      const Eigen::Vector3d& coeffs = Eigen::Vector3d(4.3034, 0.759, 10000),
                                                     MotorSpeedUnits output = MotorSpeedUnits::RAD_PER_SEC) {
    Eigen::VectorXd pwm = pwm_input.array().max(coeffs[2]); // Clip to ESC deadband

    Eigen::VectorXd rpm = coeffs[0] * (vbat + coeffs[1]) * ((pwm.array() - coeffs[2]).pow(2.0 / 3.0));

    if (output == MotorSpeedUnits::RPM) {
        return rpm;
    } else if (output == MotorSpeedUnits::RAD_PER_SEC) {
        return rpm * 0.10472;  // Conversion from RPM to rad/s
    } else {
        throw std::invalid_argument("Motor speed mapping: must specify either 'rpm' or 'rad/s'.");
    }
}