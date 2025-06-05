#include <wind_aware_ukf/wind_ukf.hpp>

WindUKF::WindUKF(
    double dt,
    double mass,
    double g,
    double keta,
    double kd,
    double kz,
    double kh
    ) : ukf_t(17, 17), 
    dt_(dt), mass_(mass), g_(g), keta_(keta), kd_(kd), kz_(kz), kh_(kh)
{
    // Initialize UKF parameters
    // ukf.Q = Eigen::MatrixXd::Identity(16, 16) * 0.1; // Process noise covariance
    // ukf.R = Eigen::MatrixXd::Identity(16, 16) * 0.1; // Measurement noise covariance
    // ukf.P = Eigen::MatrixXd::Identity(16, 16) * 1.0; // Initial state covariance

    // Compute the drag matrix from the coefficients given in the initializer. 
    K_.setZero();
    K_(0, 0) = kd_;
    K_(1, 1) = kd_;
    K_(2, 2) = kz_;
    
}

void WindUKF::state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const 
{
    /*
    Process model. Use Euler integration with class dt to predict the next state. 
    TODO: Consider higher order integration schemes!
    */

    x = xp; // Start with current state

    // Unpack state
    Eigen::Vector4d motor_speeds = xp.segment<4>(0) * 1e3; // TODO: get rid of hardcoded 1e3 normalizer. 
    double psi   = xp(4);   // yaw angle (rad)
    double theta = xp(5);   // pitch angle (rad)
    double phi   = xp(6);   // roll angle (rad)
    double p     = xp(7);   // roll rate (rad/s)
    double q     = xp(8);   // pitch rate (rad/s)
    double r     = xp(9);   // yaw rate (rad/s)
    
    Eigen::Vector3d body_velocity = xp.segment<3>(10);
    double vx = body_velocity(0); // x ground velocity (in body frame)
    double vy = body_velocity(1); // y ground velocity (in body frame)
    double vz = body_velocity(2); // z ground velocity (in body frame)

    Eigen::Vector3d wind_velocity = xp.segment<3>(13);
    double thrust_coeff_norm = xp(16); // thrust coefficient (normalized to the order of 10^0)

    // Compute airspeed vector by subtracting body velocity from wind velocity.
    Eigen::Vector3d va = body_velocity - wind_velocity;
    double vh_squared = va(0) * va(0) + va(1) * va(1);

    // Sum and sum-of-squares of motor speeds for thrust and aero computations. 
    double motor_speed_sum = motor_speeds.sum();
    double motor_speed_sos = motor_speeds.squaredNorm();

    // Compute forces in the body frame -- rotor drag, gravity, thrust, and translational lift. 
    double Fx = -kd_ * motor_speed_sum * va(0) + mass_ * g_ * std::sin(theta);
    double Fy = -kd_ * motor_speed_sum * va(1) - mass_ * g_ * std::cos(theta) * std::sin(phi);
    double Fz = -kz_ * motor_speed_sum * va(2) - mass_ * g_ * std::cos(theta) * std::cos(phi)
                + (thrust_coeff_norm * 1e-8) * motor_speed_sos + 4 * kh_ * vh_squared; // TODO: Get rid of hard coded 1e-8 normalizer

    // xdot vector
    Eigen::VectorXd xdot(17);
    xdot.setZero();

    // Motor speeds dot = 0 (TODO: model motor dynamics if needed).

    // Rotational kinematics
    xdot(4) = std::sin(phi) / std::cos(theta) * q + std::cos(phi) / std::cos(theta) * r;        // psi dot
    xdot(5) = std::cos(phi) * q - std::sin(phi) * r;                                            // theta dot
    xdot(6) = p + std::sin(phi) * std::tan(theta) * q + std::cos(phi) * std::tan(theta) * r;    // phi dot

    // Angular rates dot = 0 (TODO: model rotational dynamics).
    
    // Translational acceleration (body frame).
    xdot(10) = Fx / mass_ + r * vy - q * vz; // vx dot (in body frame)
    xdot(11) = Fy / mass_ + p * vz - r * vx; // vy dot (in body frame)
    xdot(12) = Fz / mass_ + q * vx - p * vy; // vz dot (in body frame)

    // Wind and thrust coefficient dynamics = 0.

    // Euler forward integration.
    x = xp + dt_ * xdot;
}


void WindUKF::observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const 
{
    /*
    Measurement model: predict the measurement z from the current belief of the filter state x.
    */

    z = Eigen::VectorXd::Zero(17);

    // Get current motor speeds and compute sum and sum of squares for aero computations. 
    Eigen::Vector4d motor_speeds = x.segment<4>(0) * 1e3;
    double motor_speed_sum = motor_speeds.sum();
    double motor_speed_sos = motor_speeds.squaredNorm();

    // Get the airspeed vector.
    Eigen::Vector3d va = x.segment<3>(10) - x.segment<3>(13);  // body_velocity - wind_velocity
    double vh_squared = va(0)*va(0) + va(1)*va(1);

    // Copy the first 13 states directly.
    z.segment<13>(0) = x.segment<13>(0);

    // Compute accelerations (accel_x, accel_y, accel_z).
    Eigen::Vector3d accel_body = (1.0 / mass_) * (
        ((x(16) * 1e-8) * motor_speed_sos + 4 * kh_ * vh_squared) * Eigen::Vector3d::UnitZ()
        - motor_speed_sum * K_ * va
    );
    z.segment<3>(13) = accel_body;

    // Compute commanded thrust.
    z(16) = (x(16) * 1e-8) * motor_speed_sos; // TODO: Get rid of the 1e-8 thrust normalizer!
}