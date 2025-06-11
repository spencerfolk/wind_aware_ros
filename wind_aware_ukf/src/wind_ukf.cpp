#include <wind_aware_ukf/wind_ukf.hpp>

WindUKF::WindUKF(
    double dt,
    double mass,
    double g,
    double keta,
    double kd,
    double kz,
    double kh,
    double tau_m
    ) : ukf_t(17, 13), 
    dt_(dt), mass_(mass), g_(g), keta_(keta), kd_(kd), kz_(kz), kh_(kh), tau_m_(tau_m)
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

    cmd_motor_speeds_.setZero();
    
}

void WindUKF::set_cmd_motor_speeds(Eigen::Vector4d& cmd_motor_speeds)
{
    cmd_motor_speeds_ = cmd_motor_speeds;
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
    double wx = wind_velocity(0);
    double wy = wind_velocity(1);
    double wz = wind_velocity(2);

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
    Eigen::VectorXd xdot(20);
    xdot.setZero();

    // Motor speeds dot = 0 (TODO: model motor dynamics if needed).
    xdot.segment<4>(0) = (cmd_motor_speeds_ - motor_speeds) / (1e3 * tau_m_);

    // Rotational kinematics
    xdot(4) = std::sin(phi) / std::cos(theta) * q + std::cos(phi) / std::cos(theta) * r;        // psi dot
    xdot(5) = std::cos(phi) * q - std::sin(phi) * r;                                            // theta dot
    xdot(6) = p + std::sin(phi) * std::tan(theta) * q + std::cos(phi) * std::tan(theta) * r;    // phi dot

    // Angular rates dot = 0 (TODO: model rotational dynamics).
    
    // Translational acceleration (body frame).
    xdot(10) = Fx / mass_ - (q * vz - r * vy); // vx dot (in body frame)
    xdot(11) = Fy / mass_ - (r * vx - p * vz); // vy dot (in body frame)
    xdot(12) = Fz / mass_ - (p * vy - q * vx); // vz dot (in body frame)

    // Wind dynamics in the body frame
    xdot(13) = - (q * wz - r * wy); // v_wx dot
    xdot(14) = - (r * wx - p * wz); // v_wy dot
    xdot(15) = - (p * wy - q * wx); // v_wz dot

    // Euler forward integration.
    x = xp + dt_ * xdot;
}


void WindUKF::observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const 
{
    /*
    Measurement model: predict the measurement z from the current belief of the filter state x.
    */

    z = Eigen::VectorXd::Zero(13);

    // Extract Euler angles from the state
    double psi   = x(4);  // yaw
    double theta = x(5);  // pitch
    double phi   = x(6);  // roll

    // Compute rotation matrix from body to world (ZYX convention: R = Rz * Ry * Rx)
    Eigen::Matrix3d R_bw;
    R_bw = Eigen::AngleAxisd(psi,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(phi,   Eigen::Vector3d::UnitX());

    // Define gravity in world frame
    Eigen::Vector3d gravity_world(0, 0, g_);

    // Get current motor speeds and compute sum and sum of squares for aero computations. 
    Eigen::Vector4d motor_speeds = x.segment<4>(0) * 1e3;
    double motor_speed_sum = motor_speeds.sum();
    double motor_speed_sos = motor_speeds.squaredNorm();

    // Get the airspeed vector.
    Eigen::Vector3d va = x.segment<3>(10) - x.segment<3>(13);  // body_velocity - wind_velocity
    double vh_squared = va(0)*va(0) + va(1)*va(1);

    // Copy the last 9 states
    z.segment<9>(0) = x.segment<9>(4);

    // Compute accelerations (accel_x, accel_y, accel_z).
    Eigen::Vector3d accel_body = (1.0 / mass_) * (
        ((x(16) * 1e-8) * motor_speed_sos + 4 * kh_ * vh_squared) * Eigen::Vector3d::UnitZ()
        - motor_speed_sum * K_ * va
    );
    z.segment<3>(9) = accel_body; // - R_bw.transpose() * gravity_world;

    // Compute commanded thrust.
    z(12) = (x(16) * 1e-8) * motor_speed_sos / mass_; // TODO: Get rid of the 1e-8 thrust normalizer!
}