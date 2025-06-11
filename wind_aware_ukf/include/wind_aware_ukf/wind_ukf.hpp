#ifndef WIND_UKF_HPP
#define WIND_UKF_HPP

#include <kalman_filter/ukf.hpp>
#include <Eigen/Dense>

class WindUKF : public kalman_filter::ukf_t
{
  
public:

  WindUKF(
    double dt,
    double mass,
    double g,
    double keta,
    double kd,
    double kz,
    double kh,
    double tau_m
  );

  double dt_;     // timestep of the filter (determined by the update frequency of the filter upstream)
  double mass_;   // UAV mass
  double g_;      // gravity constant
  double keta_;   // thrust coefficient
  double kd_;     // rotor drag constant
  double kz_;     // loss of thrust coeff (appears as drag in z axis)
  double kh_;     // translational lift coefficient
  double tau_m_;  // motor time constant
  Eigen::Matrix3d K_;

  Eigen::Vector4d cmd_motor_speeds_;
  void set_cmd_motor_speeds(Eigen::Vector4d& cmd_motor_speeds);

private:

  void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;
  void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override;

};

#endif // WIND_UKF_HPP