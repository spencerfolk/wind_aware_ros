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
    double kd,
    double kz,
    double kh
  );

private:

  void state_transition(const Eigen::VectorXd& xp, Eigen::VectorXd& x) const override;
  void observation(const Eigen::VectorXd& x, Eigen::VectorXd& z) const override;

  double dt_;   // timestep of the filter (determined by the update frequency of the filter upstream)
  double mass_; // UAV mass
  double g_;    // gravity constant
  double kd_;   // rotor drag constant
  double kz_;   // loss of thrust coeff (appears as drag in z axis)
  double kh_;   // translational lift coefficient
  Eigen::Matrix3d K_;

};

#endif // WIND_UKF_HPP