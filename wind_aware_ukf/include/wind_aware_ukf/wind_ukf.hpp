#ifndef WIND_UKF_HPP
#define WIND_UKF_HPP

#include <kalman_filter/ukf.hpp>
#include <Eigen/Dense>

class WindUKF 
{
  
public:

  WindUKF();

  void predict();
  void update(const Eigen::VectorXd& measurement);
  
};

#endif // WIND_UKF_HPP