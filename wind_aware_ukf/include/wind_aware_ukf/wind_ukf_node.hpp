#ifndef WIND_UKF_NODE_HPP
#define WIND_UKF_NODE_HPP 

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <kr_mav_msgs/SO3Command.h>
#include <crazyflie_driver/GenericLogData.h>
#include <wind_aware_ukf/WindEstimateStamped.h>

#include <Eigen/Dense>

#include <wind_aware_ukf/wind_ukf_node.hpp>
#include <wind_aware_ukf/wind_ukf.hpp>
#include <wind_aware_ukf/motor_utils.hpp>

Eigen::MatrixXd loadDiagonalMatrix(ros::NodeHandle& nh, const std::string& param_name, int size) { // Helper function for loading diagonal elements from ROS params. 
    std::vector<double> diag;
    if (!nh.getParam(param_name, diag) || diag.size() != size) {
        throw std::runtime_error("Missing or invalid " + param_name + " param (expected " + std::to_string(size) + " elements)");
    }

    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(size, size);
    for (int i = 0; i < size; ++i) {
        mat(i, i) = diag[i];
    }
    return mat;
}

class WindEstimatorNode
{
public:

    WindEstimatorNode(std::string ns, double dt, double mass, double g, double keta, double kd, double kz, double kh, double tau_m);    

    // Sensor Callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void so3cmdCallback(const kr_mav_msgs::SO3Command::ConstPtr& msg);
    void motorpwmCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg);
    void inaVbatCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg);
    void crazyflieVbatCallback(const std_msgs::Float32::ConstPtr& msg);

    void run(const ros::TimerEvent&);
    void initializeFilter();
    void publishWindEstimate();
    void publishWindVector();
    void publishImuVector();
    
private:

    // Configuration
    std::string mav_name_;  // Robot name for the motion capture callback. 

    // State machine
    bool filter_initialized_;
    bool imu_received_;
    bool odom_received_;
    bool so3cmd_received_;
    bool motorpwm_received_;
    bool vbat_received_;

    // Measurement arrays
    Eigen::Vector3d ground_velocity_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d linear_acceleration_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector4d cmd_motor_pwms_;
    Eigen::Vector4d cmd_motor_speeds_;
    double vbat_;
    double cmd_thrust_;

    // Acceleration Estimation from Motion Capture
    Eigen::Vector3d prev_odom_acceleration_;
    Eigen::Vector3d odom_acceleration_;
    Eigen::Vector3d prev_ground_velocity_;
    ros::Time prev_velocity_time_;

    // ROS and robot variables
    ros::NodeHandle nh_;
    
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber so3cmd_sub_;
    ros::Subscriber motorpwm_sub_;
    ros::Subscriber vbat_sub_;

    ros::Publisher wind_estimate_pub_;
    ros::Publisher wind_estimate_marker_pub_;
    ros::Publisher accel_vector_pub_;
    ros::Publisher accel_vector_marker_pub_;

    // tf variables
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
    // tf2::Transform robot_tf_;

    // Wind estimator object. 
    // std::shared_ptr<WindUKF> estimator_;
    WindUKF estimator_;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::VectorXd x0;
    Eigen::MatrixXd P0;

};

#endif //WIND_UKF_NODE_HPP