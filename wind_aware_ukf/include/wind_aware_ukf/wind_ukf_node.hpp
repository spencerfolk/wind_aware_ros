#ifndef WIND_UKF_NODE_HPP
#define WIND_UKF_NODE_HPP 

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <kr_mav_msgs/SO3Command.h>
#include <crazyflie_driver/GenericLogData.h>
#include <wind_aware_ukf/WindEstimateFullState.h>

#include <Eigen/Dense>

#include <wind_aware_ukf/wind_ukf_node.hpp>
#include <wind_aware_ukf/wind_ukf.hpp>
#include <wind_aware_ukf/motor_utils.hpp>

class WindEstimatorNode
{
public:

    WindEstimatorNode(std::string ns);    
    bool ready_;

    // Sensor Callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void so3cmdCallback(const kr_mav_msgs::SO3Command::ConstPtr& msg);
    void motorpwmCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg);
    void vbatCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg);

    void publishWindEstimate(const ros::TimerEvent&);
    
private:

    // Configuration
    std::string mav_name_;  // Robot name for the motion capture callback. 

    // Measurement arrays
    Eigen::Vector3d ground_velocity_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d linear_acceleration_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector4d motor_pwms_;
    double vbat_;
    double cmd_thrust_;

    // ROS and robot variables
    ros::NodeHandle nh_;
    
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber so3cmd_sub_;
    ros::Subscriber motorpwm_sub_;
    ros::Subscriber vbat_sub_;

    ros::Publisher wind_estimate_pub_;

    // tf variables
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;
    // tf2::Transform robot_tf_;

    // Wind estimator object. 
    std::shared_ptr<WindUKF> estimator_;
};

#endif //WIND_UKF_NODE_HPP