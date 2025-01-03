#ifndef MPPI_CONTROLLER_NODE_HPP
#define MPPI_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

class MppiNode
{

public:

    MppiNode(std::string ns);

    void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void runControl();
    void publishCmdAcc();

private:

    // Robot parameters. 
    std::string robot_name_; // Robot name for motion capture callback. 

    // Controller parameters. 
    Eigen::Vector3f cmd_acc_;   // Output commanded acceleration.
    bool filter_control_;
    bool use_traj_horizon_;
    bool project_goal_;
    bool normalize_costs_;
    bool use_wind_;
    bool adapt_goal_;
    bool twod_;

    // ROS and robot variables. 
    ros::NodeHandle nh_;

    ros::Subscriber robot_pose_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher cmd_acc_pub_;

    Eigen::Vector3f robot_pos_;
    Eigen::Quaternionf robot_quat_;

};


#endif // MPPI_CONTROLLER_NODE_HPP