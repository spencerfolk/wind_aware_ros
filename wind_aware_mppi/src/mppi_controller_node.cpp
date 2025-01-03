#include "wind_aware_mppi/mppi_controller_node.hpp"

MppiNode::MppiNode(std::string ns) : nh_(ns)
{
    /*
        Constructor
    */

   // Config variables. 
   nh_.param("robot_name", robot_name_, std::string("robot"));

   // Init subscribers. 
   robot_pose_sub_ = nh_.subscribe("pose", 1, &MppiNode::robotPoseCallback, this);
   lidar_sub_ = nh_.subscribe("lidar_raycast", 1, &MppiNode::lidarCallback, this);

   // Init the publisher. 
   cmd_acc_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("mppi_cmd_acc", 1);

}

void MppiNode::robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /*
        Every time motion capture gives a new measurement of the robot's pose, save it as the current pose of the robot.
    */

    robot_pos_.x() = msg->pose.position.x;
    robot_pos_.y() = msg->pose.position.y;
    robot_pos_.z() = msg->pose.position.z;

    robot_quat_.x() = msg->pose.orientation.x;
    robot_quat_.y() = msg->pose.orientation.y;
    robot_quat_.z() = msg->pose.orientation.z;
    robot_quat_.w() = msg->pose.orientation.w;

}

void MppiNode::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    /*
        Every time the lidar produces a new scan, save the point cloud data for the controller. 
    */

    ROS_INFO_STREAM("Received LiDAR data...");

}

void MppiNode::runControl()
{
    /*
        Run a control loop for the MPPI controller. 
    */

    // Run the control loop by feeding in the current pose of vehicle and the current time.
    cmd_acc_ << robot_pos_.x(), robot_pos_.y(), robot_pos_.z(); // Temporarily report robot position. Replace with controller when implemented.

    // Publish the commanded acceleration returned by the controller. 
    publishCmdAcc();

}

void MppiNode::publishCmdAcc()
{
    /*
        Publish the commanded acceleration vector from the MPPI controller. 
    */

    geometry_msgs::Vector3Stamped cmd_acc_msg;

    // Header
    cmd_acc_msg.header.stamp = ros::Time::now();
    cmd_acc_msg.header.frame_id = "mocap";

    // Set vector. 
    cmd_acc_msg.vector.x = cmd_acc_.x();
    cmd_acc_msg.vector.y = cmd_acc_.y();
    cmd_acc_msg.vector.z = cmd_acc_.z();

    // Publish the message
    cmd_acc_pub_.publish(cmd_acc_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mppi_node");
    ros::NodeHandle nh; // Create a NodeHandle to access parameters and namespaces
    std::string ns = nh.getNamespace(); // Get namespace of the node

    // Create MPPI controller node object.
    MppiNode mppi_node(ns);

    ros::Rate rate(50);

    while(ros::ok()  ){
        // Run a single loop of the MPPI controller. 
        mppi_node.runControl();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}