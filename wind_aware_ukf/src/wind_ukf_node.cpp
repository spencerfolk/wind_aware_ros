#include "wind_aware_ukf/wind_ukf_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_ukf_node");
    ros::NodeHandle nh; // Create a NodeHandle to access parameters and namespaces
    std::string ns = nh.getNamespace(); // Get namespace of the node

    // Create wind UKF node object.

    ros::Rate rate(50);

    while(ros::ok()  ){
        // Publish wind measurement. 
        ROS_INFO_STREAM("Wind Aware UKF: Testing.");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}