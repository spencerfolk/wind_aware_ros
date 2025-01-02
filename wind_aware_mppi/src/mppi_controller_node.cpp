#include "wind_aware_mppi/mppi_controller_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mppi_node");
    ros::NodeHandle nh; // Create a NodeHandle to access parameters and namespaces
    std::string ns = nh.getNamespace(); // Get namespace of the node

    // Create MPPI controller node object.

    ros::Rate rate(50);

    while(ros::ok()  ){
        // Publish wind measurement. 
        ROS_INFO_STREAM("Wind Aware MPPI: Testing.");
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}