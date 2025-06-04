#include <wind_aware_ukf/wind_ukf_node.hpp>

WindEstimatorNode::WindEstimatorNode(std::string ns): nh_(ns), ready_(false)
{
    /*
    Constructor
    */

    // Retrieve rosparams
    nh_.param("mav_name", mav_name_, std::string("robot"));
    
    // Initialize ROS node and subscribers
    imu_sub_ = nh_.subscribe("imu", 1, &WindEstimatorNode::imuCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &WindEstimatorNode::odomCallback, this);
    motorpwm_sub_ = nh_.subscribe("motor_pwms", 1, &WindEstimatorNode::motorpwmCallback, this);
    vbat_sub_ = nh_.subscribe("ina_voltage", 1, &WindEstimatorNode::vbatCallback, this);
    so3cmd_sub_ = nh_.subscribe("so3_cmd", 1, &WindEstimatorNode::so3cmdCallback, this);
    wind_estimate_pub_ = nh_.advertise<wind_aware_ukf::WindEstimateFullState>("wind_estimate", 1);

    // // Initialize WindEstimator
    estimator_ = std::make_shared<WindUKF>();

}

void WindEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    /*
    Process IMU data coming from the Crazyflie 
    */

    linear_acceleration_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    angular_velocity_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    // ROS_INFO_STREAM("imu_acc: " << linear_acceleration_.x() << "\t" << linear_acceleration_.y() << "\t" << linear_acceleration_.z());
}

void WindEstimatorNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    /*
    Process odometry which contains the ground velocity estimate.
    */
    // // Fill odom_data with relevant information from msg
    // estimator->update(odom_data);

    ground_velocity_.x() = msg->twist.twist.linear.x;
    ground_velocity_.y() = msg->twist.twist.linear.y;
    ground_velocity_.z() = msg->twist.twist.linear.z;
    orientation_.x() = msg->pose.pose.orientation.x;
    orientation_.y() = msg->pose.pose.orientation.y;
    orientation_.z() = msg->pose.pose.orientation.z;
    orientation_.w() = msg->pose.pose.orientation.w;

    // ROS_INFO_STREAM("ground_velocity: " << ground_velocity_.x() << "\t" << ground_velocity_.y() << "\t" << ground_velocity_.z());
}

void WindEstimatorNode::so3cmdCallback(const kr_mav_msgs::SO3Command::ConstPtr& msg)
{
    /*
    Process new SO3Command messages and retrieve cmd_thrust from that. 
    */

    // Convert orientation to Eigen quaternion
    Eigen::Quaterniond q_eigen(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    // Convert quaternion to rotation matrix
    Eigen::Matrix3d R = q_eigen.toRotationMatrix();

    // Compute body z-axis (b3 = R * [0, 0, 1])
    Eigen::Vector3d b3 = R.col(2);

    // Extract force vector
    const auto& f = msg->force;
    Eigen::Vector3d force(f.x, f.y, f.z);

    // Project force onto b3 direction (dot product)
    cmd_thrust_ = force.dot(b3);

    // ROS_INFO_STREAM("cmd_thrust: " << cmd_thrust_);
}

void WindEstimatorNode::motorpwmCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg) 
{
    /*
    Process motor pwms 
    */

    motor_pwms_ <<  static_cast<double>(msg->values[0]),
                    static_cast<double>(msg->values[1]),
                    static_cast<double>(msg->values[2]),
                    static_cast<double>(msg->values[3]);

    // ROS_INFO_STREAM("motor_pwms: " << motor_pwms_[0] << "\t" << motor_pwms_[1] << "\t" << motor_pwms_[2] << "\t" << motor_pwms_[3]);
}

void WindEstimatorNode::vbatCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg) 
{
    /*
    Process vbat information coming from the Crazyflie 
    */
   vbat_ = msg->values[0];

//    ROS_INFO_STREAM("vbat: " << vbat_);
}

void WindEstimatorNode::publishWindEstimate(const ros::TimerEvent&)
{
    ROS_INFO_STREAM("Running publishWindEstimate()");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_estimator_node");
    ros::NodeHandle nh; // Create a NodeHandle to access parameters and namespaces
    std::string ns = nh.getNamespace(); // Get namespace of the node

    ROS_INFO_STREAM("Setting up wind estimator node...");
    WindEstimatorNode wind_estimator_node(ns);

    // Retrieve the frequency from the parameter server, with a default value if not set.
    ros::NodeHandle pnh("~");
    float freq;
    pnh.param("estimator_freq", freq, 100.0f);

    // Set up a timer to produce measurements at a regular rate. 
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / freq), &WindEstimatorNode::publishWindEstimate, &wind_estimator_node);

    // Spin!
    ros::spin();
    
    return 0;
}