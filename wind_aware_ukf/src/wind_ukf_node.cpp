#include <wind_aware_ukf/wind_ukf_node.hpp>

WindEstimatorNode::WindEstimatorNode(std::string ns, double dt, double mass): nh_(ns), 
    filter_initialized_(false),
    imu_received_(false), odom_received_(false), so3cmd_received_(false), motorpwm_received_(false), vbat_received_(false),
    estimator_(dt, mass, 9.81, 3.49e-08, 2.097e-6, 1.339e-5, 5.74e-4, 0.05)
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
    wind_estimate_pub_ = nh_.advertise<wind_aware_ukf::WindEstimateStamped>("wind_estimate", 1);
    wind_estimate_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("wind_vector_marker", 1);

    linear_acceleration_bias_.setZero();
    calibration_count_ = 0;
    calibration_count_thresh_ = 50;
    velocity_thresh_ = 0.1;
    acc_thresh_ = 0.75;
    
}

void WindEstimatorNode::initializeFilter()
{
    /*
    Initialize the UKF with appropriate parameters. 
    */

    // Initialize UKF parameters (initial state and covariance)
    // TODO: Remove these hardcoded values. 

    // Initialize x0
    x0 = Eigen::VectorXd::Zero(17);
    x0.segment<4>(0)  = motor_rpms_;
    x0.segment<3>(4)  = orientation_.toRotationMatrix().eulerAngles(2, 1, 0);
    x0.segment<3>(7)  = angular_velocity_;
    x0.segment<3>(10) = ground_velocity_;
    x0.segment<3>(13) = Eigen::Vector3d::Constant(1e-5);
    x0(16) = estimator_.keta_ / 1e-8;

    // Initialize Q
    Q = Eigen::MatrixXd::Zero(17, 17);
    Q.diagonal().segment(0, 4).setConstant(1.0);        // Motor speeds
    Q.diagonal().segment(4, 3).setConstant(0.5);        // Euler angles
    Q.diagonal().segment(7, 3).setConstant(0.1);        // body rates
    Q.diagonal().segment(10, 3).setConstant(0.1);       // Ground velocity (body frame)
    Q.diagonal().segment(13, 3).setConstant(0.1);       // Wind velocity (body frame)
    Q.diagonal()(16) = 1e-5;                            // Thrust coefficient normalized

    // Initialize R
    R = Eigen::MatrixXd::Zero(13, 13);
    R.diagonal().segment(0, 3).setConstant(0.010);                  // Euler angles
    R.diagonal().segment(3, 3).setConstant(0.5);                    // Body rates
    R.diagonal().segment(6, 3).setConstant(0.01);                   // Ground velocity 
    double val = 0.1 * std::sqrt(100.0 / 2.0) * std::pow(0.38, 2); 
    R.diagonal().segment(9, 3).setConstant(10*val);                 // Accelerometer
    R.diagonal()(12) = 10.0;                                         // Command thrust

    // Initialize P0
    P0 = Eigen::MatrixXd::Zero(17, 17);
    P0.diagonal().segment(0, 4).setConstant(2.5);       // Motor speeds
    P0.diagonal().segment(4, 13).setConstant(0.5);      // All other states
    P0.diagonal()(16) = 1e-3;                           // Thrust coeff norm

    // Initialize weight spread param. 
    estimator_.wo = 0.5;  // A little more conservative

    estimator_.Q = Q;
    estimator_.R = R;
    estimator_.initialize_state(x0, P0);

}

void WindEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    /*
    Process IMU data coming from the Crazyflie 
    */

    linear_acceleration_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    angular_velocity_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    if(calibration_count_ <= calibration_count_thresh_)
    {

        if ((ground_velocity_.norm() <= velocity_thresh_) && ((linear_acceleration_.norm() - estimator_.g_) <= acc_thresh_) && (cmd_thrust_ > 0.05))
        {
            ROS_INFO_STREAM("Accelerometer calibration step " << calibration_count_ << " out of " << calibration_count_thresh_);

            // Accumulate accelerometer samples to estimate bias
            linear_acceleration_.z() -= estimator_.g_;  // Subtract off gravity in the z axis (ONLY DURING CALIBRATION)
            linear_acceleration_bias_sum_ += linear_acceleration_;
            calibration_count_++;

            // Update average bias
            if (calibration_count_ > 0) // prevent div 0 error
            {
                linear_acceleration_bias_ = linear_acceleration_bias_sum_ / calibration_count_;
            }
        }
    } else { // Calibrated!

        if (!imu_received_)
        {
            ROS_INFO_STREAM("Accelerometer calibrated. bias = " << linear_acceleration_bias_.transpose() );
        }
        // Apply bias correction before sending to estimator
        linear_acceleration_ -= linear_acceleration_bias_;

        // Send measurements to estimator
        estimator_.new_observation(3, angular_velocity_[0]);
        estimator_.new_observation(4, angular_velocity_[1]);
        estimator_.new_observation(5, angular_velocity_[2]);
        estimator_.new_observation(9, linear_acceleration_[0]);
        estimator_.new_observation(10, linear_acceleration_[1]);
        estimator_.new_observation(11, linear_acceleration_[2]);

        imu_received_ = true;

    } 

    // ROS_INFO_STREAM("imu_acc: " << linear_acceleration_.x() << "\t" << linear_acceleration_.y() << "\t" << linear_acceleration_.z());
}

void WindEstimatorNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    /*
    Process odometry which contains the ground velocity estimate.
    */

    // Orientation
    orientation_.x() = msg->pose.pose.orientation.x;
    orientation_.y() = msg->pose.pose.orientation.y;
    orientation_.z() = msg->pose.pose.orientation.z;
    orientation_.w() = msg->pose.pose.orientation.w;

    // Ground velocity (in world frame)
    ground_velocity_.x() = msg->twist.twist.linear.x;
    ground_velocity_.y() = msg->twist.twist.linear.y;
    ground_velocity_.z() = msg->twist.twist.linear.z;

    // Convert orientation to Euler angles (in radians, zyx convention)
    Eigen::Vector3d euler = orientation_.toRotationMatrix().eulerAngles(2, 1, 0);

    // Convert ground velocity to body frame. 
    Eigen::Vector3d ground_velocity_body = orientation_.inverse() * ground_velocity_;

    // Send measurements to estimator. 
    estimator_.new_observation(0, euler[0]); // yaw
    estimator_.new_observation(1, euler[1]); // pitch
    estimator_.new_observation(2, euler[2]); // roll
    estimator_.new_observation(6, ground_velocity_body.x());
    estimator_.new_observation(7, ground_velocity_body.y());
    estimator_.new_observation(8, ground_velocity_body.z());

    odom_received_ = true;

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

    // Send measurement to estimator
    estimator_.new_observation(12, cmd_thrust_/estimator_.mass_);

    so3cmd_received_ = true;

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

    // Compute motor rpms using a mapping. In this case we can use the battery compensated model. 
    // TODO: Remove these hard coded coefficients and move them to rosparams!
    motor_rpms_ = pwmToMotorSpeedsBatCompensated(motor_pwms_, vbat_, Eigen::Vector3d(4.3034, 0.759, 10000), MotorSpeedUnits::RAD_PER_SEC);

    // // Send measurement to estimator  TODO: Note the hardcoded 1e3, please remove!!
    // estimator_.new_observation(0, motor_rpms_[0]/1e3);
    // estimator_.new_observation(1, motor_rpms_[1]/1e3);
    // estimator_.new_observation(2, motor_rpms_[2]/1e3);
    // estimator_.new_observation(3, motor_rpms_[3]/1e3);

    estimator_.set_cmd_motor_speeds(motor_rpms_);

    motorpwm_received_ = true;

    // ROS_INFO_STREAM("motor_rpms: " << motor_rpms_[0] << "\t" << motor_rpms_[1] << "\t" << motor_rpms_[2] << "\t" << motor_rpms_[3]);
}

void WindEstimatorNode::vbatCallback(const crazyflie_driver::GenericLogData::ConstPtr& msg) 
{
    /*
    Process vbat information coming from the Crazyflie 
    */
   vbat_ = msg->values[0];

   vbat_received_ = true;

    // ROS_INFO_STREAM("vbat: " << vbat_);
}

void WindEstimatorNode::run(const ros::TimerEvent&)
{
    /*
    Main loop of the wind estimator. 
    */
    if(filter_initialized_) // If filter is ready...
    {
        estimator_.iterate();
        publishWindEstimate();
        publishWindVector();
    } else 
    { // Set filter initialization only when all sensors are available. 
        if(imu_received_ & odom_received_ & so3cmd_received_ & motorpwm_received_ & vbat_received_)
        {
            initializeFilter();
            filter_initialized_ = true;
        }
    }
    publishWindVector();
}

void WindEstimatorNode::publishWindEstimate()
{
    /*
    Publish the full wind estimate over ROS
    */

    wind_aware_ukf::WindEstimateStamped msg; 
    
    // Set header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = mav_name_.substr(mav_name_.find('/') + 1); // Extract after first '/'

    // Access state vector and covariance matrix
    const Eigen::VectorXd& x = estimator_.get_state();
    const Eigen::MatrixXd& P = estimator_.get_covariance();

    // Motor speeds
    msg.estimate.motor_speeds.m1 = x(0)*1e3; // TODO: Get rid of hardcoded motor speed magnitude
    msg.estimate.motor_speeds.m2 = x(1)*1e3; // TODO: Get rid of hardcoded motor speed magnitude
    msg.estimate.motor_speeds.m3 = x(2)*1e3; // TODO: Get rid of hardcoded motor speed magnitude
    msg.estimate.motor_speeds.m4 = x(3)*1e3; // TODO: Get rid of hardcoded motor speed magnitude

    // Orientation
    Eigen::AngleAxisd rollAngle(x(6), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(x(5), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(x(4), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    msg.estimate.orientation.x = q.x();
    msg.estimate.orientation.y = q.y();
    msg.estimate.orientation.z = q.z();
    msg.estimate.orientation.w = q.w();

    // Body rates
    msg.estimate.body_rates.x = x(7); // roll rate
    msg.estimate.body_rates.y = x(8); // pitch rate
    msg.estimate.body_rates.z = x(9); // yaw rate

    // Ground velocities
    Eigen::Vector3d ground_velocity_body(x(10), x(11), x(12));
    Eigen::Vector3d ground_velocity_world = orientation_* ground_velocity_body;
    msg.estimate.ground_velocity.x = ground_velocity_world.x();
    msg.estimate.ground_velocity.y = ground_velocity_world.y();
    msg.estimate.ground_velocity.z = ground_velocity_world.z();

    // Wind estimates
    Eigen::Vector3d wind_velocity_body(x(13), x(14), x(15));
    Eigen::Vector3d wind_velocity_world = orientation_* wind_velocity_body;
    msg.estimate.wind_velocity.x = wind_velocity_world.x();
    msg.estimate.wind_velocity.y = wind_velocity_world.y();
    msg.estimate.wind_velocity.z = wind_velocity_world.z();

    // Normalized thrust coefficient
    msg.estimate.thrust_coeff_norm = x(16);

    // Accel bias
    // msg.estimate.accel_bias.x = x(17);
    // msg.estimate.accel_bias.y = x(18);
    // msg.estimate.accel_bias.z = x(19);

    // Flatten covariance matrix
    msg.covariance.reserve(P.rows() * P.cols());
    for (int i = 0; i < P.rows(); ++i) {
        for (int j = 0; j < P.cols(); ++j) {
            msg.covariance.push_back(P(i, j));
        }
    }

    // Publish the message
    wind_estimate_pub_.publish(msg);
}

void WindEstimatorNode::publishWindVector()
{
    /*
    Publish the wind vector for visualization in RViz. 
    */

    visualization_msgs::Marker marker;

    // Header
    marker.header.frame_id = mav_name_.substr(mav_name_.find('/') + 1); // Extract frame_id
    marker.header.stamp = ros::Time::now();

    // Marker settings
    marker.ns = "wind_vector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Scale
    marker.scale.x = 0.05; // Arrow head size
    marker.scale.y = 0.1;  // Shaft diameter
    marker.scale.z = 0.15; // Shaft diameter

    // Color: purple (red + blue), fully opaque
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    // Orientation (identity quaternion)
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Wind vector
    const Eigen::VectorXd& x = estimator_.get_state();
    Eigen::Vector3d wind_velocity_body(x(13), x(14), x(15));

    // Points for the arrow: start at origin, end at wind vector tip
    geometry_msgs::Point start, end;
    start.x = start.y = start.z = 0.0;
    end.x = wind_velocity_body.x();
    end.y = wind_velocity_body.y();
    end.z = wind_velocity_body.z();

    marker.points.push_back(start);
    marker.points.push_back(end);

    // Publish the marker
    wind_estimate_marker_pub_.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_estimator_node");

    ros::NodeHandle nh("~"); // Create a NodeHandle to access parameters and namespaces
    std::string ns = ros::this_node::getNamespace(); // Get namespace of the node

    // Retrieve the params from the parameter server, with a default value if not set.
    float freq;
    nh.param("estimator_freq", freq, 100.0f);
    float mass;
    nh.param("mass", mass, 0.040f);

    ROS_INFO_STREAM("Setting up wind estimator node...");
    WindEstimatorNode wind_estimator_node(ns, 1.0/freq, mass);

    // Set up a timer to produce measurements at a regular rate. 
    ros::NodeHandle node_nh(ns);
    ros::Timer timer = node_nh.createTimer(ros::Duration(1.0 / freq), &WindEstimatorNode::run, &wind_estimator_node);

    // Spin!
    ros::spin();
    
    return 0;
}