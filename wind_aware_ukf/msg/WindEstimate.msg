wind_aware_ukf/MotorSpeeds motor_speeds     # Custom, because m1–m4 are specific
geometry_msgs/Quaternion orientation        # quaternion orientation
geometry_msgs/Vector3 body_rates            # roll_rate, pitch_rate, yaw_rate

geometry_msgs/Vector3 ground_velocity       # ground velocity in the world frame
geometry_msgs/Vector3 wind_velocity         # wind velocity in the world frame

float32 thrust_coeff_norm                   # Normalized thrust coefficient

geometry_msgs/Vector3 accel_bias            # accelerometer biases in body frame